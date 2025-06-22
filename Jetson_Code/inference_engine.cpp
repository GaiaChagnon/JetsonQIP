#include "inference_engine.hpp"
#include <fstream>
#include <iostream>
#include <numeric>
#include <cstring>
#include <cmath>

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <cstddef>   // std::size_t
#include <iostream>

// Logger Implementation
void Logger::log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept {
    if (severity <= nvinfer1::ILogger::Severity::kWARNING) {
        std::cerr << "[TensorRT] ";
        switch (severity) {
            case nvinfer1::ILogger::Severity::kINTERNAL_ERROR: std::cerr << "INTERNAL_ERROR: "; break;
            case nvinfer1::ILogger::Severity::kERROR:          std::cerr << "ERROR: "; break;
            case nvinfer1::ILogger::Severity::kWARNING:        std::cerr << "WARNING: "; break;
            default: break;
        }
        std::cerr << msg << std::endl;
    }
}

#if CUDA_TENSORRT_AVAILABLE
// CUDA RAII implementations
void CudaDeviceDeleter::operator()(void* ptr) const noexcept {
    if (ptr) cudaFree(ptr);
}

void PinnedHostDeleter::operator()(void* ptr) const noexcept {
    if (ptr) cudaFreeHost(ptr);
}

void StreamDeleter::operator()(cudaStream_t stream) const noexcept {
    if (stream) cudaStreamDestroy(stream);
}
#else
// Mock implementations when CUDA is not available
void CudaDeviceDeleter::operator()(void* ptr) const noexcept {
    (void)ptr; // Suppress unused parameter warning
}

void PinnedHostDeleter::operator()(void* ptr) const noexcept {
    delete[] static_cast<char*>(ptr);
}

void StreamDeleter::operator()(cudaStream_t stream) const noexcept {
    (void)stream; // Suppress unused parameter warning
}
#endif

// InferenceEngine constructor implementation
InferenceEngine::InferenceEngine(const std::string& enginePath, int num_joints)
    : num_joints_(num_joints) {

#if CUDA_TENSORRT_AVAILABLE
    Logger logger;
    std::ifstream file(enginePath, std::ios::binary);
    if (!file) throw std::runtime_error("Engine file not found: " + enginePath);

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0, file.beg);

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();

    runtime_ = TRTUniquePtr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger));
    engine_ = TRTUniquePtr<nvinfer1::ICudaEngine>(runtime_->deserializeCudaEngine(buffer.data(), size));
    context_ = TRTUniquePtr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());

    inputIndex_ = engine_->getBindingIndex("input");
    outputIndex_ = engine_->getBindingIndex("action");

    auto inputDims = engine_->getBindingDimensions(inputIndex_);
    inputNumElements_ = std::accumulate(inputDims.d, inputDims.d + inputDims.nbDims, 1LL, std::multiplies<int64_t>());
    inputSizeInBytes_ = inputNumElements_ * sizeof(float);

    auto outputDims = engine_->getBindingDimensions(outputIndex_);
    outputNumElements_ = std::accumulate(outputDims.d, outputDims.d + outputDims.nbDims, 1LL, std::multiplies<int64_t>());
    outputSizeInBytes_ = outputNumElements_ * sizeof(float);

    void* tempInputDevice;
    void* tempOutputDevice;
    CHECK_CUDA(cudaMalloc(&tempInputDevice, inputSizeInBytes_));
    CHECK_CUDA(cudaMalloc(&tempOutputDevice, outputSizeInBytes_));

    inputDevice_.reset(tempInputDevice);
    outputDevice_.reset(tempOutputDevice);

    void* tempHostInput;
    void* tempHostOutput;
    CHECK_CUDA(cudaMallocHost(&tempHostInput, inputSizeInBytes_));
    CHECK_CUDA(cudaMallocHost(&tempHostOutput, outputSizeInBytes_));

    hostInput_.reset(tempHostInput);
    hostOutput_.reset(tempHostOutput);

    cudaStream_t tempStream;
    CHECK_CUDA(cudaStreamCreate(&tempStream));
    stream_.reset(tempStream);

    bindings_.resize(engine_->getNbBindings());
    bindings_[inputIndex_] = inputDevice_.get();
    bindings_[outputIndex_] = outputDevice_.get();

    outputVec_.resize(outputNumElements_);
#else
    // Mock implementation when CUDA/TensorRT is not available
    (void)enginePath; // Suppress unused parameter warning
    inputNumElements_ = num_joints * 2 + 2; // cart pos/vel + joint pos/vel pairs
    outputNumElements_ = 1; // single control output
    outputVec_.resize(outputNumElements_, 0.0f);
    std::cerr << "Warning: CUDA/TensorRT not available. Using mock inference engine." << std::endl;
#endif
}

bool InferenceEngine::validateInput(const std::vector<float>& input) {
    // Check size
    if (input.size() != getInputNumElements()) {
        std::cerr << "Input size mismatch: expected " 
                  << getInputNumElements() 
                  << ", got " << input.size() << std::endl;
        return false;
    }
    
    // Check for NaN or infinite values
    for (size_t i = 0; i < input.size(); ++i) {
        if (std::isnan(input[i]) || std::isinf(input[i])) {
            std::cerr << "Invalid value at index " << i 
                      << ": " << input[i] << std::endl;
            return false;
        }
    }
    
    // Check if vector is empty
    if (input.empty()) {
        std::cerr << "Input vector is empty" << std::endl;
        return false;
    }
    
    return true;
}

// Inference function implementation
std::vector<float> InferenceEngine::infer(const std::vector<float>& inputVecHost) {
    
    if (!validateInput(inputVecHost)) {
        std::cerr << "Invalid input data. Aborting inference." << std::endl;
        return {};
    }

#if CUDA_TENSORRT_AVAILABLE
    std::memcpy(hostInput_.get(), inputVecHost.data(), inputSizeInBytes_);

    CHECK_CUDA(cudaMemcpyAsync(inputDevice_.get(), hostInput_.get(), inputSizeInBytes_, cudaMemcpyHostToDevice, stream_.get()));

    context_->enqueueV2(bindings_.data(), stream_.get(), nullptr);

    CHECK_CUDA(cudaMemcpyAsync(hostOutput_.get(), outputDevice_.get(), outputSizeInBytes_, cudaMemcpyDeviceToHost, stream_.get()));

    CHECK_CUDA(cudaStreamSynchronize(stream_.get()));

    std::memcpy(outputVec_.data(), hostOutput_.get(), outputSizeInBytes_);
#else
    // Mock inference - simple proportional control based on first input (position error)
    (void)inputVecHost; // Suppress unused parameter warning
    if (!inputVecHost.empty()) {
        outputVec_[0] = -0.1f * inputVecHost[0]; // Simple proportional control
    } else {
        outputVec_[0] = 0.0f;
    }
#endif

    return outputVec_;
}

size_t InferenceEngine::getInputNumElements() const { return inputNumElements_; }
size_t InferenceEngine::getOutputNumElements() const { return outputNumElements_; }

// Dummy sensor data function implementation
void readRobotSensors(int num_joints, std::vector<float>& sensor_data) {
    const size_t total_elements = (num_joints * 2) + 2;
    sensor_data.resize(total_elements);
    static float counter = 0.0f;
    for (int i = 0; i < num_joints; ++i) {
        sensor_data[i * 2] = std::sin(counter + i);
        sensor_data[i * 2 + 1] = std::cos(counter + i);
    }
    sensor_data[num_joints * 2] = std::sin(counter);
    sensor_data[num_joints * 2 + 1] = std::cos(counter);
    counter += 0.01f;
}

// Signal handling implementation
std::atomic<bool> g_stop_signal_received{false};
void signalHandler(int signum) {
    (void)signum; // Suppress unused parameter warning
    g_stop_signal_received.store(true);
}

// CartPendulumState implementation
CartPendulumState::CartPendulumState(std::size_t numJoints)
    : numJoints_(clampNumJoints(numJoints)),
      data_(staticSize(numJoints_), 0.0f)
{}

CartPendulumState::CartPendulumState(const Vector& v)
    : CartPendulumState(vectorToJointCount(v))
{
    data_ = v;
}

std::size_t CartPendulumState::numJoints() const { return numJoints_; }
CartPendulumState::Vector CartPendulumState::toVector() const { return data_; }

float CartPendulumState::cartPos() const { return data_[0]; }
float CartPendulumState::cartVel() const { return data_[1]; }
float CartPendulumState::jointPos(std::size_t j) const { return data_[indexPos(j)]; }
float CartPendulumState::jointVel(std::size_t j) const { return data_[indexVel(j)]; }

void CartPendulumState::cartPos(float v) { data_[0] = v; }
void CartPendulumState::cartVel(float v) { data_[1] = v; }
void CartPendulumState::jointPos(std::size_t j, float v) { data_[indexPos(j)] = v; }
void CartPendulumState::jointVel(std::size_t j, float v) { data_[indexVel(j)] = v; }

std::size_t CartPendulumState::staticSize(std::size_t joints) { return 2 + 2 * joints; }

std::size_t CartPendulumState::vectorToJointCount(const Vector& v)
{
    const auto sz = v.size();
    if (sz < 4 || sz > 10 || (sz - 2) % 2 != 0)
        throw std::invalid_argument("CartPendulumState – vector size must be 2+2·J with 1≤J≤4");
    std::size_t joints = (sz - 2) / 2;
    clampNumJoints(joints);
    return joints;
}

std::size_t CartPendulumState::clampNumJoints(std::size_t j)
{
    if (j < kMinJoints || j > kMaxJoints)
        throw std::invalid_argument("CartPendulumState – joints must be between 1 and 4");
    return j;
}

std::size_t CartPendulumState::indexPos(std::size_t j) const { return 2 + 2 * j; }
std::size_t CartPendulumState::indexVel(std::size_t j) const { return 3 + 2 * j; }