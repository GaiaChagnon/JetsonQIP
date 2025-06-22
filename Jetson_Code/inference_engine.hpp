#ifndef INFERENCE_ENGINE_HPP
#define INFERENCE_ENGINE_HPP

// Conditional compilation for CUDA/TensorRT availability
#ifdef __has_include
    #if __has_include(<cuda_runtime_api.h>) && __has_include(<NvInfer.h>)
        #define CUDA_TENSORRT_AVAILABLE 1
        #include <NvInfer.h>
        #include <NvInferRuntime.h>
        #include <cuda_runtime_api.h>
    #else
        #define CUDA_TENSORRT_AVAILABLE 0
        // Mock definitions for compilation without CUDA/TensorRT
        namespace nvinfer1 {
            class ILogger { 
            public: 
                enum class Severity { kINTERNAL_ERROR, kERROR, kWARNING, kINFO, kVERBOSE };
                virtual void log(Severity, const char*) noexcept = 0; 
                virtual ~ILogger() = default; 
            };
            class IRuntime { public: virtual void destroy() = 0; protected: virtual ~IRuntime() = default; };
            class ICudaEngine { public: virtual void destroy() = 0; protected: virtual ~ICudaEngine() = default; };
            class IExecutionContext { public: virtual void destroy() = 0; protected: virtual ~IExecutionContext() = default; };
        }
        typedef void* cudaStream_t;
        typedef int cudaError_t;
        #define cudaSuccess 0
        inline const char* cudaGetErrorString(cudaError_t) { return "CUDA not available"; }
    #endif
#else
    // Fallback for older compilers
    #define CUDA_TENSORRT_AVAILABLE 0
    namespace nvinfer1 {
        class ILogger { 
        public: 
            enum class Severity { kINTERNAL_ERROR, kERROR, kWARNING, kINFO, kVERBOSE };
            virtual void log(Severity, const char*) noexcept = 0; 
            virtual ~ILogger() = default; 
        };
        class IRuntime { public: virtual void destroy() = 0; protected: virtual ~IRuntime() = default; };
        class ICudaEngine { public: virtual void destroy() = 0; protected: virtual ~ICudaEngine() = default; };
        class IExecutionContext { public: virtual void destroy() = 0; protected: virtual ~IExecutionContext() = default; };
    }
    typedef void* cudaStream_t;
    typedef int cudaError_t;
    #define cudaSuccess 0
    inline const char* cudaGetErrorString(cudaError_t) { return "CUDA not available"; }
#endif

#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <cmath>

// --- CUDA Error Checking Macro ---
#define CHECK_CUDA(call)                                             \
    do {                                                             \
        cudaError_t status = call;                                   \
        if (status != cudaSuccess) {                                 \
            throw std::runtime_error(cudaGetErrorString(status));    \
        }                                                            \
    } while (0)

// --- Logger Class for TensorRT ---
class Logger : public nvinfer1::ILogger {
public:
    void log(nvinfer1::ILogger::Severity severity, const char* msg) noexcept override;
};

// --- RAII Wrappers for TensorRT and CUDA ---
template <typename T>
        struct TRTDestroy {
            void operator()(T* obj) const noexcept { 
                if (obj) {
                    obj->destroy();  // Use destroy() method for mock classes
                }
            }
        };

template <typename T>
using TRTUniquePtr = std::unique_ptr<T, TRTDestroy<T>>;

struct CudaDeviceDeleter {
    void operator()(void* ptr) const noexcept;
};
using CudaDeviceUniquePtr = std::unique_ptr<void, CudaDeviceDeleter>;

struct PinnedHostDeleter {
    void operator()(void* ptr) const noexcept;
};
using PinnedHostUniquePtr = std::unique_ptr<void, PinnedHostDeleter>;

struct StreamDeleter {
    void operator()(cudaStream_t stream) const noexcept;
};
using StreamUniquePtr = std::unique_ptr<std::remove_pointer<cudaStream_t>::type, StreamDeleter>;

// --- Inference Engine Class ---
class InferenceEngine {
public:
    InferenceEngine(const std::string& enginePath, int num_joints);

    std::vector<float> infer(const std::vector<float>& inputVecHost);

    size_t getInputNumElements() const;
    size_t getOutputNumElements() const;

private:
    int num_joints_;

    TRTUniquePtr<nvinfer1::IRuntime> runtime_;
    TRTUniquePtr<nvinfer1::ICudaEngine> engine_;
    TRTUniquePtr<nvinfer1::IExecutionContext> context_;

    CudaDeviceUniquePtr inputDevice_;
    CudaDeviceUniquePtr outputDevice_;
    PinnedHostUniquePtr hostInput_;
    PinnedHostUniquePtr hostOutput_;
    StreamUniquePtr stream_;

    std::vector<void*> bindings_;
    std::vector<float> outputVec_;

    int inputIndex_;
    int outputIndex_;
    size_t inputNumElements_;
    size_t outputNumElements_;
    size_t inputSizeInBytes_;
    size_t outputSizeInBytes_;

    bool validateInput(const std::vector<float>& input);
};

// --- Sensor Reading Function ---
void readRobotSensors(int num_joints, std::vector<float>& sensor_data);

// --- Signal Handling ---
extern std::atomic<bool> g_stop_signal_received;
void signalHandler(int signum);

// --- CartPendulumState Class ---
class CartPendulumState {
public:
    using Vector = std::vector<float>;
    
    static constexpr std::size_t kMinJoints = 1;
    static constexpr std::size_t kMaxJoints = 4;

    explicit CartPendulumState(std::size_t numJoints = 1);
    explicit CartPendulumState(const Vector& v);

    std::size_t numJoints() const;
    Vector toVector() const;

    float cartPos() const;
    float cartVel() const;
    float jointPos(std::size_t j) const;
    float jointVel(std::size_t j) const;

    void cartPos(float x);
    void cartVel(float v);
    void jointPos(std::size_t j, float pos);
    void jointVel(std::size_t j, float vel);

    static std::size_t staticSize(std::size_t joints);
    static std::size_t vectorToJointCount(const Vector& v);

private:
    std::size_t numJoints_;
    Vector data_;
    
    static std::size_t clampNumJoints(std::size_t j);
    std::size_t indexPos(std::size_t j) const;
    std::size_t indexVel(std::size_t j) const;
};

#endif // INFERENCE_ENGINE_HPP
