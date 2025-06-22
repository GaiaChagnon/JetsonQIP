/*  ─────────────────────────────────────────────────────────────────────────
    Pendulum Balance Controller - Advanced Real-Time Control System

PRIMARY PURPOSE
---------------
An advanced real-time pendulum balance controller—an inverted-pendulum robot that keeps one or more pendulum arms upright while moving a cart along a linear track.

SYSTEM ARCHITECTURE
===================

1. Hardware Setup
   • Cart on linear track: 1.5 m travel range  
   • Stepper motor: 1500 steps/rev, 31.45 mm wheel diameter  
   • Pendulum arms: 1–4 arms (configurable) mounted on cart  
   • Wireless sensors: STM32 + encoder + nRF24L01+ on each arm  
   • Main controller: NVIDIA Jetson Nano

2. Control-System Components
   • motor_controller.cpp / .hpp  
     – 75 kHz step pulses (3000 RPM max)  
     – ±50 m/s² acceleration limits  
     – Boundary safety and emergency stop  
     – Cycle-accurate timing (ARM cycle counter)  
     – Runs on dedicated CPU core 2  

   • communication_controller.cpp / .hpp  
     – Bit-banged SPI at 10 MHz to nRF24L01+ radios  
     – Supports up to 4 pendulum arms  
     – Reads encoder data at 150 Hz  
     – CRC-4 error checking and recovery  
     – Custom wireless packet format  

   • inference_engine.cpp / .hpp  
     – Loads TensorRT model file pendulum.trt  
     – GPU-accelerated inference on Jetson Nano  
     – Builds state vector (cart pos/vel + arm angles/vels)  
     – Returns cart-acceleration command  

   • pendulum_balance_controller.cpp  
     – 150 Hz control loop (6.67 ms period)  
     – Multi-threaded: motor control on its own thread  
     – Rust-style Result<T> error handling  
     – Real-time metrics and logging  
     – Fallback control when AI inference fails

CONTROL-LOOP OPERATION (150 Hz)
-------------------------------
1. Sensor collection  
   – Cart position & velocity from motor controller  
   – Pendulum angles & velocities via nRF24L01+ radios  
2. AI inference  
   – Current state → TensorRT → optimal cart acceleration  
   – Validate output, handle errors  
3. Control application  
   – Send acceleration to motor controller (step pulses)  
   – Apply safety limits and boundary checks  
4. Performance monitoring  
   – Measure timing, log events, keep real-time deadlines

WHAT THE SYSTEM DOES
--------------------
• Balances one to four pendulum arms upright in real time  
• Moves the cart to target positions while balancing  
• Adapts via neural-network inference  
• Performs emergency stop and PID fallback on failure

REAL-WORLD USES
---------------
• Control-systems research  
• Robotics education demos  
• AI/ML controller validation  
• Precision mechatronics showcases

ADVANCED FEATURES
-----------------
Performance
  – SCHED_FIFO real-time threads  
  – CPU affinity (dedicated core)  
  – RAII and smart pointers for memory safety  
  – Lock-free atomics where possible

Robustness
  – Graceful error recovery  
  – Physical boundary protection  
  – Wireless packet-loss handling  
  – PID fallback controller

Monitoring & Debug
  – Thread-safe logging with timestamps  
  – Live timing metrics  
  – Configurable verbosity levels  
  – CSV data logging for analysis

EXPECTED RUN-TIME BEHAVIOUR
---------------------------
1. Initialise hardware (motor, SPI, GPIO, radios)  
2. Calibrate timers and load TensorRT model  
3. Start 150 Hz control loop  
4. Continuously balance pendulum(s) and move cart  
5. Log performance and accept user commands  
6. Graceful shutdown on Ctrl+C
    
    Build Instructions:
    
    g++ -std=c++17 -O3 -Wall -Wextra -pthread \
    pendulum_balance_controller.cpp \
    motor_controller.cpp \
    inference_engine.cpp \
    communication_controller.cpp \
    -ljetgpio -lrt \
    -L/usr/local/cuda-10.2/lib64 \
    -L/usr/local/cuda-10.2/targets/aarch64-linux/lib \
    -lcuda -lcudart -lnvinfer -lnvinfer_plugin \
    -o pendulum_balance_controller
            
    Run (as root on Jetson for GPIO access):
        sudo ./pendulum_balance_controller
    ───────────────────────────────────────────────────────────────────────── */

#include "motor_controller.hpp"
#include "inference_engine.hpp"
#include "communication_controller.hpp"

#include <exception>
#include <stdexcept>
#include <fstream>
#include <csignal>
#include <array>
#include <functional>
#include <optional>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <memory>
#include <cstring>  // For std::memcpy

namespace pendulum_controller {

// Configuration management with structured approach
struct SystemConfig {
    struct Hardware {
        std::string chip_name{"gpiochip0"};
        unsigned step_pin{18};
        unsigned dir_pin{15};
        int active_arms_count{1};
        int motor_cpu_core{2};
    } hardware;
    
    struct ControlLoop {
        static constexpr double default_frequency_hz{150.0};
        std::chrono::microseconds cycle_period{std::chrono::microseconds(static_cast<long>(1000000.0 / default_frequency_hz))};
        double frequency_hz{default_frequency_hz};
        size_t max_consecutive_failures{5};
        std::chrono::milliseconds inference_timeout{100};
    } control_loop;
    
    struct Communication {
        std::array<std::array<uint8_t, 3>, 4> stm32_addresses{{
            {{0xD7, 0xD7, 0xD7}},
            {{0xA1, 0xA1, 0xA1}},
            {{0xB2, 0xB2, 0xB2}},
            {{0xC3, 0xC3, 0xC3}}
        }};
        std::chrono::milliseconds communication_timeout{50};
    } communication;
    
    struct Model {
        std::string model_path{"pendulum.trt"};
        bool enable_fallback_control{true};
    } model;
    
    struct Logging {
        bool enable_debug{false};
        bool enable_performance_logging{true};
        std::string log_file_path{"controller.log"};
    } logging;
};

// Thread-safe logging system
class Logger {
public:
    enum class Level { DEBUG, INFO, WARN, ERROR, FATAL };
    
    static Logger& instance() {
        static Logger logger;
        return logger;
    }
    
    void log(Level level, const std::string& message, 
             const char* file = __builtin_FILE(), 
             int line = __builtin_LINE()) {
        if (!should_log(level)) return;
        
        try {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
            
            std::ostringstream oss;
            oss << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
                << "." << std::setfill('0') << std::setw(3) << ms.count()
                << "] [" << level_to_string(level) << "] "
                << "[" << extract_filename(file) << ":" << line << "] "
                << message;
            
            std::string log_line = oss.str();
            
            // Thread-safe console output
            {
                std::lock_guard<std::mutex> lock(console_mutex_);
                if (level >= Level::ERROR) {
                    std::cerr << log_line << std::endl;
                } else {
                    std::cout << log_line << std::endl;
                }
            }
            
            // File logging (if enabled)
            if (file_logging_enabled_) {
                std::lock_guard<std::mutex> lock(file_mutex_);
                if (log_file_.is_open()) {
                    log_file_ << log_line << std::endl;
                    log_file_.flush();
                }
            }
        } catch (const std::exception&) {
            // Ignore logging errors to avoid infinite recursion
        }
    }
    
    void enable_file_logging(const std::string& path) {
        std::lock_guard<std::mutex> lock(file_mutex_);
        log_file_.open(path, std::ios::app);
        file_logging_enabled_ = log_file_.is_open();
    }
    
    void set_level(Level level) { min_level_ = level; }
    void enable_debug(bool enable) { debug_enabled_ = enable; }

private:
    Logger() = default;
    std::atomic<Level> min_level_{Level::INFO};
    std::atomic<bool> debug_enabled_{false};
    std::atomic<bool> file_logging_enabled_{false};
    std::ofstream log_file_;
    std::mutex console_mutex_;
    std::mutex file_mutex_;
    
    bool should_log(Level level) const {
        if (level == Level::DEBUG && !debug_enabled_) return false;
        return level >= min_level_;
    }
    
    static const char* level_to_string(Level level) {
        switch (level) {
            case Level::DEBUG: return "DEBUG";
            case Level::INFO:  return "INFO ";
            case Level::WARN:  return "WARN ";
            case Level::ERROR: return "ERROR";
            case Level::FATAL: return "FATAL";
            default: return "UNKNOWN";
        }
    }
    
    static const char* extract_filename(const char* path) {
        const char* filename = std::strrchr(path, '/');
        return filename ? filename + 1 : path;
    }
};

#define LOG_DEBUG(msg) Logger::instance().log(Logger::Level::DEBUG, msg, __FILE__, __LINE__)
#define LOG_INFO(msg)  Logger::instance().log(Logger::Level::INFO, msg, __FILE__, __LINE__)
#define LOG_WARN(msg)  Logger::instance().log(Logger::Level::WARN, msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) Logger::instance().log(Logger::Level::ERROR, msg, __FILE__, __LINE__)
#define LOG_FATAL(msg) Logger::instance().log(Logger::Level::FATAL, msg, __FILE__, __LINE__)

// Result type for robust error handling
template<typename T>
class Result {
public:
    static Result ok(T value) { return Result(std::move(value), true); }
    static Result error(std::string msg) { return Result(std::move(msg), false); }
    
    bool is_ok() const { return is_success_; }
    bool is_error() const { return !is_success_; }
    
    const T& value() const { 
        if (!is_success_) throw std::runtime_error("Accessing value of error result");
        return value_; 
    }
    
    const std::string& error() const {
        if (is_success_) throw std::runtime_error("Accessing error of ok result");
        return error_;
    }

private:
    Result(T val, bool success) : value_(std::move(val)), is_success_(success) {}
    Result(std::string err, bool success) : error_(std::move(err)), is_success_(success) {}
    
    T value_;
    std::string error_;
    bool is_success_;
};

// Specialization for void
template<>
class Result<void> {
public:
    static Result ok() { return Result(true); }
    static Result error(std::string msg) { return Result(std::move(msg)); }
    
    bool is_ok() const { return is_success_; }
    bool is_error() const { return !is_success_; }
    
    const std::string& error() const { return error_; }

private:
    explicit Result(bool success) : is_success_(success) {}
    explicit Result(std::string err) : error_(std::move(err)), is_success_(false) {}
    
    std::string error_;
    bool is_success_;
};

// Performance monitoring
class PerformanceMonitor {
public:
    struct Metrics {
        std::chrono::microseconds inference_time{0};
        std::chrono::microseconds communication_time{0};
        std::chrono::microseconds total_cycle_time{0};
        size_t successful_cycles{0};
        size_t failed_cycles{0};
        size_t communication_failures{0};
        size_t inference_failures{0};
    };
    
    void record_inference_time(std::chrono::microseconds time) {
        metrics_.inference_time = time;
    }
    
    void record_communication_time(std::chrono::microseconds time) {
        metrics_.communication_time = time;
    }
    
    void record_cycle_completion(std::chrono::microseconds total_time) {
        metrics_.total_cycle_time = total_time;
        ++metrics_.successful_cycles;
    }
    
    void record_failure(const std::string& type) {
        ++metrics_.failed_cycles;
        if (type == "communication") {
            ++metrics_.communication_failures;
        } else if (type == "inference") {
            ++metrics_.inference_failures;
        }
    }
    
    void log_performance_summary() const {
        LOG_INFO("Performance Summary:");
        LOG_INFO("  Successful cycles: " + std::to_string(metrics_.successful_cycles));
        LOG_INFO("  Failed cycles: " + std::to_string(metrics_.failed_cycles));
        LOG_INFO("  Communication failures: " + std::to_string(metrics_.communication_failures));
        LOG_INFO("  Inference failures: " + std::to_string(metrics_.inference_failures));
        LOG_INFO("  Last inference time: " + std::to_string(metrics_.inference_time.count()) + " μs");
        LOG_INFO("  Last communication time: " + std::to_string(metrics_.communication_time.count()) + " μs");
        LOG_INFO("  Last total cycle time: " + std::to_string(metrics_.total_cycle_time.count()) + " μs");
    }

private:
    Metrics metrics_;
};

// Safe signal handling
class SignalHandler {
public:
    static SignalHandler& instance() {
        static SignalHandler handler;
        return handler;
    }
    
    void setup() {
        std::signal(SIGINT, signal_callback);
        std::signal(SIGTERM, signal_callback);
        LOG_INFO("Signal handlers registered");
    }
    
    bool should_shutdown() const {
        return shutdown_requested_.load();
    }
    
    void request_shutdown() {
        shutdown_requested_.store(true);
    }

private:
    static std::atomic<bool> shutdown_requested_;
    
    static void signal_callback(int /*signal*/) {
        shutdown_requested_.store(true);
    }
};

std::atomic<bool> SignalHandler::shutdown_requested_{false};

// Main controller class with proper RAII and error handling
class PendulumBalanceController {
public:
    explicit PendulumBalanceController(SystemConfig config) 
        : config_(std::move(config)) {}
    
    ~PendulumBalanceController() {
        if (initialized_) {
            shutdown();
        }
    }
    
    // Non-copyable, non-movable for safety
    PendulumBalanceController(const PendulumBalanceController&) = delete;
    PendulumBalanceController& operator=(const PendulumBalanceController&) = delete;
    PendulumBalanceController(PendulumBalanceController&&) = delete;
    PendulumBalanceController& operator=(PendulumBalanceController&&) = delete;
    
    Result<void> initialize() {
        try {
            LOG_INFO("Initializing Pendulum Balance Controller");
            
            // Setup logging
            if (config_.logging.enable_debug) {
                Logger::instance().enable_debug(true);
            }
            if (!config_.logging.log_file_path.empty()) {
                Logger::instance().enable_file_logging(config_.logging.log_file_path);
            }
            
            // Initialize motor controller
            motor_controller_ = std::make_unique<MotorController>(
                config_.hardware.chip_name.c_str(),
                config_.hardware.step_pin,
                config_.hardware.dir_pin
            );
            
            motor_controller_->start(config_.hardware.motor_cpu_core);
            LOG_INFO("Motor controller initialized");
            
            // Initialize inference engine
            inference_engine_ = std::make_unique<InferenceEngine>(
                config_.model.model_path,
                config_.hardware.active_arms_count
            );
            LOG_INFO("Inference engine initialized");
            
            // Initialize state
            pendulum_state_ = std::make_unique<CartPendulumState>(
                config_.hardware.active_arms_count
            );
            LOG_INFO("Pendulum state initialized");
            
            // Initialize STM32 communication manager
            stm32_manager_ = std::make_unique<STM32Manager>();
            
            // Initialize SPI communication
            if (!SPI1_NRF::initialize()) {
                return Result<void>::error("Failed to initialize SPI communication");
            }
            LOG_INFO("SPI communication initialized");
            
            // Add STM32 devices for each active arm
            for (int i = 0; i < config_.hardware.active_arms_count; ++i) {
                STM32Config device_config;
                device_config.radio_address = config_.communication.stm32_addresses[i];
                device_config.rf_channel = 76; // Default channel
                
                if (!stm32_manager_->addDevice(i, device_config)) {
                    return Result<void>::error("Failed to add STM32 device " + std::to_string(i));
                }
                LOG_INFO("Added STM32 device " + std::to_string(i));
            }
            
            // Initialize all STM32 devices
            if (!stm32_manager_->initializeAll()) {
                LOG_WARN("Some STM32 devices failed to initialize - continuing anyway");
            }
            LOG_INFO("STM32 communication manager initialized");
            
            // Setup signal handling
            SignalHandler::instance().setup();
            
            initialized_ = true;
            LOG_INFO("Controller initialization complete");
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            std::string error_msg = "Initialization failed: " + std::string(e.what());
            LOG_FATAL(error_msg);
            return Result<void>::error(error_msg);
        }
    }
    
    Result<void> run() {
        if (!initialized_) {
            return Result<void>::error("Controller not initialized");
        }
        
        LOG_INFO("Starting control loop");
        consecutive_failures_ = 0;
        
        try {
            while (!SignalHandler::instance().should_shutdown()) {
                auto cycle_start = std::chrono::high_resolution_clock::now();
                
                auto result = execute_control_cycle();
                if (result.is_error()) {
                    handle_cycle_error(result.error());
                    if (consecutive_failures_ >= config_.control_loop.max_consecutive_failures) {
                        LOG_FATAL("Too many consecutive failures, shutting down");
                        break;
                    }
                } else {
                    consecutive_failures_ = 0;
                    auto cycle_end = std::chrono::high_resolution_clock::now();
                    auto cycle_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        cycle_end - cycle_start);
                    performance_monitor_.record_cycle_completion(cycle_time);
                }
                
                wait_for_next_cycle(cycle_start);
            }
            
        } catch (const std::exception& e) {
            LOG_FATAL("Unhandled exception in control loop: " + std::string(e.what()));
            return Result<void>::error("Control loop failed: " + std::string(e.what()));
        }
        
        return shutdown();
    }

private:
    SystemConfig config_;
    std::unique_ptr<MotorController> motor_controller_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    std::unique_ptr<CartPendulumState> pendulum_state_;
    std::unique_ptr<STM32Manager> stm32_manager_;
    PerformanceMonitor performance_monitor_;
    bool initialized_{false};
    size_t consecutive_failures_{0};
    
    Result<void> execute_control_cycle() {
        // 1. Collect sensor data
        auto sensor_result = collect_sensor_data();
        if (sensor_result.is_error()) {
            return Result<void>::error("Sensor collection failed: " + sensor_result.error());
        }
        
        // 2. Run inference
        auto inference_result = run_inference();
        if (inference_result.is_error()) {
            if (config_.model.enable_fallback_control) {
                LOG_WARN("Inference failed, using fallback control: " + inference_result.error());
                return execute_fallback_control();
            } else {
                return Result<void>::error("Inference failed: " + inference_result.error());
            }
        }
        
        // 3. Apply control actions
        auto control_result = apply_control_actions(inference_result.value());
        if (control_result.is_error()) {
            return Result<void>::error("Control application failed: " + control_result.error());
        }
        
        return Result<void>::ok();
    }
    
    Result<void> collect_sensor_data() {
        auto comm_start = std::chrono::high_resolution_clock::now();
        
        try {
            // Collect data from all active arms
            for (int i = 0; i < config_.hardware.active_arms_count; ++i) {
                auto result = collect_arm_data(i);
                if (result.is_error()) {
                    performance_monitor_.record_failure("communication");
                    return result;
                }
            }
            
            // Update cart position and velocity
            pendulum_state_->cartPos(motor_controller_->getPosition());
            pendulum_state_->cartVel(motor_controller_->curSpeed_m_s());
            
            auto comm_end = std::chrono::high_resolution_clock::now();
            auto comm_time = std::chrono::duration_cast<std::chrono::microseconds>(
                comm_end - comm_start);
            performance_monitor_.record_communication_time(comm_time);
            
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            return Result<void>::error("Sensor data collection exception: " + std::string(e.what()));
        }
    }
    
    Result<void> collect_arm_data(int arm_index) {
        if (arm_index >= config_.hardware.active_arms_count) {
            return Result<void>::error("Invalid arm index: " + std::to_string(arm_index));
        }
        
        try {
            // Get the STM32 device for this arm
            STM32Device* device = stm32_manager_->getDevice(arm_index);
            if (!device) {
                return Result<void>::error("STM32 device " + std::to_string(arm_index) + " not found");
            }
            
            // Read sensor data from the device
            STM32Device::SensorData sensor_data;
            if (!device->readSensorData(sensor_data)) {
                return Result<void>::error("Failed to read sensor data from STM32 #" + 
                                         std::to_string(arm_index));
            }
            
            // Check if data is valid
            if (!sensor_data.valid) {
                return Result<void>::error("Invalid sensor data from STM32 #" + 
                                         std::to_string(arm_index));
            }
            
            // Update pendulum state with the received data
            pendulum_state_->jointPos(arm_index, sensor_data.encoder_position);
            pendulum_state_->jointVel(arm_index, sensor_data.angular_velocity_rad_per_sec);
            
            LOG_DEBUG("Arm " + std::to_string(arm_index) + " data collected successfully: " +
                     "pos=" + std::to_string(sensor_data.encoder_position) + 
                     " vel=" + std::to_string(sensor_data.angular_velocity_rad_per_sec));
            
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            return Result<void>::error("Exception collecting arm " + std::to_string(arm_index) + 
                                     " data: " + std::string(e.what()));
        }
    }
    
    Result<std::vector<float>> run_inference() {
        auto inference_start = std::chrono::high_resolution_clock::now();
        
        try {
            std::vector<float> input_vector = pendulum_state_->toVector();
            std::vector<float> control_actions = inference_engine_->infer(input_vector);
            
            auto inference_end = std::chrono::high_resolution_clock::now();
            auto inference_time = std::chrono::duration_cast<std::chrono::microseconds>(
                inference_end - inference_start);
            performance_monitor_.record_inference_time(inference_time);
            
            // Validate output
            if (control_actions.size() != inference_engine_->getOutputNumElements()) {
                return Result<std::vector<float>>::error(
                    "Unexpected inference output size: " + 
                    std::to_string(control_actions.size()) + 
                    " (expected " + std::to_string(inference_engine_->getOutputNumElements()) + ")");
            }
            
            LOG_DEBUG("Inference completed in " + std::to_string(inference_time.count()) + " μs");
            return Result<std::vector<float>>::ok(std::move(control_actions));
            
        } catch (const std::exception& e) {
            performance_monitor_.record_failure("inference");
            return Result<std::vector<float>>::error("Inference exception: " + std::string(e.what()));
        }
    }
    
    Result<void> apply_control_actions(const std::vector<float>& control_actions) {
        if (control_actions.empty()) {
            return Result<void>::error("Empty control actions vector");
        }
        
        try {
            // Apply the primary control action (cart acceleration)
            motor_controller_->setTarget(control_actions[0]);
            
            LOG_DEBUG("Applied control action: " + std::to_string(control_actions[0]));
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            return Result<void>::error("Control application exception: " + std::string(e.what()));
        }
    }
    
    Result<void> execute_fallback_control() {
        try {
            // Simple fallback: try to stabilize by reducing cart acceleration
            float cart_velocity = pendulum_state_->cartVel();
            float fallback_action = -0.1f * cart_velocity;  // Simple proportional control
            
            motor_controller_->setTarget(fallback_action);
            LOG_DEBUG("Applied fallback control action: " + std::to_string(fallback_action));
            
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            return Result<void>::error("Fallback control exception: " + std::string(e.what()));
        }
    }
    
    void handle_cycle_error(const std::string& error) {
        ++consecutive_failures_;
        LOG_ERROR("Control cycle failed (attempt " + std::to_string(consecutive_failures_) + 
                 "/" + std::to_string(config_.control_loop.max_consecutive_failures) + 
                 "): " + error);
        performance_monitor_.record_failure("cycle");
    }
    
    void wait_for_next_cycle(std::chrono::high_resolution_clock::time_point cycle_start) {
        auto next_cycle_time = cycle_start + config_.control_loop.cycle_period;
        std::this_thread::sleep_until(next_cycle_time);
    }
    
    Result<void> shutdown() {
        LOG_INFO("Shutting down controller");
        
        try {
            if (motor_controller_) {
                motor_controller_->stop();
                LOG_INFO("Motor controller stopped");
            }
            
            if (stm32_manager_) {
                LOG_INFO("STM32 Manager status before shutdown:");
                LOG_INFO(stm32_manager_->getStatusReport());
                stm32_manager_.reset();
                LOG_INFO("STM32 communication manager stopped");
            }
            
            // Clean up SPI resources
            SPI1_NRF::terminate();
            LOG_INFO("SPI communication terminated");
            
            if (config_.logging.enable_performance_logging) {
                performance_monitor_.log_performance_summary();
            }
            
            initialized_ = false;
            LOG_INFO("Controller shutdown complete");
            return Result<void>::ok();
            
        } catch (const std::exception& e) {
            LOG_ERROR("Shutdown error: " + std::string(e.what()));
            return Result<void>::error("Shutdown failed: " + std::string(e.what()));
        }
    }
};

// Configuration loading with command-line support
Result<SystemConfig> load_configuration(int argc, char* argv[]) {
    SystemConfig config;
    
    // Simple command-line argument parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--debug") {
            config.logging.enable_debug = true;
        } else if (arg == "--model" && i + 1 < argc) {
            config.model.model_path = argv[++i];
        } else if (arg == "--frequency" && i + 1 < argc) {
            try {
                double freq = std::stod(argv[++i]);
                config.control_loop.frequency_hz = freq;
                config.control_loop.cycle_period = 
                    std::chrono::microseconds(static_cast<long>(1000000.0 / freq));
            } catch (const std::exception&) {
                return Result<SystemConfig>::error("Invalid frequency value");
            }
        } else if (arg == "--arms" && i + 1 < argc) {
            try {
                config.hardware.active_arms_count = std::stoi(argv[++i]);
                if (config.hardware.active_arms_count < 1 || 
                    config.hardware.active_arms_count > 4) {
                    return Result<SystemConfig>::error("Invalid arms count (must be 1-4)");
                }
            } catch (const std::exception&) {
                return Result<SystemConfig>::error("Invalid arms count value");
            }
        }
    }
    
    return Result<SystemConfig>::ok(std::move(config));
}

} // namespace pendulum_controller

// Main function with proper error handling and exit codes
int main(int argc, char* argv[]) {
    using namespace pendulum_controller;
    
    try {
        // Load configuration
        auto config_result = load_configuration(argc, argv);
        if (config_result.is_error()) {
            std::cerr << "Configuration error: " << config_result.error() << std::endl;
            return 1;
        }
        
        // Create and initialize controller
        PendulumBalanceController controller(config_result.value());
        
        auto init_result = controller.initialize();
        if (init_result.is_error()) {
            std::cerr << "Initialization error: " << init_result.error() << std::endl;
            return 2;
        }
        
        // Run the controller
        auto run_result = controller.run();
        if (run_result.is_error()) {
            std::cerr << "Runtime error: " << run_result.error() << std::endl;
            return 3;
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Unhandled exception: " << e.what() << std::endl;
        return 255;
    } catch (...) {
        std::cerr << "Unknown exception occurred" << std::endl;
        return 254;
    }
} 