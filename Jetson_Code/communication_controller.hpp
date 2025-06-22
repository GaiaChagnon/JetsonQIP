#ifndef BITBANGSPI_HPP
#define BITBANGSPI_HPP

#if defined(__linux__) && defined(__aarch64__)
#include <jetgpio.h>  // For Jetson GPIO control
#else
#include "mock_gpio.h"  // Mock for non-Jetson systems
#endif
#include <sched.h>    // For real-time scheduling (sched_setscheduler)
#include <unistd.h>   // For usleep (used in timer calibration)
#include <cstdint>    // For uint8_t, uint64_t
#include <cstring>    // For memcpy, memset
#include <iostream>   // For std::cout, std::cerr (used in timer calibration, debug)
#include <iomanip>    // For std::setw, std::setfill, std::hex (debug formatting)
#include <chrono>     // For clock calibration (high_resolution_clock)
#include <atomic>     // For std::atomic
#include <thread>     // For std::this_thread::sleep_for (timer calibration)
#include <sstream>    // For std::stringstream (debug formatting)
#include <array>      // For std::array
#include <memory>     // For smart pointers
#include <string>     // For std::string
#include <vector>     // For std::vector

// Configuration structure for STM32 communication
struct STM32Config {
    std::array<uint8_t, 3> radio_address;
    uint8_t rf_channel{76};
    size_t ping_payload_size{1};
    size_t pong_payload_size{4};
    uint32_t stm_pkt1_timeout_us{3500};
    uint32_t stm_pkt2_poll_timeout_us{200};
    uint32_t pkt2_polling_sleep_us{2};
    uint32_t tx_ds_timeout_ms{10};
    uint32_t stm32_processing_delay_us{200}; // Time for STM32 to process ping and prepare response
    uint16_t encoder_ppr{5000};
    uint32_t counts_per_revolution{20000}; // encoder_ppr * 4
    float q_scale_speed{16384.0f}; // Q6.14 (1 << 14)
    
    STM32Config() = default;
    STM32Config(const std::array<uint8_t, 3>& addr) : radio_address(addr) {}
};

// Communication flags
constexpr uint8_t FLAG_NONE = 0x00;
constexpr uint8_t FLAG_REQUEST_RESYNC_ABSOLUTE = (1 << 0);
constexpr uint8_t FLAG_REQUEST_RESYNC_LAST_TWO = (1 << 1);
constexpr uint8_t FLAG_REQUEST_ZERO_POSITION = (1 << 7);

// Forward declaration
class SPI1_NRF;

// STM32 Device Class - encapsulates communication with individual STM32 units
class STM32Device {
public:
    struct SensorData {
        int32_t encoder_position{0};
        float angular_velocity_rad_per_sec{0.0f};
        std::chrono::steady_clock::time_point timestamp;
        bool valid{false};
    };

    struct Statistics {
        std::atomic<uint32_t> successful_pings{0};
        std::atomic<uint32_t> failed_pings{0};
        std::atomic<uint32_t> pkt1_timeouts{0};
        std::atomic<uint32_t> pkt2_failures{0};
        std::atomic<uint32_t> crc_errors{0};
        std::atomic<uint32_t> consecutive_failures{0};
        
        void reset() {
            successful_pings = 0;
            failed_pings = 0;
            pkt1_timeouts = 0;
            pkt2_failures = 0;
            crc_errors = 0;
            consecutive_failures = 0;
        }
    };

    // Constructor
    explicit STM32Device(int device_id, const STM32Config& config);
    
    // Destructor
    ~STM32Device() = default;
    
    // Copy/move semantics
    STM32Device(const STM32Device&) = delete;
    STM32Device& operator=(const STM32Device&) = delete;
    STM32Device(STM32Device&&) = delete;
    STM32Device& operator=(STM32Device&&) = delete;

    // Public interface
    bool initialize();
    bool ping(uint8_t flags = FLAG_NONE);
    bool readSensorData(SensorData& data);
    bool isDataAvailable() const;
    bool requestAbsoluteResync();
    bool requestDeltaResync();
    bool requestZeroPosition();
    
    // Getters
    int getDeviceId() const { return device_id_; }
    const STM32Config& getConfig() const { return config_; }
    const SensorData& getLastSensorData() const { return last_sensor_data_; }
    const Statistics& getStatistics() const { return statistics_; }
    bool isInitialized() const { return initialized_; }
    bool isConnected() const { return last_sensor_data_.valid && 
                                     (std::chrono::steady_clock::now() - last_sensor_data_.timestamp) < 
                                     std::chrono::milliseconds(100); }
    
    // Configuration
    void updateConfig(const STM32Config& config);
    void resetStatistics() { statistics_.reset(); }
    
    // Utility
    std::string getStatusString() const;

private:
    // Private member variables
    int device_id_;
    STM32Config config_;
    bool initialized_{false};
    SensorData last_sensor_data_;
    Statistics statistics_;
    uint8_t last_ping_flags_sent_{FLAG_NONE};
    
    // Internal state tracking
    int32_t accumulated_encoder_position_{0};
    std::chrono::steady_clock::time_point last_communication_time_;
    
    // Private methods
    bool initializeRadio();
    bool sendPing(uint8_t flags);
    bool receiveBurst(uint32_t& data_word1, uint32_t& data_word2,
                     std::chrono::steady_clock::time_point& t1,
                     std::chrono::steady_clock::time_point& t2);
    void decodeAndUpdateState(uint32_t pong_word);
    uint8_t calculateCRC4(uint32_t data_word, uint8_t num_data_bits) const;
    bool validateCRC(uint32_t pong_word) const;
    void logError(const std::string& message) const;
    void logDebug(const std::string& message) const;
};

// STM32 Manager Class - manages multiple STM32 devices
class STM32Manager {
public:
    using DevicePtr = std::unique_ptr<STM32Device>;
    using DeviceList = std::vector<DevicePtr>;

    STM32Manager() = default;
    ~STM32Manager() = default;
    
    // Copy/move semantics
    STM32Manager(const STM32Manager&) = delete;
    STM32Manager& operator=(const STM32Manager&) = delete;
    STM32Manager(STM32Manager&&) = delete;
    STM32Manager& operator=(STM32Manager&&) = delete;

    // Device management
    bool addDevice(int device_id, const STM32Config& config);
    bool removeDevice(int device_id);
    STM32Device* getDevice(int device_id);
    const STM32Device* getDevice(int device_id) const;
    
    // Bulk operations
    bool initializeAll();
    bool pingAll();
    bool readAllSensorData(std::vector<STM32Device::SensorData>& data_list);
    
    // Information
    size_t getDeviceCount() const { return devices_.size(); }
    std::vector<int> getDeviceIds() const;
    std::string getStatusReport() const;
    
    // Iteration support
    auto begin() { return devices_.begin(); }
    auto end() { return devices_.end(); }
    auto begin() const { return devices_.begin(); }
    auto end() const { return devices_.end(); }

private:
    DeviceList devices_;
    
    DeviceList::iterator findDevice(int device_id);
    DeviceList::const_iterator findDevice(int device_id) const;
};

//------------------------------------------------------------------------------
// Class: SPI1_NRF
// Purpose: Bit-banged SPI interface dedicated to nRF24L01+ radios
//------------------------------------------------------------------------------
class SPI1_NRF {
public:
    // GPIO Pin definitions (BCM numbering for Jetson 40-pin header)
    static constexpr int PIN_MOSI = 19;
    static constexpr int PIN_MISO = 21;
    static constexpr int PIN_SCK  = 23;
    static constexpr int PIN_CSN  = 26; // Chip Select Not (active LOW)
    static constexpr int PIN_CE   = 11; // Chip Enable (for TX/RX activation)
    static constexpr int PIN_IRQ  = 12; // Interrupt Request from nRF24L01+ (active LOW)

    // Target SPI clock frequency for bit-banging
    static constexpr uint64_t SPI_FREQ_HZ = 10000000ULL; // 10 MHz

private:
    // Inline static members (C++17) for header-only library
    inline static std::atomic<bool> debug_output_enabled_{false};
    inline static std::atomic<uint64_t> calibrated_timer_hz_{0};

    // Reads the ARMv8 virtual cycle counter (CNTVCT_EL0)
    static inline uint64_t read_arm_cycle_counter() {
        uint64_t v;
        asm volatile("mrs %0, cntvct_el0" : "=r"(v));
        return v;
    }

public:
    // Calibrates and/or returns the frequency of the ARM cycle counter.
    // This is called internally when needed.
    static uint64_t get_timer_frequency_hz() {
        uint64_t cached_val = calibrated_timer_hz_.load(std::memory_order_relaxed);
        if (cached_val != 0) {
            return cached_val;
        }

        // Calibration logic (takes ~100ms)
        using namespace std::chrono;
        std::cout << "[BitBangSPI] Calibrating timer frequency..." << std::endl;
        auto t_start_chrono = high_resolution_clock::now();
        uint64_t c_start_cycles = read_arm_cycle_counter();
        std::this_thread::sleep_for(milliseconds(100)); // Calibrate over 100ms
        uint64_t c_end_cycles = read_arm_cycle_counter();
        auto t_end_chrono = high_resolution_clock::now();

        double elapsed_seconds = duration<double>(t_end_chrono - t_start_chrono).count();
        uint64_t elapsed_cycles = c_end_cycles - c_start_cycles;

        if (elapsed_seconds > 0.001) { // Ensure some time has passed
            cached_val = static_cast<uint64_t>(static_cast<double>(elapsed_cycles) / elapsed_seconds + 0.5);
        } else {
            std::cerr << "[BitBangSPI] WARN: Timer calibration time too short. Defaulting to 1GHz." << std::endl;
            cached_val = 1000000000ULL; // Fallback to 1 GHz if calibration fails
        }
        
        calibrated_timer_hz_.store(cached_val, std::memory_order_relaxed);
        std::cout << "[BitBangSPI] Generic timer calibrated to: " << cached_val << " Hz" << std::endl;
        return cached_val;
    }

    // Convert timer cycles to microseconds
    static double cycles_to_us(uint64_t cycles) {
        uint64_t timer_freq = get_timer_frequency_hz();
        if (timer_freq == 0) return 0.0; // Avoid division by zero if not calibrated
        return static_cast<double>(cycles) * 1000000.0 / static_cast<double>(timer_freq);
    }

    // Convert microseconds to timer cycles
    static uint64_t us_to_cycles(float us) {
        uint64_t timer_freq = get_timer_frequency_hz();
        if (timer_freq == 0) return 0;
        return static_cast<uint64_t>(us * static_cast<double>(timer_freq) / 1000000.0f + 0.5f);
    }

    // Initialize GPIO lines and set real-time priority.
    static bool initialize(int prio = 90) {
        if (gpioInitialise() < 0) {
            std::cerr << "[BitBangSPI] ERROR: gpioInitialise() failed." << std::endl;
            return false;
        }

        // Force calibration on first init
        get_timer_frequency_hz(); 

        // Configure pin directions
        gpioSetMode(PIN_MOSI, JET_OUTPUT);
        gpioSetMode(PIN_MISO, JET_INPUT);
        gpioSetMode(PIN_SCK, JET_OUTPUT);
        gpioSetMode(PIN_CSN, JET_OUTPUT);
        gpioSetMode(PIN_CE, JET_OUTPUT);
        gpioSetMode(PIN_IRQ, JET_INPUT);

        // Set initial idle states for nRF24L01+
        gpioWrite(PIN_MOSI, 0);       // MOSI low
        gpioWrite(PIN_SCK, 0);        // SCK low
        gpioWrite(PIN_CSN, 1);        // CSN high (inactive)
        gpioWrite(PIN_CE, 0);         // CE low (standby)

        // Elevate process to real-time scheduling
        sched_param sp_params;
        sp_params.sched_priority = prio;
#if defined(__linux__)
        if (sched_setscheduler(0, SCHED_FIFO, &sp_params) != 0) {
#else
        if (false) { // Skip scheduler setup on non-Linux systems
#endif
            std::cerr << "[BitBangSPI] WARN: Failed to set real-time scheduling priority (SCHED_FIFO). "
                      << "Try running with sudo. Error: " << strerror(errno) << std::endl;
            // Not returning false, as it might still work, but with more jitter.
        }
        return true;
    }

    // Clean up GPIO resources.
    static void terminate() {
        gpioTerminate();
    }

    // Enable or disable debug prints for SPI operations.
    static void setDebug(bool on) {
        debug_output_enabled_.store(on, std::memory_order_relaxed);
    }

    // Low-level SPI transfer
    static uint8_t transferByte(uint8_t tx_byte);
    
    // SPI transfer of multiple bytes
    static void xfer(const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t num_bytes);

    // nRF24L01+ control functions
    static void ceHigh() { gpioWrite(PIN_CE, 1); }
    static void ceLow()  { gpioWrite(PIN_CE, 0); }
    
    // Check if data is ready (IRQ pin low)
    static bool isDataReady() {
        return (gpioRead(PIN_IRQ) == 0);
    }

    // Configure nRF24L01+ for ShockBurst without auto-acknowledgment
    static void config_shockburst_no_ack(uint8_t channel, const uint8_t address[3], uint8_t payload_size);
};

// Legacy compatibility - will be removed after refactoring
extern std::atomic<bool> g_running;
extern int32_t g_current_encoder_position;
extern float g_current_speed_rad_per_sec;

// Legacy function declarations - will be removed after refactoring
bool radio_init_jetson_prx();
bool jetson_send_ping(uint8_t ping_flags_byte);
bool jetson_receive_stm_burst(uint32_t& data_word1, uint32_t& data_word2,
                              std::chrono::steady_clock::time_point& t_pkt1_rx_event,
                              std::chrono::steady_clock::time_point& t_pkt2_rx_event);
void decode_and_update_state(uint32_t pong_word);

#endif // BITBANGSPI_HPP