/**
* communication_controller.cpp - Object-oriented STM32 communication system
*/

#include "communication_controller.hpp"

#include <vector>
#include <string>
#include <csignal>
#include <numeric>
#include <algorithm>
#include <array>
#include <cmath>
#include <atomic>
#include <sstream>
#include <iomanip>

// --- Configuration Constants (used for array sizes etc.) ---
// These need to be truly compile-time constants for std::array
static constexpr size_t   GLOBAL_PING_PAYLOAD_SIZE      = 1;
static constexpr size_t   GLOBAL_PONG_PAYLOAD_SIZE      = 4;
static constexpr uint8_t  GLOBAL_RF_CHANNEL_VAL         = 76;
static constexpr uint8_t  GLOBAL_RADIO_ADDRESS[3]       = {0xD7, 0xD7, 0xD7};

// CRC-4 ITU (Polynomial 0x3 -> x^4 + x + 1)
#define CRC4_POLY_JETSON 0x03
#define CRC4_BITS_JETSON 4

// NRF24L01+ Commands & Registers
#define CMD_R_REG           0x00
#define CMD_W_REG           0x20
#define CMD_R_RX_PAYLOAD    0x61
#define CMD_W_TX_PAYLOAD    0xA0
#define CMD_FLUSH_TX        0xE1
#define CMD_FLUSH_RX        0xE2
#define CMD_NOP             0xFF
#define REG_CONFIG          0x00
#define REG_EN_AA           0x01
#define REG_EN_RXADDR       0x02
#define REG_SETUP_AW        0x03
#define REG_SETUP_RETR      0x04
#define REG_RF_CH           0x05
#define REG_RF_SETUP        0x06
#define REG_STATUS          0x07
#define REG_RX_ADDR_P0      0x0A
#define REG_TX_ADDR         0x10
#define REG_RX_PW_P0        0x11
#define REG_FIFO_STATUS     0x17
#define STATUS_RX_DR        (1U << 6)
#define STATUS_TX_DS        (1U << 5)
#define STATUS_MAX_RT       (1U << 4)
#define CONFIG_EN_CRC       (1U << 3)
#define CONFIG_PWR_UP       (1U << 1)
#define CONFIG_PRIM_RX      (1U << 0)
#define FIFO_RX_EMPTY       (1U << 0)

//==============================================================================
// SPI1_NRF Implementation
//==============================================================================

uint8_t SPI1_NRF::transferByte(uint8_t tx_byte) {
    uint64_t timer_freq = get_timer_frequency_hz();
    if (timer_freq == 0) {
        std::cerr << "[BitBangSPI] ERROR: Timer not calibrated in transferByte." << std::endl;
        return 0xFF;
    }
    
    const uint64_t spi_clk_half_period_cycles = timer_freq / (2 * SPI_FREQ_HZ);
    uint8_t received_byte = 0;
    
    for (int bit_idx = 7; bit_idx >= 0; --bit_idx) {
        // Set MOSI for the current bit
        gpioWrite(PIN_MOSI, (tx_byte >> bit_idx) & 1);
        
        // SCK rising edge
        gpioWrite(PIN_SCK, 1);
        uint64_t t_edge = read_arm_cycle_counter();
        while (read_arm_cycle_counter() - t_edge < spi_clk_half_period_cycles) { /* wait */ }
        
        // Sample MISO at SCK high
        if (gpioRead(PIN_MISO)) {
            received_byte |= (1 << bit_idx);
        }
        
        // SCK falling edge
        gpioWrite(PIN_SCK, 0);
        t_edge = read_arm_cycle_counter();
        while (read_arm_cycle_counter() - t_edge < spi_clk_half_period_cycles) { /* wait */ }
    }
    return received_byte;
}

void SPI1_NRF::xfer(const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t num_bytes) {
    if (num_bytes == 0) return;
    
    gpioWrite(PIN_CSN, 0); // Assert CSN (active low)
    
    for (size_t i = 0; i < num_bytes; ++i) {
        uint8_t byte_out = tx_buffer ? tx_buffer[i] : 0xFF;
        uint8_t byte_in = transferByte(byte_out);
        if (rx_buffer) {
            rx_buffer[i] = byte_in;
        }
    }
    gpioWrite(PIN_CSN, 1); // De-assert CSN
}

void SPI1_NRF::config_shockburst_no_ack(uint8_t channel, const uint8_t address[3], uint8_t payload_size) {
    if (debug_output_enabled_.load(std::memory_order_relaxed)) {
        std::cout << "[BitBangSPI] Configuring No-ACK mode: CH=" << (int)channel 
                  << " Addr=" << std::hex << (int)address[0] << (int)address[1] << (int)address[2] << std::dec
                  << " PLen=" << (int)payload_size << std::endl;
    }
    
    uint8_t cmd_buf[6];
    
    ceLow();
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
    
    // Configure nRF24L01+ registers
    cmd_buf[0] = 0x20 | 0x00; cmd_buf[1] = 0x00; // CONFIG: PWR_DOWN
    xfer(cmd_buf, nullptr, 2);
    std::this_thread::sleep_for(std::chrono::microseconds(2000));
    
    cmd_buf[0] = 0x20 | 0x01; cmd_buf[1] = 0x00; // EN_AA: Disable Auto-ACK
    xfer(cmd_buf, nullptr, 2);
    
    cmd_buf[0] = 0x20 | 0x02; cmd_buf[1] = 0x01; // EN_RXADDR: Enable Pipe 0
    xfer(cmd_buf, nullptr, 2);
    
    cmd_buf[0] = 0x20 | 0x03; cmd_buf[1] = 0x01; // SETUP_AW: 3-byte address
    xfer(cmd_buf, nullptr, 2);
    
    cmd_buf[0] = 0x20 | 0x04; cmd_buf[1] = 0x00; // SETUP_RETR: Disable retransmit
    xfer(cmd_buf, nullptr, 2);
    
    cmd_buf[0] = 0x20 | 0x05; cmd_buf[1] = channel & 0x7F; // RF_CH
    xfer(cmd_buf, nullptr, 2);
    
    cmd_buf[0] = 0x20 | 0x06; cmd_buf[1] = 0x0E; // RF_SETUP: 2Mbps, 0dBm
    xfer(cmd_buf, nullptr, 2);
    
    // Set addresses
    cmd_buf[0] = 0x20 | 0x10; // TX_ADDR
    memcpy(cmd_buf + 1, address, 3);
    xfer(cmd_buf, nullptr, 4);
    
    cmd_buf[0] = 0x20 | 0x0A; // RX_ADDR_P0
    memcpy(cmd_buf + 1, address, 3);
    xfer(cmd_buf, nullptr, 4);
    
    cmd_buf[0] = 0x20 | 0x11; cmd_buf[1] = payload_size; // RX_PW_P0
    xfer(cmd_buf, nullptr, 2);
    
    // Clear FIFOs and status
    cmd_buf[0] = 0xE1; xfer(cmd_buf, nullptr, 1); // FLUSH_TX
    cmd_buf[0] = 0xE2; xfer(cmd_buf, nullptr, 1); // FLUSH_RX
    cmd_buf[0] = 0x20 | 0x07; cmd_buf[1] = 0x70; // Clear status flags
    xfer(cmd_buf, nullptr, 2);
    
    // Power up
    cmd_buf[0] = 0x20 | 0x00; cmd_buf[1] = 0x0A; // CONFIG: PWR_UP, EN_CRC
    xfer(cmd_buf, nullptr, 2);
    std::this_thread::sleep_for(std::chrono::microseconds(1500));
}

//==============================================================================
// STM32Device Implementation
//==============================================================================

STM32Device::STM32Device(int device_id, const STM32Config& config)
    : device_id_(device_id), config_(config) {
    last_communication_time_ = std::chrono::steady_clock::now();
}

bool STM32Device::initialize() {
    if (initialized_) {
        return true;
    }
    
    logDebug("Initializing STM32 device " + std::to_string(device_id_));
    
    if (!initializeRadio()) {
        logError("Failed to initialize radio for device " + std::to_string(device_id_));
        return false;
    }
    
    initialized_ = true;
    resetStatistics();
    logDebug("STM32 device " + std::to_string(device_id_) + " initialized successfully");
    return true;
}

bool STM32Device::ping(uint8_t flags) {
    if (!initialized_) {
        logError("Device not initialized");
        return false;
    }
    
    last_ping_flags_sent_ = flags;
    bool success = sendPing(flags);
    
    if (success) {
        statistics_.successful_pings++;
        statistics_.consecutive_failures = 0;
    } else {
        statistics_.failed_pings++;
        statistics_.consecutive_failures++;
    }
    
    return success;
}

bool STM32Device::readSensorData(SensorData& data) {
    if (!initialized_) {
        logError("Device not initialized");
        return false;
    }
    
    // First ping the device
    auto ping_start_time = std::chrono::steady_clock::now();
    if (!ping()) {
        return false;
    }
    auto ping_end_time = std::chrono::steady_clock::now();
    
    // Wait for STM32 processing time before trying to receive
    // STM32 needs time to process the ping and prepare the response
    auto ping_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        ping_end_time - ping_start_time);
    
    // Ensure minimum delay for STM32 processing (configurable per device)
    auto min_processing_delay = std::chrono::microseconds(config_.stm32_processing_delay_us);
    if (ping_duration < min_processing_delay) {
        auto additional_delay = min_processing_delay - ping_duration;
        std::this_thread::sleep_for(additional_delay);
        logDebug("Added " + std::to_string(additional_delay.count()) + "μs processing delay");
    }
    
    // Now receive the sensor data with proper timing
    uint32_t data_word1, data_word2;
    std::chrono::steady_clock::time_point t1, t2;
    
    if (receiveBurst(data_word1, data_word2, t1, t2)) {
        decodeAndUpdateState(data_word1);
        
        // Update the output data structure
        data.encoder_position = last_sensor_data_.encoder_position;
        data.angular_velocity_rad_per_sec = last_sensor_data_.angular_velocity_rad_per_sec;
        data.timestamp = last_sensor_data_.timestamp;
        data.valid = last_sensor_data_.valid;
        
        last_communication_time_ = std::chrono::steady_clock::now();
        
        // Log timing information for debugging
        auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(
            last_communication_time_ - ping_start_time);
        logDebug("Total communication time: " + std::to_string(total_time.count()) + "μs");
        
        return true;
    }
    
    return false;
}

bool STM32Device::requestAbsoluteResync() {
    return ping(FLAG_REQUEST_RESYNC_ABSOLUTE);
}

bool STM32Device::requestDeltaResync() {
    return ping(FLAG_REQUEST_RESYNC_LAST_TWO);
}

bool STM32Device::requestZeroPosition() {
    return ping(FLAG_REQUEST_ZERO_POSITION);
}

bool STM32Device::isDataAvailable() const {
    if (!initialized_) {
        return false;
    }
    
    // Quick check if data is ready without blocking
    uint8_t cmd_buf[1] = {CMD_NOP};
    uint8_t rx_buf[1];
    SPI1_NRF::xfer(cmd_buf, rx_buf, 1);
    
    // Check if RX_DR flag is set (data ready)
    return (rx_buf[0] & STATUS_RX_DR) != 0;
}

void STM32Device::updateConfig(const STM32Config& config) {
    config_ = config;
    if (initialized_) {
        // Re-initialize with new config
        initialized_ = false;
        initialize();
    }
}

std::string STM32Device::getStatusString() const {
    std::ostringstream oss;
    oss << "STM32[" << device_id_ << "]: ";
    oss << (initialized_ ? "INIT" : "UNINIT") << " ";
    oss << (isConnected() ? "CONN" : "DISC") << " ";
    oss << "Pings:" << statistics_.successful_pings.load() << "/" 
        << (statistics_.successful_pings.load() + statistics_.failed_pings.load()) << " ";
    oss << "Pos:" << last_sensor_data_.encoder_position << " ";
    oss << "Vel:" << std::fixed << std::setprecision(3) << last_sensor_data_.angular_velocity_rad_per_sec;
    return oss.str();
}

bool STM32Device::initializeRadio() {
    logDebug("Initializing nRF24L01+ for device " + std::to_string(device_id_));
    
    SPI1_NRF::ceLow();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Configure the radio for this device
    SPI1_NRF::config_shockburst_no_ack(config_.rf_channel, 
                                        config_.radio_address.data(), 
                                        config_.pong_payload_size);
    
    // Verify configuration by reading back the RF channel
    uint8_t cmd_buf[2] = {CMD_R_REG | REG_RF_CH, CMD_NOP};
    uint8_t rx_buf[2];
    SPI1_NRF::xfer(cmd_buf, rx_buf, 2);
    
    bool success = (rx_buf[1] == config_.rf_channel);
    if (success) {
        logDebug("Radio initialized successfully for device " + std::to_string(device_id_));
    } else {
        logError("Radio initialization failed for device " + std::to_string(device_id_));
    }
    
    return success;
}

bool STM32Device::sendPing(uint8_t flags) {
    // Ensure TX standby mode
    uint8_t cmd_buf[2] = {CMD_R_REG | REG_CONFIG, CMD_NOP};
    uint8_t rx_buf[2];
    SPI1_NRF::xfer(cmd_buf, rx_buf, 2);
    
    if (rx_buf[1] & CONFIG_PRIM_RX) {
        SPI1_NRF::ceLow();
        cmd_buf[0] = CMD_W_REG | REG_CONFIG;
        cmd_buf[1] = rx_buf[1] & ~CONFIG_PRIM_RX;
        SPI1_NRF::xfer(cmd_buf, nullptr, 2);
        std::this_thread::sleep_for(std::chrono::microseconds(150));
    }
    SPI1_NRF::ceLow();
    
    // Flush TX FIFO and clear status
    cmd_buf[0] = CMD_FLUSH_TX;
    SPI1_NRF::xfer(cmd_buf, nullptr, 1);
    
    cmd_buf[0] = CMD_W_REG | REG_STATUS;
    cmd_buf[1] = STATUS_TX_DS | STATUS_MAX_RT;
    SPI1_NRF::xfer(cmd_buf, nullptr, 2);
    
    // Send ping payload
    uint8_t ping_payload[GLOBAL_PING_PAYLOAD_SIZE] = {flags};
    uint8_t tx_cmd[1 + GLOBAL_PING_PAYLOAD_SIZE];
    tx_cmd[0] = CMD_W_TX_PAYLOAD;
    memcpy(tx_cmd + 1, ping_payload, GLOBAL_PING_PAYLOAD_SIZE);
    SPI1_NRF::xfer(tx_cmd, nullptr, 1 + GLOBAL_PING_PAYLOAD_SIZE);
    
    // Pulse CE to start transmission
    SPI1_NRF::ceHigh();
    std::this_thread::sleep_for(std::chrono::microseconds(20));
    SPI1_NRF::ceLow();
    
    // Wait for transmission complete
    auto tx_start_time = std::chrono::steady_clock::now();
    bool tx_success = false;
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - tx_start_time).count() < config_.tx_ds_timeout_ms) {
        
        cmd_buf[0] = CMD_NOP;
        SPI1_NRF::xfer(cmd_buf, rx_buf, 1);
        uint8_t status = rx_buf[0];
        
        if (status & STATUS_TX_DS) {
            cmd_buf[0] = CMD_W_REG | REG_STATUS;
            cmd_buf[1] = STATUS_TX_DS;
            SPI1_NRF::xfer(cmd_buf, nullptr, 2);
            tx_success = true;
            break;
        }
        
        if (status & STATUS_MAX_RT) {
            logError("MAX_RT on PING for device " + std::to_string(device_id_));
            cmd_buf[0] = CMD_W_REG | REG_STATUS;
            cmd_buf[1] = STATUS_MAX_RT;
            SPI1_NRF::xfer(cmd_buf, nullptr, 2);
            cmd_buf[0] = CMD_FLUSH_TX;
            SPI1_NRF::xfer(cmd_buf, nullptr, 1);
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    
    return tx_success;
}

bool STM32Device::receiveBurst(uint32_t& data_word1, uint32_t& data_word2,
                              std::chrono::steady_clock::time_point& t1,
                              std::chrono::steady_clock::time_point& t2) {
    // Enter RX active mode
    uint8_t cmd_buf[2] = {CMD_R_REG | REG_CONFIG, CMD_NOP};
    uint8_t rx_buf[2];
    SPI1_NRF::xfer(cmd_buf, rx_buf, 2);
    
    if (!(rx_buf[1] & CONFIG_PRIM_RX)) {
        cmd_buf[0] = CMD_W_REG | REG_CONFIG;
        cmd_buf[1] = rx_buf[1] | CONFIG_PRIM_RX;
        SPI1_NRF::xfer(cmd_buf, nullptr, 2);
        std::this_thread::sleep_for(std::chrono::microseconds(150));
    }
    SPI1_NRF::ceHigh();
    std::this_thread::sleep_for(std::chrono::microseconds(130));
    
    bool got_packet1 = false, got_packet2 = false;
    uint8_t payload_buffer[GLOBAL_PONG_PAYLOAD_SIZE];
    
    // Wait for first packet with adaptive polling
    auto p1_start_time = std::chrono::steady_clock::now();
    uint32_t poll_count = 0;
    
    while (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::steady_clock::now() - p1_start_time).count() < config_.stm_pkt1_timeout_us) {
        
        cmd_buf[0] = CMD_NOP;
        SPI1_NRF::xfer(cmd_buf, rx_buf, 1);
        poll_count++;
        
        if (rx_buf[0] & STATUS_RX_DR) {
            t1 = std::chrono::steady_clock::now();
            
            // Data is available - read payload immediately
            uint8_t read_cmd[1 + GLOBAL_PONG_PAYLOAD_SIZE];
            uint8_t read_rx[1 + GLOBAL_PONG_PAYLOAD_SIZE];
            read_cmd[0] = CMD_R_RX_PAYLOAD;
            memset(read_cmd + 1, CMD_NOP, GLOBAL_PONG_PAYLOAD_SIZE);
            SPI1_NRF::xfer(read_cmd, read_rx, 1 + GLOBAL_PONG_PAYLOAD_SIZE);
            memcpy(payload_buffer, read_rx + 1, GLOBAL_PONG_PAYLOAD_SIZE);
            
            // Clear RX_DR flag
            cmd_buf[0] = CMD_W_REG | REG_STATUS;
            cmd_buf[1] = STATUS_RX_DR;
            SPI1_NRF::xfer(cmd_buf, nullptr, 2);
            
            memcpy(&data_word1, payload_buffer, GLOBAL_PONG_PAYLOAD_SIZE);
            got_packet1 = true;
            
            auto wait_time = std::chrono::duration_cast<std::chrono::microseconds>(t1 - p1_start_time);
            logDebug("Packet 1 received after " + std::to_string(wait_time.count()) + 
                    "μs (" + std::to_string(poll_count) + " polls)");
            break;
        }
        
        // Adaptive delay - start with shorter delays, increase if no response
        uint32_t delay_us = (poll_count < 10) ? 10 : 
                           (poll_count < 50) ? 20 : 50;
        std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    }
    
    if (!got_packet1) {
        statistics_.pkt1_timeouts++;
        SPI1_NRF::ceLow();
        return false;
    }
    
    // Check for second packet (simplified version - immediate check)
    cmd_buf[0] = CMD_R_REG | REG_FIFO_STATUS;
    SPI1_NRF::xfer(cmd_buf, rx_buf, 2);
    
    if (!(rx_buf[1] & FIFO_RX_EMPTY) && (rx_buf[1] != 0xFF)) {
        t2 = std::chrono::steady_clock::now();
        
        uint8_t read_cmd[1 + GLOBAL_PONG_PAYLOAD_SIZE];
        uint8_t read_rx[1 + GLOBAL_PONG_PAYLOAD_SIZE];
        read_cmd[0] = CMD_R_RX_PAYLOAD;
        memset(read_cmd + 1, CMD_NOP, GLOBAL_PONG_PAYLOAD_SIZE);
        SPI1_NRF::xfer(read_cmd, read_rx, 1 + GLOBAL_PONG_PAYLOAD_SIZE);
        memcpy(payload_buffer, read_rx + 1, GLOBAL_PONG_PAYLOAD_SIZE);
        
        memcpy(&data_word2, payload_buffer, GLOBAL_PONG_PAYLOAD_SIZE);
        got_packet2 = true;
    }
    
    if (got_packet1 && !got_packet2) {
        statistics_.pkt2_failures++;
    }
    
    SPI1_NRF::ceLow();
    return got_packet1 && got_packet2;
}

void STM32Device::decodeAndUpdateState(uint32_t pong_word) {
    bool recovery_frame = (last_ping_flags_sent_ & FLAG_REQUEST_RESYNC_ABSOLUTE) ||
                          (last_ping_flags_sent_ & FLAG_REQUEST_RESYNC_LAST_TWO);
    
    if (recovery_frame) {
        if (!validateCRC(pong_word)) {
            statistics_.crc_errors++;
            statistics_.consecutive_failures++;
            last_sensor_data_.valid = false;
            return;
        }
    }
    
    if (last_ping_flags_sent_ & FLAG_REQUEST_RESYNC_ABSOLUTE) {
        bool dir_abs = (pong_word >> 31) & 0x01;
        int32_t abs_pos_mag = (pong_word >> 8) & 0x007FFFFF;
        accumulated_encoder_position_ = dir_abs ? -abs_pos_mag : abs_pos_mag;
        last_sensor_data_.angular_velocity_rad_per_sec = 0.0f;
    } 
    else if (last_ping_flags_sent_ & FLAG_REQUEST_RESYNC_LAST_TWO) {
        bool dir_d1 = (pong_word >> 31) & 0x01;
        int32_t delta1_mag = (pong_word >> 20) & 0x7FF;
        int32_t delta1 = dir_d1 ? -delta1_mag : delta1_mag;
        bool dir_d2 = (pong_word >> 19) & 0x01;
        int32_t delta2_mag = (pong_word >> 8) & 0x7FF;
        int32_t delta2 = dir_d2 ? -delta2_mag : delta2_mag;
        accumulated_encoder_position_ += delta2 + delta1;
        last_sensor_data_.angular_velocity_rad_per_sec = 0.0f;
    } 
    else { // Normal Frame
        bool dir_pos = (pong_word >> 31) & 0x01;
        int32_t delta_pos_mag = (pong_word >> 20) & 0x7FF;
        int32_t delta_pos = dir_pos ? -delta_pos_mag : delta_pos_mag;
        accumulated_encoder_position_ += delta_pos;
        
        bool dir_spd = (pong_word >> 19) & 0x01;
        int32_t speed_mag_q_lower_19 = pong_word & 0x0007FFFF;
        int32_t speed_fixed = dir_spd ? -speed_mag_q_lower_19 : speed_mag_q_lower_19;
        last_sensor_data_.angular_velocity_rad_per_sec = (float)speed_fixed / config_.q_scale_speed;
    }
    
    // Update sensor data
    last_sensor_data_.encoder_position = accumulated_encoder_position_;
    last_sensor_data_.timestamp = std::chrono::steady_clock::now();
    last_sensor_data_.valid = true;
}

uint8_t STM32Device::calculateCRC4(uint32_t data_word, uint8_t num_data_bits) const {
    uint8_t crc = 0x00;
    uint32_t poly = CRC4_POLY_JETSON;
    for (int i = num_data_bits - 1; i >= 0; i--) {
        bool data_bit_is_set = (data_word >> (i + CRC4_BITS_JETSON)) & 1;
        bool crc_msb_is_set = (crc >> (CRC4_BITS_JETSON - 1)) & 1;
        crc <<= 1;
        if (data_bit_is_set ^ crc_msb_is_set) {
            crc ^= poly;
        }
    }
    return crc & ((1 << CRC4_BITS_JETSON) - 1);
}

bool STM32Device::validateCRC(uint32_t pong_word) const {
    uint8_t received_crc = pong_word & 0x0F;
    uint8_t calculated_crc = calculateCRC4(pong_word, 28);
    return (received_crc == calculated_crc);
}

void STM32Device::logError(const std::string& message) const {
    std::cerr << "[STM32Device " << device_id_ << "] ERROR: " << message << std::endl;
}

void STM32Device::logDebug(const std::string& message) const {
    std::cout << "[STM32Device " << device_id_ << "] DEBUG: " << message << std::endl;
}

//==============================================================================
// STM32Manager Implementation
//==============================================================================

bool STM32Manager::addDevice(int device_id, const STM32Config& config) {
    // Check if device already exists
    if (findDevice(device_id) != devices_.end()) {
        return false; // Device already exists
    }
    
    auto device = std::make_unique<STM32Device>(device_id, config);
    devices_.push_back(std::move(device));
    return true;
}

bool STM32Manager::removeDevice(int device_id) {
    auto it = findDevice(device_id);
    if (it != devices_.end()) {
        devices_.erase(it);
        return true;
    }
    return false;
}

STM32Device* STM32Manager::getDevice(int device_id) {
    auto it = findDevice(device_id);
    return (it != devices_.end()) ? it->get() : nullptr;
}

const STM32Device* STM32Manager::getDevice(int device_id) const {
    auto it = findDevice(device_id);
    return (it != devices_.end()) ? it->get() : nullptr;
}

bool STM32Manager::initializeAll() {
    bool all_success = true;
    for (auto& device : devices_) {
        if (!device->initialize()) {
            all_success = false;
        }
    }
    return all_success;
}

bool STM32Manager::pingAll() {
    bool all_success = true;
    for (auto& device : devices_) {
        if (!device->ping()) {
            all_success = false;
        }
    }
    return all_success;
}

bool STM32Manager::readAllSensorData(std::vector<STM32Device::SensorData>& data_list) {
    data_list.clear();
    data_list.reserve(devices_.size());
    
    bool all_success = true;
    for (auto& device : devices_) {
        STM32Device::SensorData data;
        if (device->readSensorData(data)) {
            data_list.push_back(data);
        } else {
            all_success = false;
            // Still add invalid data to maintain indexing
            data.valid = false;
            data_list.push_back(data);
        }
    }
    return all_success;
}

std::vector<int> STM32Manager::getDeviceIds() const {
    std::vector<int> ids;
    ids.reserve(devices_.size());
    for (const auto& device : devices_) {
        ids.push_back(device->getDeviceId());
    }
    return ids;
}

std::string STM32Manager::getStatusReport() const {
    std::ostringstream oss;
    oss << "STM32Manager Status Report:\n";
    oss << "Total devices: " << devices_.size() << "\n";
    
    for (const auto& device : devices_) {
        oss << "  " << device->getStatusString() << "\n";
    }
    
    return oss.str();
}

STM32Manager::DeviceList::iterator STM32Manager::findDevice(int device_id) {
    return std::find_if(devices_.begin(), devices_.end(),
                       [device_id](const DevicePtr& device) {
                           return device->getDeviceId() == device_id;
                       });
}

STM32Manager::DeviceList::const_iterator STM32Manager::findDevice(int device_id) const {
    return std::find_if(devices_.begin(), devices_.end(),
                       [device_id](const DevicePtr& device) {
                           return device->getDeviceId() == device_id;
                       });
}

//==============================================================================
// Legacy Compatibility Layer (to be removed after full refactoring)
//==============================================================================

// Legacy global configuration
struct LegacyAppConfig {
    uint8_t  rf_channel;
    uint8_t  radio_address[3];
    size_t   ping_payload_size;
    size_t   pong_payload_size;
    unsigned target_hz;
    uint32_t stm_pkt1_timeout_us;
    uint32_t stm_pkt2_poll_timeout_us;
    uint32_t pkt2_polling_sleep_us;
    uint32_t tx_ds_timeout_ms;
    uint16_t encoder_ppr;
    uint32_t counts_per_revolution;
    float    q_scale_speed;
    uint32_t data_log_interval_hz_divisor;

    LegacyAppConfig() :
        rf_channel(GLOBAL_RF_CHANNEL_VAL),
        ping_payload_size(GLOBAL_PING_PAYLOAD_SIZE),
        pong_payload_size(GLOBAL_PONG_PAYLOAD_SIZE),
        target_hz(150),
        stm_pkt1_timeout_us(3500),
        stm_pkt2_poll_timeout_us(200),
        pkt2_polling_sleep_us(2),
        tx_ds_timeout_ms(10),
        encoder_ppr(5000),
        counts_per_revolution(encoder_ppr * 4),
        q_scale_speed(16384.0f),
        data_log_interval_hz_divisor(30) {
        memcpy(radio_address, GLOBAL_RADIO_ADDRESS, sizeof(GLOBAL_RADIO_ADDRESS));
    }
};

const LegacyAppConfig Cfg; // Global const config instance
const STM32Config LegacyCfg{}; // For new system compatibility

// Legacy global variables
std::atomic<bool> g_running{true};
int32_t g_current_encoder_position = 0;
float g_current_speed_rad_per_sec = 0.0f;

// Legacy functions - simple wrappers around new system
bool radio_init_jetson_prx() {
    return SPI1_NRF::initialize();
}

bool jetson_send_ping(uint8_t ping_flags_byte) {
    // This is a simplified legacy wrapper
    // In practice, you should use the STM32Device class directly
    static STM32Device legacy_device(0, LegacyCfg);
    static bool initialized = false;
    
    if (!initialized) {
        initialized = legacy_device.initialize();
    }
    
    return legacy_device.ping(ping_flags_byte);
}

bool jetson_receive_stm_burst(uint32_t& data_word1, uint32_t& data_word2,
                              std::chrono::steady_clock::time_point& t_pkt1_rx_event,
                              std::chrono::steady_clock::time_point& t_pkt2_rx_event) {
    // Legacy wrapper - simplified implementation
    static STM32Device legacy_device(0, LegacyCfg);
    STM32Device::SensorData data;
    
    if (legacy_device.readSensorData(data)) {
        t_pkt1_rx_event = data.timestamp;
        t_pkt2_rx_event = data.timestamp;
        data_word1 = data.encoder_position; // Simplified
        data_word2 = 0; // Not used in new system
        return true;
    }
    return false;
}

void decode_and_update_state(uint32_t pong_word) {
    // Legacy function - updates global variables
    // This should be replaced with STM32Device usage
    static STM32Device legacy_device(0, LegacyCfg);
    // Implementation would go here, but this is deprecated
    (void)pong_word; // Suppress unused parameter warning
}

