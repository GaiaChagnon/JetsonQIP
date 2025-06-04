/**
* comms.cpp
*/

#include "BitBangSPI.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <atomic>
#include <numeric>
#include <algorithm>
#include <array>
#include <sstream>
#include <cmath>

// --- Configuration Constants (used for array sizes etc.) ---
// These need to be truly compile-time constants for std::array
static constexpr size_t   GLOBAL_PING_PAYLOAD_SIZE      = 1;
static constexpr size_t   GLOBAL_PONG_PAYLOAD_SIZE      = 4;
static constexpr uint8_t  GLOBAL_RF_CHANNEL_VAL         = 76;
static constexpr uint8_t  GLOBAL_RADIO_ADDRESS[3]       = {0xD7, 0xD7, 0xD7};


// --- Configuration Struct ---
struct AppConfig {
    // Radio
    uint8_t  rf_channel;
    uint8_t  radio_address[3];
    size_t   ping_payload_size;
    size_t   pong_payload_size;
    unsigned target_hz;
    uint32_t stm_pkt1_timeout_us;
    uint32_t stm_pkt2_poll_timeout_us;
    uint32_t pkt2_polling_sleep_us;
    uint32_t tx_ds_timeout_ms;

    // Encoder & Speed (mostly for display/interpretation)
    uint16_t encoder_ppr;
    uint32_t counts_per_revolution;
    float    q_scale_speed;

    // Logging
    uint32_t data_log_interval_hz_divisor;

    // Default constructor to initialize from global constants
    AppConfig() :
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
        q_scale_speed(16384.0f), // Q6.14 (1 << 14)
        data_log_interval_hz_divisor(30) // Log data ~TARGET_HZ / this value times per sec
    {
        memcpy(radio_address, GLOBAL_RADIO_ADDRESS, sizeof(GLOBAL_RADIO_ADDRESS));
    }
};
const AppConfig Cfg; // Global const config instance

// PING Payload Flags (Jetson to STM32)
static constexpr uint8_t FLAG_NONE                       = 0x00;
static constexpr uint8_t FLAG_REQUEST_RESYNC_ABSOLUTE    = (1 << 0);
static constexpr uint8_t FLAG_REQUEST_RESYNC_LAST_TWO    = (1 << 1);
static constexpr uint8_t FLAG_ILLEGAL_RECOVERY_COMBO     = (FLAG_REQUEST_RESYNC_ABSOLUTE | FLAG_REQUEST_RESYNC_LAST_TWO);
static constexpr uint8_t FLAG_REQUEST_ZERO_POSITION      = (1 << 7);

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


// Globals
static std::atomic<bool> g_running{true};
static bool g_debug_spi = false;

// Use the compile-time constants for std::array sizes
static std::array<uint8_t, 1 + 5> g_spi_cmd_addr_buf_tx;
static std::array<uint8_t, 1 + 5> g_spi_cmd_addr_buf_rx;
static std::array<uint8_t, 1 + GLOBAL_PONG_PAYLOAD_SIZE> g_spi_payload_buf_tx_data; // Renamed for clarity
static std::array<uint8_t, 1 + GLOBAL_PONG_PAYLOAD_SIZE> g_spi_payload_buf_rx_data; // Renamed for clarity

// Statistics and State variables (same as before)
static std::vector<double> g_rtt_to_first_packet_us_all;
static std::vector<double> g_burst_duration_on_jetson_us_all;
static uint32_t g_pings_sent_total = 0;
static uint32_t g_ping_tx_fails_total = 0;
static uint32_t g_bursts_received_ok_total = 0;
static uint32_t g_burst_pkt1_timeouts_total = 0;
static uint32_t g_burst_pkt2_fails_after_pkt1_ok_total = 0;
static uint32_t g_zero_requests_sent = 0;
static uint32_t g_miss1_recovery_requests_sent = 0;
static uint32_t g_resync_absolute_requests_sent = 0;
static uint32_t g_successful_miss1_recoveries = 0;
static uint32_t g_successful_resync_abs_recoveries = 0;
static uint32_t g_crc_errors_detected = 0;
static int32_t g_current_encoder_position = 0;
static float   g_current_speed_rad_per_sec = 0.0f;
static uint32_t g_consecutive_missed_pongs = 0;
static bool g_is_first_ping = true;
static uint8_t g_last_ping_flags_sent = FLAG_NONE;


// Utility functions
static void log_msg(const std::string& level, const std::string& msg) { using namespace std::chrono; auto now_sys = system_clock::now(); auto ms = duration_cast<milliseconds>(now_sys.time_since_epoch()) % 1000; std::time_t t = system_clock::to_time_t(now_sys); char time_buf[30]; std::tm tm_info_buf; localtime_r(&t, &tm_info_buf); std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &tm_info_buf); std::cout << "[" << time_buf << "." << std::setw(3) << std::setfill('0') << ms.count() << "]" << "[" << level << "] " << msg << std::endl; }
std::string format_hex_val(uint32_t val, int bytes = 4) { std::stringstream ss; ss << "0x"; for (int i = bytes - 1; i >= 0; --i) { ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>((val >> (i * 8)) & 0xFF); } return ss.str(); }
std::string format_hex_byte(uint8_t val) { std::stringstream ss; ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(val); return ss.str(); }
static void sig_handler(int signum) { log_msg("INFO", "Signal " + std::to_string(signum) + " caught. Shutting down..."); g_running = false; }


// NRF Low-Level Control (adapted for renamed global buffers)
uint8_t read_status_register() { g_spi_cmd_addr_buf_tx[0] = CMD_NOP; SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 1); return g_spi_cmd_addr_buf_rx[0]; }
uint8_t read_register(uint8_t reg) { g_spi_cmd_addr_buf_tx[0] = CMD_R_REG | (reg & 0x1F); g_spi_cmd_addr_buf_tx[1] = CMD_NOP; SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 2); return g_spi_cmd_addr_buf_rx[1]; }
void write_register(uint8_t reg, uint8_t value) { g_spi_cmd_addr_buf_tx[0] = CMD_W_REG | (reg & 0x1F); g_spi_cmd_addr_buf_tx[1] = value; SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 2); }
void write_register_multi(uint8_t reg, const uint8_t* data, size_t len) { g_spi_cmd_addr_buf_tx[0] = CMD_W_REG | (reg & 0x1F); memcpy(g_spi_cmd_addr_buf_tx.data() + 1, data, len); SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 1 + len); }
void read_payload(uint8_t* buffer, size_t len) {
    g_spi_payload_buf_tx_data[0] = CMD_R_RX_PAYLOAD;
    memset(g_spi_payload_buf_tx_data.data() + 1, CMD_NOP, len); // Use .data()
    SPI1_NRF::xfer(g_spi_payload_buf_tx_data.data(), g_spi_payload_buf_rx_data.data(), 1 + len);
    memcpy(buffer, g_spi_payload_buf_rx_data.data() + 1, len);
}
void write_payload(const uint8_t* data, size_t len) {
    g_spi_payload_buf_tx_data[0] = CMD_W_TX_PAYLOAD;
    memcpy(g_spi_payload_buf_tx_data.data() + 1, data, len);
    SPI1_NRF::xfer(g_spi_payload_buf_tx_data.data(), g_spi_payload_buf_rx_data.data(), 1 + len);
}
void flush_tx_fifo() {  g_spi_cmd_addr_buf_tx[0] = CMD_FLUSH_TX; SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 1); }
void flush_rx_fifo() { g_spi_cmd_addr_buf_tx[0] = CMD_FLUSH_RX; SPI1_NRF::xfer(g_spi_cmd_addr_buf_tx.data(), g_spi_cmd_addr_buf_rx.data(), 1); }
void clear_interrupt_flags() { write_register(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT); }


bool radio_init_jetson_prx() {
    log_msg("INFO", "Initializing nRF24L01+ for Jetson (Encoder PRX)...");
    SPI1_NRF::ceLow(); std::this_thread::sleep_for(std::chrono::milliseconds(100));
    write_register(REG_CONFIG, 0x00); std::this_thread::sleep_for(std::chrono::milliseconds(5));
    write_register(REG_EN_AA, 0x00); write_register(REG_EN_RXADDR, 0x01); write_register(REG_SETUP_AW, 0x01); write_register(REG_SETUP_RETR, 0x00);
    write_register(REG_RF_CH, Cfg.rf_channel);
    write_register(REG_RF_SETUP, 0x0E);
    write_register_multi(REG_TX_ADDR, Cfg.radio_address, sizeof(Cfg.radio_address));
    write_register_multi(REG_RX_ADDR_P0, Cfg.radio_address, sizeof(Cfg.radio_address));
    write_register(REG_RX_PW_P0, Cfg.pong_payload_size);
    uint8_t config_val = CONFIG_PWR_UP | CONFIG_EN_CRC;
    write_register(REG_CONFIG, config_val);
    std::this_thread::sleep_for(std::chrono::microseconds(1500));
    flush_tx_fifo(); flush_rx_fifo(); clear_interrupt_flags();
    log_msg("INFO", "nRF24L01+ on Jetson initialized.");
    return (read_register(REG_RF_CH) == Cfg.rf_channel);
}
void ensure_tx_standby_mode() { uint8_t current_config = read_register(REG_CONFIG); if (current_config & CONFIG_PRIM_RX) { SPI1_NRF::ceLow(); write_register(REG_CONFIG, (current_config & ~CONFIG_PRIM_RX)); std::this_thread::sleep_for(std::chrono::microseconds(150)); } SPI1_NRF::ceLow(); }
void enter_rx_active_mode() { uint8_t current_config = read_register(REG_CONFIG); if (!(current_config & CONFIG_PRIM_RX)) { write_register(REG_CONFIG, (current_config | CONFIG_PRIM_RX)); std::this_thread::sleep_for(std::chrono::microseconds(150)); } SPI1_NRF::ceHigh(); std::this_thread::sleep_for(std::chrono::microseconds(130)); }

bool jetson_send_ping(uint8_t ping_flags_byte) {
    ensure_tx_standby_mode(); flush_tx_fifo(); write_register(REG_STATUS, STATUS_TX_DS | STATUS_MAX_RT);
    uint8_t ping_payload[Cfg.ping_payload_size] = {ping_flags_byte}; // Array size from Cfg
    write_payload(ping_payload, Cfg.ping_payload_size);
    SPI1_NRF::ceHigh(); std::this_thread::sleep_for(std::chrono::microseconds(20)); SPI1_NRF::ceLow();
    auto tx_start_time = std::chrono::steady_clock::now(); bool tx_success = false;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - tx_start_time).count() < Cfg.tx_ds_timeout_ms) {
        uint8_t status = read_status_register();
        if (status & STATUS_TX_DS) { write_register(REG_STATUS, STATUS_TX_DS); tx_success = true; break; }
        if (status & STATUS_MAX_RT) { log_msg("WARN", "MAX_RT on PING!"); write_register(REG_STATUS, STATUS_MAX_RT); flush_tx_fifo(); break; }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    if (!tx_success) { g_ping_tx_fails_total++; log_msg("WARN", "PING TX fail. Flags: " + format_hex_byte(ping_flags_byte)); }
    return tx_success;
}

bool jetson_receive_stm_burst(uint32_t& data_word1, uint32_t& data_word2,
                              std::chrono::steady_clock::time_point& t_pkt1_rx_event,
                              std::chrono::steady_clock::time_point& t_pkt2_rx_event) {
    enter_rx_active_mode();
    bool got_packet1 = false; bool got_packet2 = false;
    uint8_t payload_buffer[Cfg.pong_payload_size]; // Use Cfg for size
    auto p1_loop_start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - p1_loop_start_time).count() < Cfg.stm_pkt1_timeout_us) {
        if (!g_running) { SPI1_NRF::ceLow(); return false; }
        if (read_status_register() & STATUS_RX_DR) {
            t_pkt1_rx_event = std::chrono::steady_clock::now();
            read_payload(payload_buffer, Cfg.pong_payload_size);
            write_register(REG_STATUS, STATUS_RX_DR);
            memcpy(&data_word1, payload_buffer, Cfg.pong_payload_size);
            got_packet1 = true; break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
    if (!got_packet1) { if (g_running) g_burst_pkt1_timeouts_total++; SPI1_NRF::ceLow(); return false; }

    uint8_t fifo_stat_after_pkt1 = read_register(REG_FIFO_STATUS);
    if (!(fifo_stat_after_pkt1 & FIFO_RX_EMPTY) && (fifo_stat_after_pkt1 != 0xFF)) {
        t_pkt2_rx_event = std::chrono::steady_clock::now();
        read_payload(payload_buffer, Cfg.pong_payload_size);
        if(read_status_register() & STATUS_RX_DR) write_register(REG_STATUS, STATUS_RX_DR);
        memcpy(&data_word2, payload_buffer, Cfg.pong_payload_size);
        got_packet2 = true;
    } else {
        auto p2_poll_start_time = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - p2_poll_start_time).count() < Cfg.stm_pkt2_poll_timeout_us) {
            if (!g_running) { SPI1_NRF::ceLow(); return false; }
            if (read_status_register() & STATUS_RX_DR) {
                t_pkt2_rx_event = std::chrono::steady_clock::now();
                read_payload(payload_buffer, Cfg.pong_payload_size);
                write_register(REG_STATUS, STATUS_RX_DR);
                memcpy(&data_word2, payload_buffer, Cfg.pong_payload_size);
                got_packet2 = true; break;
            }
            if (Cfg.pkt2_polling_sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(Cfg.pkt2_polling_sleep_us));
            else std::this_thread::yield();
        }
    }
    if (got_packet1 && !got_packet2 && g_running) g_burst_pkt2_fails_after_pkt1_ok_total++;
    uint8_t final_fifo_status = read_register(REG_FIFO_STATUS);
    if (!(final_fifo_status & FIFO_RX_EMPTY) && (final_fifo_status != 0xFF)) { log_msg("WARN", "RX FIFO not empty post-burst. Flushing. FIFO_STATUS: " + format_hex_byte(final_fifo_status)); flush_rx_fifo(); }
    SPI1_NRF::ceLow();
    return got_packet1 && got_packet2;
}

uint8_t calculate_crc4_itu_jetson(uint32_t data_word, uint8_t num_data_bits) {
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

void decode_and_update_state(uint32_t pong_word) {
    bool recovery_frame = (g_last_ping_flags_sent & FLAG_REQUEST_RESYNC_ABSOLUTE) ||
                          (g_last_ping_flags_sent & FLAG_REQUEST_RESYNC_LAST_TWO);
    if (recovery_frame) {
        uint8_t received_crc = pong_word & 0x0F;
        uint8_t calculated_crc = calculate_crc4_itu_jetson(pong_word, 28);
        if (received_crc != calculated_crc) {
            log_msg("ERROR", "CRC Mismatch! Rec: " + format_hex_byte(received_crc) +
                             " Calc: " + format_hex_byte(calculated_crc) + " Data: " + format_hex_val(pong_word));
            g_crc_errors_detected++; g_consecutive_missed_pongs++; return;
        }
    }

    if (g_last_ping_flags_sent & FLAG_REQUEST_RESYNC_ABSOLUTE) {
        bool dir_abs = (pong_word >> 31) & 0x01;
        int32_t abs_pos_mag = (pong_word >> 8) & 0x007FFFFF;
        g_current_encoder_position = dir_abs ? -abs_pos_mag : abs_pos_mag;
        g_current_speed_rad_per_sec = 0.0f;
        log_msg("RECOVERY", "Abs Resync OK: New Pos: " + std::to_string(g_current_encoder_position));
        g_successful_resync_abs_recoveries++;
    } else if (g_last_ping_flags_sent & FLAG_REQUEST_RESYNC_LAST_TWO) {
        bool dir_d1 = (pong_word >> 31) & 0x01;
        int32_t delta1_mag = (pong_word >> 20) & 0x7FF;
        int32_t delta1 = dir_d1 ? -delta1_mag : delta1_mag;
        bool dir_d2 = (pong_word >> 19) & 0x01;
        int32_t delta2_mag = (pong_word >> 8) & 0x7FF;
        int32_t delta2 = dir_d2 ? -delta2_mag : delta2_mag;
        g_current_encoder_position += delta2; g_current_encoder_position += delta1;
        g_current_speed_rad_per_sec = 0.0f;
        log_msg("RECOVERY", "Miss-1 Resync OK: D2=" + std::to_string(delta2) + ", D1=" + std::to_string(delta1) + ". New Pos: " + std::to_string(g_current_encoder_position));
        g_successful_miss1_recoveries++;
    } else { // Normal Frame
        bool dir_pos = (pong_word >> 31) & 0x01;
        int32_t delta_pos_mag = (pong_word >> 20) & 0x7FF;
        int32_t delta_pos = dir_pos ? -delta_pos_mag : delta_pos_mag;
        g_current_encoder_position += delta_pos;
        bool dir_spd = (pong_word >> 19) & 0x01;
        int32_t speed_mag_q_lower_19 = pong_word & 0x0007FFFF;
        int32_t speed_fixed = speed_mag_q_lower_19;
        if (dir_spd) speed_fixed = -speed_fixed;
        g_current_speed_rad_per_sec = (float)speed_fixed / Cfg.q_scale_speed;
    }
}

void print_final_statistics() {
    log_msg("STATS", "---------------- FINAL CUMULATIVE STATISTICS -----------------");
    log_msg("STATS", "Total PINGs Sent: " + std::to_string(g_pings_sent_total));
    log_msg("STATS", "PING TX Fails: " + std::to_string(g_ping_tx_fails_total));
    uint32_t pings_tx_ok = g_pings_sent_total - g_ping_tx_fails_total; // Declare here
    log_msg("STATS", "PINGs TX OK: " + std::to_string(pings_tx_ok));
    log_msg("STATS", "PONG Bursts RX OK (Both Pkts): " + std::to_string(g_bursts_received_ok_total));
    log_msg("STATS", "PONG Pkt1 RX Timeouts: " + std::to_string(g_burst_pkt1_timeouts_total));
    log_msg("STATS", "PONG Pkt2 Fails (Pkt1 was OK): " + std::to_string(g_burst_pkt2_fails_after_pkt1_ok_total));
    log_msg("STATS", "CRC Errors on Recovery Frames: " + std::to_string(g_crc_errors_detected));
    double overall_burst_success_rate = (pings_tx_ok > 0) ? (static_cast<double>(g_bursts_received_ok_total) / pings_tx_ok * 100.0) : 0.0;
    log_msg("STATS", "Overall PONG Burst Success Rate (Both Pkts OK / PINGs TX OK): " + std::to_string(overall_burst_success_rate) + "%");
    log_msg("STATS", "Zero Position Requests Sent: " + std::to_string(g_zero_requests_sent));
    log_msg("STATS", "Miss-1 Recovery Requests Sent: " + std::to_string(g_miss1_recovery_requests_sent));
    log_msg("STATS", "Successful Miss-1 Recoveries: " + std::to_string(g_successful_miss1_recoveries));
    log_msg("STATS", "Resync Absolute Requests Sent: " + std::to_string(g_resync_absolute_requests_sent));
    log_msg("STATS", "Successful Resync Absolute Recoveries: " + std::to_string(g_successful_resync_abs_recoveries));
    if (!g_rtt_to_first_packet_us_all.empty()) { double sum_rtt = std::accumulate(g_rtt_to_first_packet_us_all.begin(), g_rtt_to_first_packet_us_all.end(), 0.0); double avg_rtt = sum_rtt / g_rtt_to_first_packet_us_all.size(); auto mm_rtt = std::minmax_element(g_rtt_to_first_packet_us_all.begin(), g_rtt_to_first_packet_us_all.end()); log_msg("STATS", "RTT (PING TX_DS to Pkt1 RX_DR) in us: [Min: " + std::to_string(*mm_rtt.first) + " | Avg: " + std::to_string(avg_rtt) + " | Max: " + std::to_string(*mm_rtt.second) + "]"); }
    if (!g_burst_duration_on_jetson_us_all.empty()) { double sum_burst = std::accumulate(g_burst_duration_on_jetson_us_all.begin(), g_burst_duration_on_jetson_us_all.end(), 0.0); double avg_burst = sum_burst / g_burst_duration_on_jetson_us_all.size(); auto mm_burst = std::minmax_element(g_burst_duration_on_jetson_us_all.begin(), g_burst_duration_on_jetson_us_all.end()); log_msg("STATS", "Jetson Burst Duration (Pkt1 RX_DR to Pkt2 RX_DR) in us: [Min: " + std::to_string(*mm_burst.first) + " | Avg: " + std::to_string(avg_burst) + " | Max: " + std::to_string(*mm_burst.second) + "]"); }
    log_msg("STATS", "------------------------------------------------------------");
}

int main(int argc, char *argv[]) {
    if (argc > 1 && std::string(argv[1]) == "--debug") { g_debug_spi = true; log_msg("INFO", "Debug SPI mode enabled."); }
    struct sigaction sa; memset(&sa, 0, sizeof(sa)); sa.sa_handler = sig_handler; sigaction(SIGINT, &sa, NULL); sigaction(SIGTERM, &sa, NULL);
    log_msg("INFO", "Jetson nRF24L01+ Encoder RX (Corrected) Starting...");

    if (!SPI1_NRF::initialize(90)) { log_msg("FATAL", "Failed to initialize BitBangSPI."); return 1; }
    SPI1_NRF::setDebug(g_debug_spi);
    if (!radio_init_jetson_prx()) { log_msg("FATAL", "Failed to initialize nRF radio."); SPI1_NRF::terminate(); return 1; }

    uint32_t stm_pong_pkt1_word, stm_pong_pkt2_word;
    const auto cycle_period = std::chrono::microseconds(1000000 / Cfg.target_hz);
    auto next_ping_time = std::chrono::steady_clock::now();

    log_msg("INFO", "Starting main PING loop at " + std::to_string(Cfg.target_hz) + " Hz.");

    while (g_running) {
        uint8_t current_ping_flags = FLAG_NONE;
        if (g_is_first_ping) {
            current_ping_flags |= FLAG_REQUEST_ZERO_POSITION; g_zero_requests_sent++;
        } else if (g_consecutive_missed_pongs == 1) {
            current_ping_flags |= FLAG_REQUEST_RESYNC_LAST_TWO; g_miss1_recovery_requests_sent++;
        } else if (g_consecutive_missed_pongs > 1) {
            current_ping_flags |= FLAG_REQUEST_RESYNC_ABSOLUTE; g_resync_absolute_requests_sent++;
        }
        if ((current_ping_flags & FLAG_ILLEGAL_RECOVERY_COMBO) == FLAG_ILLEGAL_RECOVERY_COMBO) {
            current_ping_flags = FLAG_REQUEST_RESYNC_ABSOLUTE;
            log_msg("WARN_JETSON", "Internal: tried illegal flag combo. Defaulting to Absolute Resync.");
        }
        g_last_ping_flags_sent = current_ping_flags;

        g_pings_sent_total++;
        if (jetson_send_ping(current_ping_flags)) {
            auto t_ping_tx_done = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t_pkt1_event, t_pkt2_event;
            if (jetson_receive_stm_burst(stm_pong_pkt1_word, stm_pong_pkt2_word, t_pkt1_event, t_pkt2_event)) {
                g_bursts_received_ok_total++;
                double rtt_us = std::chrono::duration_cast<std::chrono::microseconds>(t_pkt1_event - t_ping_tx_done).count();
                g_rtt_to_first_packet_us_all.push_back(rtt_us);
                double burst_jetson_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(t_pkt2_event - t_pkt1_event).count();
                g_burst_duration_on_jetson_us_all.push_back(burst_jetson_duration_us);

                if (stm_pong_pkt1_word != stm_pong_pkt2_word) {
                    log_msg("WARN", "PONG Burst Mismatch! P1: " + format_hex_val(stm_pong_pkt1_word) + " P2: " + format_hex_val(stm_pong_pkt2_word));
                }
                decode_and_update_state(stm_pong_pkt1_word);

                if (g_debug_spi || (g_pings_sent_total % (Cfg.target_hz / Cfg.data_log_interval_hz_divisor + 1) == 0) ) {
                    std::stringstream log_data;
                    log_data << "Pos: " << std::setw(7) << g_current_encoder_position
                             << " (" << std::fixed << std::setprecision(3) << std::setw(7) << (float)g_current_encoder_position / Cfg.counts_per_revolution << "r)"
                             << ", Spd: " << std::fixed << std::setprecision(3) << std::setw(7) << g_current_speed_rad_per_sec << "rad/s";
                    if(g_last_ping_flags_sent != FLAG_NONE) log_data << " (RecvMode: " << format_hex_byte(g_last_ping_flags_sent) << ")";
                    log_msg("DATA", log_data.str());
                }
                g_consecutive_missed_pongs = 0; g_is_first_ping = false;
            } else {
                if (g_running) { log_msg("WARN", "PONG Burst RX FAIL. Ping Flags: " + format_hex_byte(current_ping_flags)); g_consecutive_missed_pongs++; }
            }
        } else {
             if(g_running) g_consecutive_missed_pongs++;
        }

        next_ping_time += cycle_period;
        auto current_time = std::chrono::steady_clock::now();
        if (next_ping_time < current_time) next_ping_time = current_time;
        if (g_running) std::this_thread::sleep_until(next_ping_time);
    }

    log_msg("INFO", "Exiting. Final Stats:"); print_final_statistics();
    SPI1_NRF::ceLow(); write_register(REG_CONFIG, read_register(REG_CONFIG) & ~CONFIG_PWR_UP);
    log_msg("INFO", "nRF Powered Down."); SPI1_NRF::terminate(); log_msg("INFO", "BitBangSPI Terminated.");
    return 0;
}