#ifndef BITBANGSPI_HPP
#define BITBANGSPI_HPP

#include <jetgpio.h>  // For Jetson GPIO control
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

        // IMPORTANT: Pull-up for IRQ pin (GPIO 12)
        // The nRF24L01+ IRQ pin is active-LOW, typically open-drain.
        // An EXTERNAL PULL-UP RESISTOR (e.g., 4.7k-10k Ohm to 3.3V) on the IRQ line IS STRONGLY RECOMMENDED.
        // The jetgpio library call for internal pull-up caused an error, so it's removed.
        // Without a pull-up, the IRQ line might float and give unreliable readings.
        // std::cout << "[BitBangSPI] INFO: Ensure an external pull-up resistor is on IRQ pin (GPIO " 
        //           << PIN_IRQ << ")." << std::endl;


        // Set initial idle states for nRF24L01+
        gpioWrite(PIN_MOSI, 0);       // MOSI low
        gpioWrite(PIN_SCK, 0);        // SCK low
        gpioWrite(PIN_CSN, 1);        // CSN high (inactive)
        gpioWrite(PIN_CE, 0);         // CE low (standby)

        // Elevate process to real-time scheduling
        sched_param sp_params;
        sp_params.sched_priority = prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp_params) != 0) {
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

    // Low-level SPI transfer of a single byte.
    // Assumes CSN is already handled (e.g., by the calling xfer function).
    // Targets SPI_FREQ_HZ.
    static uint8_t transfer_byte(uint8_t byte_to_send) {
        uint64_t timer_freq = get_timer_frequency_hz();
        if (timer_freq == 0) { // Should not happen if initialized
             std::cerr << "[BitBangSPI] ERROR: Timer not calibrated in transfer_byte." << std::endl;
             return 0xFF; // Error
        }
        // Calculate half-period delay in timer cycles for the target SPI frequency
        const uint64_t spi_clk_half_period_cycles = timer_freq / (2 * SPI_FREQ_HZ);
        
        uint8_t received_byte = 0;
        for (int bit_idx = 7; bit_idx >= 0; --bit_idx) {
            // Set MOSI for the current bit
            gpioWrite(PIN_MOSI, (byte_to_send >> bit_idx) & 1);
            // asm volatile("nop;"); // Small delay, likely not needed with cycle counting

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

    // Multi-byte SPI transfer. Manages CSN (pulls low before, high after).
    // tx_buffer: Pointer to bytes to send. If null, 0xFF (NOP) is sent for each byte.
    // rx_buffer: Buffer to store received bytes. If null, received bytes are discarded.
    // num_bytes: Number of bytes to transfer.
    static void xfer(const uint8_t* tx_buffer, uint8_t* rx_buffer, size_t num_bytes) {
        if (num_bytes == 0) return;

        gpioWrite(PIN_CSN, 0); // Assert CSN (active low)

        for (size_t i = 0; i < num_bytes; ++i) {
            uint8_t byte_out = tx_buffer ? tx_buffer[i] : 0xFF;
            uint8_t byte_in = transfer_byte(byte_out);
            if (rx_buffer) {
                rx_buffer[i] = byte_in;
            }
        }
        gpioWrite(PIN_CSN, 1); // De-assert CSN
    }

    // Control nRF24L01+ CE (Chip Enable) pin.
    static void ceHigh() { gpioWrite(PIN_CE, 1); }
    static void ceLow()  { gpioWrite(PIN_CE, 0); }

    // Check nRF24L01+ IRQ pin state.
    // Returns true if IRQ is asserted (LOW), false otherwise.
    static bool isDataReady() {
        return gpioRead(PIN_IRQ) == 0; // IRQ is active LOW
    }

    // --- Configuration Helpers (optional, but can be convenient) ---
    // These are examples; the main application code can also write registers directly.

    // Example: Configure for basic ShockBurst without Auto-Acknowledgement.
    // Matches the setup in the provided test plan more closely.
    static void config_shockburst_no_ack(uint8_t channel, const uint8_t address[3], uint8_t payload_size) {
        if (debug_output_enabled_.load(std::memory_order_relaxed)) {
            std::cout << "[BitBangSPI] Configuring No-ACK mode: CH=" << (int)channel 
                      << " Addr=" << std::hex << (int)address[0] << (int)address[1] << (int)address[2] << std::dec
                      << " PLen=" << (int)payload_size << std::endl;
        }
        
        uint8_t cmd_buf[6]; // Max needed for address write + command

        ceLow();
        std::this_thread::sleep_for(std::chrono::microseconds(5000)); // Power down settling

        // CONFIG: PWR_UP=0 initially
        cmd_buf[0] = 0x20 | 0x00; cmd_buf[1] = 0x00; // W_REGISTER for CONFIG, value 0x00 (PWR_DOWN)
        xfer(cmd_buf, nullptr, 2);
        std::this_thread::sleep_for(std::chrono::microseconds(2000));


        // EN_AA: Disable Auto-Acknowledgement on all pipes
        cmd_buf[0] = 0x20 | 0x01; cmd_buf[1] = 0x00; // W_REGISTER for EN_AA, value 0x00
        xfer(cmd_buf, nullptr, 2);

        // EN_RXADDR: Enable RX Address on Pipe 0 (can be changed by app)
        cmd_buf[0] = 0x20 | 0x02; cmd_buf[1] = 0x01; // W_REGISTER for EN_RXADDR, value 0x01
        xfer(cmd_buf, nullptr, 2);

        // SETUP_AW: Address Width (0x01 for 3 bytes, 0x02 for 4, 0x03 for 5)
        cmd_buf[0] = 0x20 | 0x03; cmd_buf[1] = 0x01; // 3-byte address
        xfer(cmd_buf, nullptr, 2);

        // SETUP_RETR: Disable Auto-Retransmit
        cmd_buf[0] = 0x20 | 0x04; cmd_buf[1] = 0x00; // W_REGISTER for SETUP_RETR, value 0x00
        xfer(cmd_buf, nullptr, 2);

        // RF_CH: Set RF Channel
        cmd_buf[0] = 0x20 | 0x05; cmd_buf[1] = channel & 0x7F; // Channel 0-125
        xfer(cmd_buf, nullptr, 2);

        // RF_SETUP: 0dBm TX power, 2Mbps data rate, LNA gain high
        // Bits: CONT_WAVE(0), RF_DR_LOW(0), PLL_LOCK(0), RF_DR_HIGH(1), RF_PWR(11) -> 00011110 = 0x0E
        // For 2Mbps RF_DR_HIGH = 1. For 1Mbps RF_DR_LOW=0, RF_DR_HIGH=0. (Mistake here, 2Mbps is RF_DR_HIGH=1, LSB of RF_DR_LOW is reserved)
        // Correct 2Mbps: Bit 3 (RF_DR_HIGH) = 1. Bit 5 (RF_DR_LOW) = 0.
        // So for 2Mbps, 0dBm: 0b00001110 = 0x0E
        cmd_buf[0] = 0x20 | 0x06; cmd_buf[1] = 0x0E;
        xfer(cmd_buf, nullptr, 2);

        // TX_ADDR: Set Transmit address (use same as Pipe 0 for PING-PONG)
        cmd_buf[0] = 0x20 | 0x10; // W_REGISTER for TX_ADDR
        memcpy(cmd_buf + 1, address, 3);
        xfer(cmd_buf, nullptr, 1 + 3);

        // RX_ADDR_P0: Set Receive address for Pipe 0
        cmd_buf[0] = 0x20 | 0x0A; // W_REGISTER for RX_ADDR_P0
        memcpy(cmd_buf + 1, address, 3);
        xfer(cmd_buf, nullptr, 1 + 3);
        
        // RX_PW_P0: Set payload width for Pipe 0
        cmd_buf[0] = 0x20 | 0x11; cmd_buf[1] = payload_size;
        xfer(cmd_buf, nullptr, 2);

        // FIFO_STATUS: (Read-only, but useful for debug)
        // DYNPD: Disable Dynamic Payload Length for all pipes (default, but good to be explicit)
        cmd_buf[0] = 0x20 | 0x1C; cmd_buf[1] = 0x00;
        xfer(cmd_buf, nullptr, 2);
        // FEATURE: Disable EN_DPL, EN_ACK_PAY, EN_DYN_ACK (default, but good to be explicit if not using these)
        cmd_buf[0] = 0x20 | 0x1D; cmd_buf[1] = 0x00;
        xfer(cmd_buf, nullptr, 2);


        // FLUSH FIFOs
        cmd_buf[0] = 0xE1; xfer(cmd_buf, nullptr, 1); // FLUSH_TX
        cmd_buf[0] = 0xE2; xfer(cmd_buf, nullptr, 1); // FLUSH_RX

        // Clear STATUS register interrupt flags
        cmd_buf[0] = 0x20 | 0x07; cmd_buf[1] = 0x70; // Write 1s to RX_DR, TX_DS, MAX_RT
        xfer(cmd_buf, nullptr, 2);

        // CONFIG: Power up, CRC enabled (1-byte), Start in Standby-I (PRIM_RX=0 or 1 as needed by app)
        // App should set PRIM_RX as needed after this. For now, leave in Standby-I, PRIM_RX=0
        cmd_buf[0] = 0x20 | 0x00; cmd_buf[1] = 0x0A; // PWR_UP=1, EN_CRC=1 (1-byte), PRIM_RX=0
        xfer(cmd_buf, nullptr, 2);
        std::this_thread::sleep_for(std::chrono::microseconds(1500)); // Tpd2stby
    }
};

#endif // BITBANGSPI_HPP