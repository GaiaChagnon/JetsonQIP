/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Encoder + NRF24L01+ TX (Corrected)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h> // For memcpy, memset
#include <stdio.h>  // For snprintf (optional debug)
#include <stdbool.h>
#include "core_cm4.h" // For DWT
#include <math.h>     // For fabsf, M_PI

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int32_t position;
    uint32_t timestamp_cycles; // DWT->CYCCNT value
} PulseRecord_t;

typedef struct {
    uint16_t ppr;
    uint32_t counts_per_revolution;
    uint8_t  speed_calc_pulse_count;
    float    rad_per_count;
    float    q_scale_speed;
    uint32_t max_dt_cycles_for_speed;
    uint32_t speed_data_stale_timeout_cycles;
} EncoderConfig_t;

typedef struct {
    uint32_t pre_tx_delay_us;
    uint32_t tx_ds_poll_timeout_us;
    uint32_t ce_pulse_duration_us;
    uint8_t  rf_channel;
} RadioTimingConfig_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// THIS MUST BE A COMPILE-TIME CONSTANT FOR ARRAY DECLARATION
#define SPEED_CALC_PULSE_COUNT_CONST 5

const EncoderConfig_t AppEncoderConfig = {
    .ppr = 5000,
    .counts_per_revolution = 5000 * 4,
    .speed_calc_pulse_count = SPEED_CALC_PULSE_COUNT_CONST, // Use the define here
    .rad_per_count = (2.0f * M_PI) / (5000.0f * 4.0f),
    .q_scale_speed = 16384.0f, // Q6.14 (1 << 14)
    .max_dt_cycles_for_speed = 200000000, // Approx 2s @100MHz
	.speed_data_stale_timeout_cycles = 10000000
};

const RadioTimingConfig_t AppRadioConfig = {
    .pre_tx_delay_us = 1000,
    .tx_ds_poll_timeout_us = 250,
    .ce_pulse_duration_us = 12,
    .rf_channel = 76
};

static const uint8_t RADIO_ADDRESS[3] = {0xD7, 0xD7, 0xD7};
#define PING_PAYLOAD_SIZE      1
#define DATA_PAYLOAD_SIZE      4

// LED Configuration
#define LED_RED_PIN           GPIO_PIN_0
#define LED_RED_PORT          GPIOA
#define LED_GREEN_PIN         GPIO_PIN_1
#define LED_GREEN_PORT        GPIOA
#define LED_BLUE_PIN          GPIO_PIN_2
#define LED_BLUE_PORT         GPIOA
#define PULSE_LED_ON_DURATION_MS 5

// Encoder GPIO Configuration
#define ENC_CHA_PIN           GPIO_PIN_6
#define ENC_CHA_PORT          GPIOB
#define ENC_CHB_PIN           GPIO_PIN_7
#define ENC_CHB_PORT          GPIOB
#define ENC_CHI_PIN           GPIO_PIN_9
#define ENC_CHI_PORT          GPIOA

// NRF24L01+ HW
#define NRF_CE_PORT       GPIOB
#define NRF_CE_PIN        GPIO_PIN_0
#define NRF_CSN_PORT      GPIOB
#define NRF_CSN_PIN       GPIO_PIN_1

// NRF CMDS & REGS
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
#define CONFIG_CRCO         (1U << 2)
#define CONFIG_PWR_UP       (1U << 1)
#define CONFIG_PRIM_RX      (1U << 0)
#define FIFO_TX_EMPTY       (1U << 4)

// Jetson PING Flags
#define PING_FLAG_REQUEST_RESYNC_ABSOLUTE (1 << 0)
#define PING_FLAG_REQUEST_RESYNC_LAST_TWO (1 << 1)
#define PING_FLAG_ILLEGAL_RECOVERY_COMBO  (PING_FLAG_REQUEST_RESYNC_ABSOLUTE | PING_FLAG_REQUEST_RESYNC_LAST_TWO)
#define PING_FLAG_REQUEST_ZERO_POSITION   (1 << 7)

// CRC-4 ITU
#define CRC4_POLY 0x03
#define CRC4_BITS 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
volatile uint32_t hclk_mhz = 0;

volatile int32_t g_encoder_position = 0;
volatile int32_t g_position_for_turn_led = 0;
static volatile uint8_t g_last_encoded_state = 0;
const int8_t g_quadrature_lookup_table[16] = {
    0,  1, -1,  0, -1,  0,  0,  1,
    1,  0,  0, -1,  0, -1,  1,  0
};

// Use the compile-time constant for array dimension
volatile PulseRecord_t g_pulse_history[SPEED_CALC_PULSE_COUNT_CONST];
volatile uint8_t g_pulse_history_head_idx = 0;
volatile uint8_t g_pulse_history_valid_count = 0;

static int32_t g_last_transmitted_absolute_pos = 0;
static int32_t g_previous_pos_delta_for_recovery = 0;

volatile bool g_flag_turn_led_toggle = false;
volatile uint32_t g_pulse_led_off_time_ticks = 0;

volatile uint32_t g_last_pulse_dwt_timestamp = 0; // Time stamp of the absolute last valid pulse
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
static inline void DWT_Delay_us(volatile uint32_t microseconds);
static void NRF_CSN_LOW(void); static void NRF_CSN_HIGH(void); static void NRF_CE_LOW(void); static void NRF_CE_HIGH(void);
uint8_t spi_transfer_byte(uint8_t byte_out); uint8_t nrf_read_status(void); uint8_t nrf_read_register(uint8_t reg);
void nrf_write_register(uint8_t reg, uint8_t value); void nrf_write_register_multi(uint8_t reg, const uint8_t* data, size_t len);
void nrf_read_payload(uint8_t* buffer, size_t len); void nrf_write_payload(const uint8_t* data, size_t len);
void nrf_flush_tx_fifo(void); void nrf_flush_rx_fifo(void); void nrf_clear_interrupt_flags(void);
bool radio_init_stm32_ptx(void); void enter_tx_standby_config(void); void enter_rx_active_config(void);
uint8_t calculate_crc4_itu(uint32_t data_word, uint8_t num_data_bits);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DWT_Init(void) { CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; DWT->CYCCNT = 0; hclk_mhz = HAL_RCC_GetHCLKFreq() / 1000000; if (hclk_mhz == 0) hclk_mhz = 1; }
static inline void DWT_Delay_us(volatile uint32_t microseconds) { uint32_t clk_cycle_start = DWT->CYCCNT; uint32_t cycles_to_wait = microseconds * hclk_mhz; while (((DWT->CYCCNT - clk_cycle_start)) < cycles_to_wait); }
static void NRF_CSN_LOW() { HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET); }
static void NRF_CSN_HIGH() { HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET); }
static void NRF_CE_LOW() { HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET); }
static void NRF_CE_HIGH() { HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET); }
uint8_t spi_transfer_byte(uint8_t byte_out) { uint8_t byte_in = 0; HAL_SPI_TransmitReceive(&hspi1, &byte_out, &byte_in, 1, 100); return byte_in; }
uint8_t nrf_read_status() { NRF_CSN_LOW(); uint8_t status = spi_transfer_byte(CMD_NOP); NRF_CSN_HIGH(); return status; }
uint8_t nrf_read_register(uint8_t reg) { NRF_CSN_LOW(); spi_transfer_byte(CMD_R_REG | (reg & 0x1F)); uint8_t value = spi_transfer_byte(CMD_NOP); NRF_CSN_HIGH(); return value; }
void nrf_write_register(uint8_t reg, uint8_t value) { NRF_CSN_LOW(); spi_transfer_byte(CMD_W_REG | (reg & 0x1F)); spi_transfer_byte(value); NRF_CSN_HIGH(); }
void nrf_write_register_multi(uint8_t reg, const uint8_t* data, size_t len) { NRF_CSN_LOW(); spi_transfer_byte(CMD_W_REG | (reg & 0x1F)); HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, 100); NRF_CSN_HIGH(); }
void nrf_read_payload(uint8_t* buffer, size_t len) { NRF_CSN_LOW(); spi_transfer_byte(CMD_R_RX_PAYLOAD); HAL_SPI_Receive(&hspi1, buffer, len, 100); NRF_CSN_HIGH(); }
void nrf_write_payload(const uint8_t* data, size_t len) { NRF_CSN_LOW(); spi_transfer_byte(CMD_W_TX_PAYLOAD); HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, 100); NRF_CSN_HIGH(); }
void nrf_flush_tx_fifo() { NRF_CSN_LOW(); spi_transfer_byte(CMD_FLUSH_TX); NRF_CSN_HIGH(); }
void nrf_flush_rx_fifo() { NRF_CSN_LOW(); spi_transfer_byte(CMD_FLUSH_RX); NRF_CSN_HIGH(); }
void nrf_clear_interrupt_flags() { nrf_write_register(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT); }

bool radio_init_stm32_ptx() {
    NRF_CE_LOW(); NRF_CSN_HIGH(); HAL_Delay(100);
    nrf_write_register(REG_CONFIG, 0x00); HAL_Delay(5);
    nrf_write_register(REG_EN_AA, 0x00);
    nrf_write_register(REG_EN_RXADDR, 0x01);
    nrf_write_register(REG_SETUP_AW, 0x01);
    nrf_write_register(REG_SETUP_RETR, 0x00);
    nrf_write_register(REG_RF_CH, AppRadioConfig.rf_channel);
    nrf_write_register(REG_RF_SETUP, 0x0E);
    nrf_write_register_multi(REG_TX_ADDR, RADIO_ADDRESS, 3);
    nrf_write_register_multi(REG_RX_ADDR_P0, RADIO_ADDRESS, 3);
    nrf_write_register(REG_RX_PW_P0, PING_PAYLOAD_SIZE);
    nrf_flush_tx_fifo(); nrf_flush_rx_fifo(); nrf_clear_interrupt_flags();
    nrf_write_register(REG_CONFIG, CONFIG_PWR_UP | CONFIG_EN_CRC | CONFIG_PRIM_RX); // NRF HW CRC default 1 byte
    DWT_Delay_us(1500); NRF_CE_HIGH(); DWT_Delay_us(130);
    return (nrf_read_register(REG_RF_CH) == AppRadioConfig.rf_channel);
}
void enter_tx_standby_config() { NRF_CE_LOW(); nrf_write_register(REG_CONFIG, CONFIG_PWR_UP | CONFIG_EN_CRC); DWT_Delay_us(150); }
void enter_rx_active_config() { NRF_CE_LOW(); nrf_write_register(REG_CONFIG, CONFIG_PWR_UP | CONFIG_EN_CRC | CONFIG_PRIM_RX); DWT_Delay_us(150); NRF_CE_HIGH(); DWT_Delay_us(130); }

uint8_t calculate_crc4_itu(uint32_t data_word, uint8_t num_data_bits) {
    uint8_t crc = 0x00; // Initial value for CRC-4-ITU is typically 0
    uint32_t poly = CRC4_POLY;

    // Iterate over data bits from MSB. Data is in the upper bits of data_word.
    // CRC is calculated on the data portion *before* CRC bits are inserted.
    for (int i = num_data_bits - 1; i >= 0; i--) {
        bool data_bit_is_set = (data_word >> (i + CRC4_BITS)) & 1; // Check bit from data part
        bool crc_msb_is_set = (crc >> (CRC4_BITS - 1)) & 1;
        crc <<= 1;
        if (data_bit_is_set ^ crc_msb_is_set) {
            crc ^= poly;
        }
    }
    return crc & ((1 << CRC4_BITS) - 1); // Mask to 4 bits
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t ping_buffer[PING_PAYLOAD_SIZE];
  uint32_t pong_payload_word_to_tx;
  uint8_t data_payload_bytes[DATA_PAYLOAD_SIZE];

  int32_t latched_encoder_pos_main;
  // Use the compile-time constant for local array dimension
  PulseRecord_t local_pulse_history_main[SPEED_CALC_PULSE_COUNT_CONST];
  uint8_t local_pulse_history_valid_count_main;
  uint8_t local_pulse_history_head_idx_main; // To know where the newest data was in g_pulse_history

  float current_speed_rad_per_sec = 0.0f;
  int32_t pos_delta_this_frame = 0;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  DWT_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);

  __disable_irq();
  uint8_t initial_chA = HAL_GPIO_ReadPin(ENC_CHA_PORT, ENC_CHA_PIN);
  uint8_t initial_chB = HAL_GPIO_ReadPin(ENC_CHB_PORT, ENC_CHB_PIN);
  g_last_encoded_state = (initial_chA << 1) | initial_chB;
  memset((void*)g_pulse_history, 0, sizeof(g_pulse_history));
  g_pulse_history_head_idx = 0;
  g_pulse_history_valid_count = 0;
  g_last_pulse_dwt_timestamp = DWT->CYCCNT; // Initialize to current time
  __enable_irq();

  if (!radio_init_stm32_ptx()) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
  /* USER CODE BEGIN 3 */
    if (nrf_read_status() & STATUS_RX_DR) {
        uint32_t t_ping_rx_dr_cycles = DWT->CYCCNT;
        NRF_CE_LOW();
        nrf_read_payload(ping_buffer, PING_PAYLOAD_SIZE);
        nrf_write_register(REG_STATUS, STATUS_RX_DR);
        uint8_t jetson_ping_flags = ping_buffer[0];
        uint32_t last_isr_pulse_ts_main;

        __disable_irq();
        latched_encoder_pos_main = g_encoder_position;
        local_pulse_history_head_idx_main = g_pulse_history_head_idx; // Capture current head
        local_pulse_history_valid_count_main = g_pulse_history_valid_count; // Correctly copy the ring buffer relative to the captured head
        last_isr_pulse_ts_main = g_last_pulse_dwt_timestamp;

        for(uint8_t i = 0; i < AppEncoderConfig.speed_calc_pulse_count; ++i) {
            // local_pulse_history_main[i] will be newest ... oldest
            // Newest is at g_pulse_history[ (head - 1 + N) % N ]
            // Oldest for window of N is at g_pulse_history[ (head - N + N) % N ] = g_pulse_history [head]
            // So, local_pulse_history_main[0] should get g_pulse_history[ (local_pulse_history_head_idx_main - 1 + AppEncoderConfig.speed_calc_pulse_count) % AppEncoderConfig.speed_calc_pulse_count ]
            // and local_pulse_history_main[AppEncoderConfig.speed_calc_pulse_count-1] gets g_pulse_history[local_pulse_history_head_idx_main]
            uint8_t src_idx = (local_pulse_history_head_idx_main - 1 - i + AppEncoderConfig.speed_calc_pulse_count * 2) % AppEncoderConfig.speed_calc_pulse_count;

            if (local_pulse_history_valid_count_main > i) { // Only copy valid entries
                 local_pulse_history_main[i] = g_pulse_history[src_idx];

            } else {
                 memset(&local_pulse_history_main[i], 0, sizeof(PulseRecord_t)); // Zero out if not enough history
            }
        }
        __enable_irq();

        if (jetson_ping_flags & PING_FLAG_REQUEST_ZERO_POSITION) {
            __disable_irq();
            g_encoder_position = 0; g_position_for_turn_led = 0;
            g_last_transmitted_absolute_pos = 0;
            g_previous_pos_delta_for_recovery = 0;
            memset((void*)g_pulse_history, 0, sizeof(g_pulse_history));
            g_pulse_history_head_idx = 0;
            g_pulse_history_valid_count = 0;
            __enable_irq();
            last_isr_pulse_ts_main = DWT->CYCCNT;
            latched_encoder_pos_main = 0;
            local_pulse_history_valid_count_main = 0;
        }

        current_speed_rad_per_sec = 0.0f;
        uint32_t current_dwt_for_speed_check = DWT->CYCCNT;
        uint32_t time_since_last_isr_pulse = current_dwt_for_speed_check - last_isr_pulse_ts_main;

        if (time_since_last_isr_pulse > AppEncoderConfig.speed_data_stale_timeout_cycles) {
            // Speed data is stale (no pulses for a while), current_speed_rad_per_sec remains 0.0f
        } else if (local_pulse_history_valid_count_main >= AppEncoderConfig.speed_calc_pulse_count) {
            // Only calculate speed if data is NOT stale AND we have enough history
            // Newest is local_pulse_history_main[0], oldest in window is local_pulse_history_main[N-1]
            int32_t pos_diff = local_pulse_history_main[0].position -
                               local_pulse_history_main[AppEncoderConfig.speed_calc_pulse_count-1].position;
            uint32_t time_diff_cycles = local_pulse_history_main[0].timestamp_cycles -
                                        local_pulse_history_main[AppEncoderConfig.speed_calc_pulse_count-1].timestamp_cycles;

            if (time_diff_cycles > 0 && time_diff_cycles < AppEncoderConfig.max_dt_cycles_for_speed) {
                float time_diff_s = (float)time_diff_cycles / ((float)hclk_mhz * 1000000.0f); // Convert DWT cycles to seconds
                if (time_diff_s > 1e-7f) { // Min sensible time diff (0.1 us) to avoid division by zero/tiny
                    current_speed_rad_per_sec = (float)pos_diff * AppEncoderConfig.rad_per_count / time_diff_s;
                }
                // If time_diff_s is too small, speed remains 0.0f for this iteration
            }
            // If time_diff_cycles is not valid (0, negative, or too large), speed remains 0.0f for this iteration
        }
        // If not enough history (local_pulse_history_valid_count_main < N), speed remains 0.0f

        pos_delta_this_frame = latched_encoder_pos_main - g_last_transmitted_absolute_pos;
        pong_payload_word_to_tx = 0;
        uint8_t crc4_val = 0;

        if ((jetson_ping_flags & PING_FLAG_ILLEGAL_RECOVERY_COMBO) == PING_FLAG_ILLEGAL_RECOVERY_COMBO) {
             jetson_ping_flags = PING_FLAG_REQUEST_RESYNC_ABSOLUTE;
        }

        if (jetson_ping_flags & PING_FLAG_REQUEST_RESYNC_ABSOLUTE) {
            uint8_t dir_abs_flag = (latched_encoder_pos_main < 0);
            uint32_t abs_absolute_pos = (uint32_t)fabsf((float)latched_encoder_pos_main);

            if (abs_absolute_pos > 0x007FFFFF) abs_absolute_pos = 0x007FFFFF;
            pong_payload_word_to_tx = (dir_abs_flag << 31) | (abs_absolute_pos << 8);
            crc4_val = calculate_crc4_itu(pong_payload_word_to_tx, 28);
            pong_payload_word_to_tx |= crc4_val;

        } else if (jetson_ping_flags & PING_FLAG_REQUEST_RESYNC_LAST_TWO) {
            uint8_t dir_d1_flag = (pos_delta_this_frame < 0);
            uint32_t abs_d1 = (uint32_t)fabsf((float)pos_delta_this_frame);

            if (abs_d1 > 0x7FF) abs_d1 = 0x7FF;
            uint8_t dir_d2_flag = (g_previous_pos_delta_for_recovery < 0);
            uint32_t abs_d2 = (uint32_t)fabsf((float)g_previous_pos_delta_for_recovery);

            if (abs_d2 > 0x7FF) abs_d2 = 0x7FF;
            pong_payload_word_to_tx = (dir_d1_flag << 31) | (abs_d1 << 20) | \
                                      (dir_d2_flag << 19) | (abs_d2 << 8);
            crc4_val = calculate_crc4_itu(pong_payload_word_to_tx, 28);
            pong_payload_word_to_tx |= crc4_val;

        } else { // Normal Frame
            uint8_t dir_pos_flag = (pos_delta_this_frame < 0);
            uint32_t abs_pos_delta = (uint32_t)fabsf((float)pos_delta_this_frame);

            if (abs_pos_delta > 0x7FF) abs_pos_delta = 0x7FF;
            int32_t speed_q_fixed = (int32_t)(current_speed_rad_per_sec * AppEncoderConfig.q_scale_speed);
            uint8_t dir_spd_flag = (speed_q_fixed < 0);
            uint32_t speed_mag_19b = (uint32_t)(fabsf((float)speed_q_fixed)) & 0x0007FFFF;
            pong_payload_word_to_tx = (dir_pos_flag << 31) | (abs_pos_delta << 20) | \
                                      (dir_spd_flag << 19) | speed_mag_19b;
        }

        data_payload_bytes[0] = (uint8_t)(pong_payload_word_to_tx >> 0);
        data_payload_bytes[1] = (uint8_t)(pong_payload_word_to_tx >> 8);
        data_payload_bytes[2] = (uint8_t)(pong_payload_word_to_tx >> 16);
        data_payload_bytes[3] = (uint8_t)(pong_payload_word_to_tx >> 24);

        enter_tx_standby_config();
        uint32_t t_prep_done_cycles = DWT->CYCCNT;
        long long time_for_prep_us = (long long)(t_prep_done_cycles - t_ping_rx_dr_cycles) / hclk_mhz;
        long long final_wait_target_us = AppRadioConfig.pre_tx_delay_us - time_for_prep_us;

        if (final_wait_target_us > 0 && final_wait_target_us < (AppRadioConfig.pre_tx_delay_us + 500)) {
             DWT_Delay_us(final_wait_target_us);
        }

        bool pkt1_sent = false, pkt2_sent = false;
        nrf_flush_tx_fifo();
        nrf_write_payload(data_payload_bytes, DATA_PAYLOAD_SIZE);
        NRF_CE_HIGH(); DWT_Delay_us(AppRadioConfig.ce_pulse_duration_us); NRF_CE_LOW();
        uint32_t tx_start_cyc = DWT->CYCCNT;
        while(((DWT->CYCCNT - tx_start_cyc) / hclk_mhz) < AppRadioConfig.tx_ds_poll_timeout_us) {
            if (nrf_read_status() & STATUS_TX_DS) { nrf_write_register(REG_STATUS, STATUS_TX_DS); pkt1_sent = true; break; }
        }
        if(pkt1_sent) HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);

        if (pkt1_sent) {
            nrf_write_payload(data_payload_bytes, DATA_PAYLOAD_SIZE);
            NRF_CE_HIGH(); DWT_Delay_us(AppRadioConfig.ce_pulse_duration_us); NRF_CE_LOW();
            tx_start_cyc = DWT->CYCCNT;
            while(((DWT->CYCCNT - tx_start_cyc) / hclk_mhz) < AppRadioConfig.tx_ds_poll_timeout_us) {
                if (nrf_read_status() & STATUS_TX_DS) { nrf_write_register(REG_STATUS, STATUS_TX_DS); pkt2_sent = true; break; }
            }
        }
        if(pkt1_sent) { DWT_Delay_us(100); HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET); }
        if (!pkt1_sent || !pkt2_sent) { if (!(nrf_read_register(REG_FIFO_STATUS) & FIFO_TX_EMPTY)) nrf_flush_tx_fifo(); }

        g_last_transmitted_absolute_pos = latched_encoder_pos_main;
        g_previous_pos_delta_for_recovery = pos_delta_this_frame;
        enter_rx_active_config();
    }

    if (g_pulse_led_off_time_ticks > 0 && HAL_GetTick() >= g_pulse_led_off_time_ticks) {
        if (HAL_GPIO_ReadPin(LED_RED_PORT, LED_RED_PIN) == GPIO_PIN_SET) {
            HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
            g_pulse_led_off_time_ticks = 0;
        }
    }
    if (g_flag_turn_led_toggle) { HAL_GPIO_TogglePin(LED_BLUE_PORT, LED_BLUE_PIN); g_flag_turn_led_toggle = false; }

  /* USER CODE END 3 */
  }
}

void SystemClock_Config(void) { RCC_OscInitTypeDef RCC_OscInitStruct = {0}; RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; RCC_OscInitStruct.HSIState = RCC_HSI_ON; RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; RCC_OscInitStruct.PLL.PLLM = 8; RCC_OscInitStruct.PLL.PLLN = 100; RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; RCC_OscInitStruct.PLL.PLLQ = 4; if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler(); RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler(); }
static void MX_GPIO_Init(void) { GPIO_InitTypeDef GPIO_InitStruct = {0}; __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_RESET); GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET); HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET); GPIO_InitStruct.Pin = NRF_CE_PIN; GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; GPIO_InitStruct.Pull = GPIO_NOPULL; GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; HAL_GPIO_Init(NRF_CE_PORT, &GPIO_InitStruct); GPIO_InitStruct.Pin = NRF_CSN_PIN; HAL_GPIO_Init(NRF_CSN_PORT, &GPIO_InitStruct); GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; GPIO_InitStruct.Pull = GPIO_PULLUP; GPIO_InitStruct.Pin = ENC_CHA_PIN | ENC_CHB_PIN; HAL_GPIO_Init(ENC_CHA_PORT, &GPIO_InitStruct); GPIO_InitStruct.Pin = ENC_CHI_PIN; GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; HAL_GPIO_Init(ENC_CHI_PORT, &GPIO_InitStruct); HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); }
static void MX_SPI1_Init(void) { hspi1.Instance = SPI1; hspi1.Init.Mode = SPI_MODE_MASTER; hspi1.Init.Direction = SPI_DIRECTION_2LINES; hspi1.Init.DataSize = SPI_DATASIZE_8BIT; hspi1.Init.CLKPolarity = SPI_POLARITY_LOW; hspi1.Init.CLKPhase = SPI_PHASE_1EDGE; hspi1.Init.NSS = SPI_NSS_SOFT; hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB; hspi1.Init.TIMode = SPI_TIMODE_DISABLE; hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; hspi1.Init.CRCPolynomial = 10; if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler(); }

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ENC_CHA_PIN || GPIO_Pin == ENC_CHB_PIN) {
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    g_pulse_led_off_time_ticks = HAL_GetTick() + PULSE_LED_ON_DURATION_MS;

    uint32_t cb_dwt_now = DWT->CYCCNT;

    uint8_t chA = HAL_GPIO_ReadPin(ENC_CHA_PORT, ENC_CHA_PIN);
    uint8_t chB = HAL_GPIO_ReadPin(ENC_CHB_PORT, ENC_CHB_PIN);
    uint8_t current_state = (chA << 1) | chB;
    int8_t increment = g_quadrature_lookup_table[(g_last_encoded_state << 2) | current_state];

    if (increment != 0) {
      __disable_irq();
      g_encoder_position += increment;
      g_position_for_turn_led += increment;
      g_last_pulse_dwt_timestamp = cb_dwt_now;

      g_pulse_history[g_pulse_history_head_idx].position = g_encoder_position;
      g_pulse_history[g_pulse_history_head_idx].timestamp_cycles = cb_dwt_now;
      g_pulse_history_head_idx = (g_pulse_history_head_idx + 1) % AppEncoderConfig.speed_calc_pulse_count;

      if (g_pulse_history_valid_count < AppEncoderConfig.speed_calc_pulse_count) {
          g_pulse_history_valid_count++;
      }

      if (g_position_for_turn_led >= (int32_t)AppEncoderConfig.counts_per_revolution) {
        g_flag_turn_led_toggle = true; g_position_for_turn_led -= AppEncoderConfig.counts_per_revolution;

      } else if (g_position_for_turn_led <= -(int32_t)AppEncoderConfig.counts_per_revolution) {
        g_flag_turn_led_toggle = true; g_position_for_turn_led += AppEncoderConfig.counts_per_revolution;

      }

      __enable_irq();
    }
    g_last_encoded_state = current_state;
  } else if (GPIO_Pin == ENC_CHI_PIN) { /* Optional CHI handling */ }
}
/* USER CODE END 4 */

void Error_Handler(void) { __disable_irq(); HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET); HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, GPIO_PIN_SET); while (1) {}}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { Error_Handler(); }
#endif