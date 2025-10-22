/*

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(ESP32)

#include "SBUS_usart.h"
#include "esp32-hal.h"
#include "esp32-hal-uart.h"

#include <string.h>

#include "driver/uart.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "soc/uart_struct.h"   // direct UARTx register access
#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5
  // Use the UART driver’s event queue on IDF5+ (uart_isr_register() was removed)
  #include "freertos/FreeRTOS.h"
  #include "freertos/queue.h"
  #include "freertos/task.h"
#endif

// ------------------------ configuration ------------------------

#define SLOT_DATA_LENGTH            3
#define NUMBER_OF_FRAMES            4
#define NUMBER_OF_SLOT              32
#define NUMBER_OF_SLOT_IN_FRAME     8
#define SBUS_FRAME_SIZE             25

#if CONFIG_IDF_TARGET_ESP32
  #define SBUS2_UART_RX_PIN   (GPIO_NUM_25)
  #define SBUS2_UART_TX_PIN   (GPIO_NUM_26)
#elif CONFIG_IDF_TARGET_ESP32S2
  #define SBUS2_UART_RX_PIN   (2)
  #define SBUS2_UART_TX_PIN   (7)
#elif CONFIG_IDF_TARGET_ESP32S3
  // Choose safe pins (you can override when calling SBUS2_uart_setup)
  #define SBUS2_UART_RX_PIN   (12)
  #define SBUS2_UART_TX_PIN   (13)
#else
  #define SBUS2_UART_RX_PIN   (GPIO_NUM_6)
  #define SBUS2_UART_TX_PIN   (GPIO_NUM_5)
#endif

#define UART_RXBUFSIZE              30
#define SLOT_TIME                   660   // µs
#define WAIT_TIME                   2000  // µs
#define RX_BUF_SIZE                 1024  // UART driver RX ring size

// IDF version–portable inversion flags
#if ESP_IDF_VERSION_MAJOR >= 5
  #define UART_INV_RX  UART_SIGNAL_RXD_INV
  #define UART_INV_TX  UART_SIGNAL_TXD_INV
#else
  #define UART_INV_RX  UART_INVERSE_RXD
  #define UART_INV_TX  UART_INVERSE_TXD
#endif

// ------------------------ globals ------------------------

bool DemoMode = false;

// 32 slot IDs for telemetry
static const uint8_t Slot_ID[NUMBER_OF_SLOT] = {
  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3,
  0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB,
  0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB
};

static char sbus_frame[SBUS_FRAME_SIZE] = {
  0x0F, 0xA0, 0xA3, 0x20, 0x56, 0x2C, 0x08, 0x16, 0x50, 0x03,
  0x10, 0x80, 0x00, 0x04, 0x20, 0x00, 0x01, 0x08, 0x07, 0xC8,
  0x03, 0x10, 0x80, 0x02, 0x04
};

// channels for connected servos at the SBUS (declared in SBUS_usart.h)
static uint16_t channels[NUMBER_OF_CHANNELS];

static volatile uint8_t  rxbuf[UART_RXBUFSIZE];
static volatile uint8_t  sbusData[UART_RXBUFSIZE];
static volatile uint8_t  telemetryData[256];

static volatile bool     frame_ready      = false;
static volatile bool     telemetry_ready  = false;
static volatile bool     sbus_ready       = false;
static volatile uint8_t  gl_current_frame = 0;
static volatile uint16_t uart_lost_frame  = 0;
static volatile uint8_t  FER_count        = 0;
static volatile uint8_t  FER_buf[100];
static volatile uint8_t  buffer_index     = 0;

static volatile uint8_t  tx_pin;
static uart_port_t       uart_num;

typedef enum {
  EMPTY = 0,
  TRANSMITTING,
  AVAILABLE
} SLOT_DATA_STATUS;

static volatile int8_t   previousFrame   = 0;
static volatile uint32_t frameCounter    = 0;
static volatile uint8_t  sequence_count  = 0;   // transmit step
static volatile bool     transmit        = false;

static volatile SLOT_DATA_STATUS transmit_data_per_slot_status[NUMBER_OF_SLOT];
static volatile uint8_t         transmit_data_per_slot_data[NUMBER_OF_SLOT][SLOT_DATA_LENGTH];

#if ESP_IDF_VERSION_MAJOR >= 5
static QueueHandle_t s_uart_queue = nullptr;
static TaskHandle_t  s_uart_task  = nullptr;
#endif

// ------------------------ forward decls ------------------------

static void initialize_slot_status();
static void enable_receiving();
static void disable_receiving();
static void start_transmit_sequencer(uint8_t frame_number);
static void SBUS2_get_all_servo_data();

static void IRAM_ATTR ISR_transmit_frame(void *arg);
static void IRAM_ATTR uart_intr_handle(void *arg);  // used on IDF4.x

#if ESP_IDF_VERSION_MAJOR >= 5
static void uart_event_task(void *pvParameters);
static bool IRAM_ATTR timer_cb_wrapper(void *arg) { ISR_transmit_frame(arg); return false; }
#endif

// ------------------------ peripheral configs ------------------------

static uart_config_t uart_config = {
  .baud_rate  = 100000,
  .data_bits  = UART_DATA_8_BITS,
  .parity     = UART_PARITY_EVEN,
  .stop_bits  = UART_STOP_BITS_2,
  .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION_MAJOR >= 5
  .source_clk = UART_SCLK_DEFAULT
#endif
};

static timer_config_t timer_config = {
  .alarm_en     = TIMER_ALARM_EN,
  .counter_en   = TIMER_PAUSE,
  .intr_type    = TIMER_INTR_LEVEL,
  .counter_dir  = TIMER_COUNT_UP,
  .auto_reload  = TIMER_AUTORELOAD_EN,
  .divider      = 80       // 80 MHz APB -> 1 µs ticks
};

// ------------------------ public API ------------------------

void SBUS2_enable_simulation()  { log_i("[SBUS2] Demo Mode enabled");  DemoMode = true; }
void SBUS2_disable_simulation() { log_i("[SBUS2] Demo Mode disabled"); DemoMode = false; }

void SBUS2_uart_setup()                  { SBUS2_uart_setup(SBUS2_UART_RX_PIN, SBUS2_UART_TX_PIN, UART_NUM_1); }
void SBUS2_uart_setup(int rx, int tx)    { SBUS2_uart_setup(rx,               tx,               UART_NUM_1);  }

void SBUS2_uart_setup(int rx, int tx, int uart)
{
  tx_pin  = (uint8_t)tx;
  uart_num = (uart_port_t)uart;

  // Basic UART config
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

  // Invert RX/TX for SBUS
#if ESP_IDF_VERSION_MAJOR >= 5
  ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_INV_RX | UART_INV_TX));
#else
  ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_INV_RX | UART_INV_TX));
#endif

  ESP_ERROR_CHECK(uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Pull-down on RX to keep line stable when idle
  gpio_set_pull_mode((gpio_num_t)rx, GPIO_PULLDOWN_ONLY);
  gpio_pulldown_en((gpio_num_t)rx);

  // Install UART driver, with RX ring buffer and an event queue on IDF5+
#if ESP_IDF_VERSION_MAJOR >= 5
  ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, 0, 20, &s_uart_queue, 0));
#else
  ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, 0, 0, NULL, 0));
#endif

  // On IDF4: hook our low-level ISR. On IDF5: driver owns the ISR.
#if ESP_IDF_VERSION_MAJOR < 5
  ESP_ERROR_CHECK(uart_isr_free(uart_num));
  ESP_ERROR_CHECK(uart_isr_register(uart_num, uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, NULL));
  ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
#else
  ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
#endif

  // === Portable RX thresholds / timeout (replaces uart_intr_config + macros) ===
  // trigger interrupt when ~full SBUS frame is in FIFO and set RX timeout (~2 chars)
  uart_set_rx_full_threshold(uart_num, 26);  // SBUS frame is 25 bytes
  uart_set_rx_timeout(uart_num, 2);

  // Timer for telemetry TX sequence
  timer_init(TIMER_GROUP_0, TIMER_1, &timer_config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
  timer_set_alarm_value(TIMER_GROUP_0,   TIMER_1, SLOT_TIME);
  timer_enable_intr(TIMER_GROUP_0,       TIMER_1);

#if ESP_IDF_VERSION_MAJOR >= 5
  // IDF5+: use callback API
  ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, &timer_cb_wrapper, NULL, 0));
#else
  // IDF4.x: legacy ISR register
  timer_isr_register(TIMER_GROUP_0, TIMER_1, &ISR_transmit_frame, NULL, 0, NULL);
#endif

  timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);

  // Make sure RX ring is clean
#if ESP_IDF_VERSION_MAJOR >= 5
  uart_flush_input(uart_num);
#else
  uart_flush(uart_num);
#endif

  initialize_slot_status();
  enable_receiving();

#if ESP_IDF_VERSION_MAJOR >= 5
  // Spawn a tiny task to process UART events and read bytes from the driver RX ring
  if (s_uart_task == nullptr) {
    xTaskCreatePinnedToCore(uart_event_task, "sbus2_uart_evt", 2048, NULL, 20, &s_uart_task, ARDUINO_RUNNING_CORE);
  }
#endif

  if (DemoMode) {
    uart_write_bytes(uart_num, sbus_frame, SBUS_FRAME_SIZE);
    sbus_frame[24] += 0x10;
  } else {
    // detach TX from matrix unless we actually transmit
    pinMatrixOutDetach(tx_pin, false, true);
  }
}

// ------------------------ helpers ------------------------

static void initialize_slot_status() {
  for (uint8_t i = 0; i < NUMBER_OF_SLOT; i++) {
    transmit_data_per_slot_status[i] = EMPTY;
  }
}

static void disable_receiving() {
  sequence_count = 0;

  // Start 2ms timer to schedule slot #1 after a valid SBUS2 frame
  timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_DIS);
  timer_pause(TIMER_GROUP_0,  TIMER_1);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, WAIT_TIME);
  timer_set_counter_value(TIMER_GROUP_0,   TIMER_1, 0);
  timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);
  timer_start(TIMER_GROUP_0, TIMER_1);
}

static void enable_receiving() {
  // 15ms guard (approx, long enough to catch next frame timeout)
  timer_set_counter_value(TIMER_GROUP_0,   TIMER_1, 0);
  timer_set_alarm_value(TIMER_GROUP_0,     TIMER_1, 15000);
  timer_set_alarm(TIMER_GROUP_0,           TIMER_1, TIMER_ALARM_EN);

  // Reset buffer for new RX
  buffer_index = 0;
}

static inline void IncreaseTimer(int8_t frameNumber) {
  int8_t temp = (frameNumber - previousFrame) % NUMBER_OF_FRAMES;
  if (temp <= 0) temp += NUMBER_OF_FRAMES;
  if (temp > 1)  {
  	uart_lost_frame=(uart_lost_frame+1); 
  }
  frameCounter += temp;
  previousFrame = frameNumber;
}

// ------------------------ UART RX processing ------------------------

#if ESP_IDF_VERSION_MAJOR >= 5
// IDF5+: process UART driver events and read bytes from the RX ring
static void uart_event_task(void *pvParameters)
{
  (void)pvParameters;
  uart_event_t event;

  for (;;) {
    if (xQueueReceive(s_uart_queue, &event, portMAX_DELAY) != pdTRUE) continue;

    if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
      // reset RX ring if overflow
      uart_flush_input(uart_num);
      continue;
    }

    if (event.type != UART_DATA && event.type != UART_PATTERN_DET && event.type != UART_DATA_BREAK && event.type != UART_PARITY_ERR && event.type != UART_FRAME_ERR) {
      continue;
    }

    size_t avail = 0;
    uart_get_buffered_data_len(uart_num, &avail);

 while (avail--) {
  uint8_t b;
  int r = uart_read_bytes(uart_num, &b, 1, 0);
  if (r == 1) {
    if (buffer_index < UART_RXBUFSIZE) {
      rxbuf[buffer_index] = b;
      buffer_index++;
    }
  }
}


    // Mirror the original ISR logic to classify the received chunk
    if ((buffer_index == SBUS_FRAME_SIZE) && (rxbuf[0] == 0x0F)) {
      telemetry_ready = false;
      sbus_ready      = false;
      frame_ready     = true;

      for (uint8_t i = 0; i < UART_RXBUFSIZE; i++) {
        sbusData[i] = rxbuf[i];
      }

      disable_receiving();
      IncreaseTimer((rxbuf[24] & 0x30) >> 4);
      buffer_index = 0;

      if ((rxbuf[24] & 0x0F) == 0x04) {
        telemetry_ready = true;
        sbus_ready      = true;
        SBUS2_get_all_servo_data();
        start_transmit_sequencer(((rxbuf[24] & 0x30) >> 4));
      } else if ((rxbuf[24] & 0x0F) == 0x00) {
        telemetry_ready = false;
        sbus_ready      = true;
        SBUS2_get_all_servo_data();
      } else {
        uart_flush_input(uart_num);
        log_i("[SBUS2] Invalid Frame End Byte");
      }
    }
    else if (buffer_index == SBUS_FRAME_SIZE) {
      uart_flush_input(uart_num);
      log_i("[SBUS2] Invalid Frame Start Byte");
      buffer_index = 0;
    }
    else if ((buffer_index == SLOT_DATA_LENGTH) && (((rxbuf[0] & 0x0F) == 0x03) || ((rxbuf[0] & 0x0F) == 0x0B))) {
      uint8_t temp = rxbuf[0];
      telemetryData[temp]     = rxbuf[0];
      telemetryData[temp + 1] = rxbuf[1];
      telemetryData[temp + 2] = rxbuf[2];
      buffer_index = 0;
    }
    else if (buffer_index == SLOT_DATA_LENGTH) {
      uart_flush_input(uart_num);
      log_i("[SBUS2] Invalid Slot ID");
      buffer_index = 0;
    }
  }
}
#else
// IDF4.x: custom low-level ISR that reads UART1 FIFO directly
static void IRAM_ATTR uart_intr_handle(void *arg)
{
  (void)arg;

  // Detach TX while receiving
  pinMatrixOutDetach(tx_pin, false, true);

  // clear reasons
  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);

  uint16_t rx_fifo_len = UART1.status.rxfifo_cnt; // bytes available

  if (rx_fifo_len == 0) {
    return;
  } else if (rx_fifo_len == SBUS_FRAME_SIZE || rx_fifo_len == SLOT_DATA_LENGTH) {
    // ok
  } else {
    // length mismatch -> flush
    uart_flush(uart_num);
    uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    log_i("[SBUS2] Wrong Frame length");
    return;
  }

  while (rx_fifo_len) {
    rxbuf[buffer_index] = UART1.fifo.rw_byte; // read byte
    rx_fifo_len--;
    buffer_index++;
  }

  if ((buffer_index == SBUS_FRAME_SIZE) && (rxbuf[0] == 0x0F)) {
    telemetry_ready = false;
    sbus_ready      = false;
    frame_ready     = true;

    for (uint8_t i = 0; i < UART_RXBUFSIZE; i++) sbusData[i] = rxbuf[i];

    disable_receiving();
    IncreaseTimer((rxbuf[24] & 0x30) >> 4);
    buffer_index = 0;

    if ((rxbuf[24] & 0x0F) == 0x04) {
      telemetry_ready = true;
      sbus_ready      = true;
      SBUS2_get_all_servo_data();
      start_transmit_sequencer(((rxbuf[24] & 0x30) >> 4));
    } else if ((rxbuf[24] & 0x0F) == 0x00) {
      telemetry_ready = false;
      sbus_ready      = true;
      SBUS2_get_all_servo_data();
    } else {
      uart_flush(uart_num);
      uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
      log_i("[SBUS2] Invalid Frame End Byte");
    }
  }
  else if (buffer_index == SBUS_FRAME_SIZE) {
    uart_flush(uart_num);
    uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    log_i("[SBUS2] Invalid Frame Start Byte");
  }
  else if ((buffer_index == SLOT_DATA_LENGTH) && ((((rxbuf[0] & 0x0F) == 0x03) || ((rxbuf[0] & 0x0F) == 0x0B)))) {
    uint8_t temp  = rxbuf[0];
    telemetryData[temp]     = rxbuf[0];
    telemetryData[temp + 1] = rxbuf[1];
    telemetryData[temp + 2] = rxbuf[2];
    buffer_index = 0;
  }
  else if (buffer_index == SLOT_DATA_LENGTH) {
    uart_flush(uart_num);
    uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
    log_i("[SBUS2] Invalid Slot ID");
  }

  // clear UART interrupt status again
  uart_clear_intr_status(uart_num, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
}
#endif

// ------------------------ Timer ISR: transmit telemetry slots ------------------------

static void ISR_transmit_frame(void *arg)
{
  (void)arg;

#if ESP_IDF_VERSION_MAJOR >= 5
  timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
  timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_1);
#else
  TIMERG0.int_clr_timers.t1 = 1;
  TIMERG0.hw_timer[1].config.alarm_en = 1;
#endif

  // detach TX while preparing
  pinMatrixOutDetach(tx_pin, false, true);

  if (sequence_count < 8) {
    uint8_t actual_slot = (sequence_count) + (gl_current_frame * NUMBER_OF_SLOT_IN_FRAME);

    // if data available in slot then send it
    if (transmit_data_per_slot_status[actual_slot] == AVAILABLE) {
      pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);

      char buffer[SLOT_DATA_LENGTH];
      memcpy(buffer, (const void *)transmit_data_per_slot_data[actual_slot], SLOT_DATA_LENGTH);

      transmit_data_per_slot_status[actual_slot] = TRANSMITTING;
      uart_write_bytes(uart_num, buffer, SLOT_DATA_LENGTH);
      transmit_data_per_slot_status[actual_slot] = EMPTY;
    }

    // last slot has slightly longer pause
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    if (sequence_count == 7) {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 5150);
    } else {
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, SLOT_TIME);
    }
    timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_EN);
  } else {
    if (DemoMode) {
      pinMatrixOutAttach(tx_pin, U1TXD_OUT_IDX, false, false);
      uart_write_bytes(uart_num, sbus_frame, SBUS_FRAME_SIZE);
      sbus_frame[24] += 0x10;
      if (sbus_frame[24] > 0x34) sbus_frame[24] = 0x04;
    }

    buffer_index   = 0;
    transmit       = false;
    sequence_count = 0;
    enable_receiving();
  }

  sequence_count=(sequence_count+1);
}

static void start_transmit_sequencer(uint8_t frame_number)
{
  gl_current_frame = frame_number;
  // we don’t need more frames to be received while transmitting this one
  disable_receiving();
}

// ------------------------ public TX/RX helpers ------------------------

void SBUS2_transmit_telemetry_data(uint8_t slot, uint8_t data[SLOT_DATA_LENGTH])
{
  if (transmit_data_per_slot_status[slot] != TRANSMITTING) {
    transmit_data_per_slot_data[slot][0] = Slot_ID[slot];
    transmit_data_per_slot_data[slot][1] = data[1];
    transmit_data_per_slot_data[slot][2] = data[2];
    transmit_data_per_slot_status[slot]  = AVAILABLE;
  }
}

static void SBUS2_get_all_servo_data()
{
  channels[0]  = ((sbusData[1] | sbusData[2] << 8) & 0x07FF);
  channels[1]  = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
  channels[2]  = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10) & 0x07FF);
  channels[3]  = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
  channels[4]  = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
  channels[5]  = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9) & 0x07FF);
  channels[6]  = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
  channels[7]  = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF);
  channels[8]  = ((sbusData[12] | sbusData[13] << 8) & 0x07FF);
  channels[9]  = ((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF);
  channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
  channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF);
  channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF);
  channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF);
  channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF);
  channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF);

  channels[16] = (sbusData[23] & (1 << 0)) ? 1 : 0; // Digichannel 1
  channels[17] = (sbusData[23] & (1 << 1)) ? 1 : 0; // Digichannel 2

  if (sbusData[23] & 0x04) {
    FER_buf[FER_count] = 1;
  } else {
    FER_buf[FER_count] = 0;
  }
  FER_count=(FER_count+1);
  if (FER_count > 99) FER_count = 0;
}

void SBUS2_get_status(uint16_t *uart_dropped_frame, bool *transmission_dropt_frame, bool *failsafe)
{
  if (frameCounter < 60) { uart_lost_frame = 0; }

  *uart_dropped_frame       = uart_lost_frame;
  uart_lost_frame           = 0;
  *transmission_dropt_frame = (sbusData[23] & 0x04) ? true : false;
  *failsafe                 = (sbusData[23] & 0x08) ? true : false;
}

int16_t SBUS2_get_servo_data(uint8_t channel)
{
  if (channel < NUMBER_OF_CHANNELS) return channels[channel];
  return -1;
}

bool SBUS_Ready()
{
  if (sbus_ready) { sbus_ready = false; return true; }
  return false;
}

bool SBUS2_Ready()
{
  if (telemetry_ready) { telemetry_ready = false; return true; }
  return false;
}

bool SBUS_Ready(bool reset)
{
  if (sbus_ready) { if (reset) sbus_ready = false; return true; }
  return false;
}

bool SBUS2_Ready(bool reset)
{
  if (telemetry_ready) { if (reset) telemetry_ready = false; return true; }
  return false;
}

uint8_t SBUS_get_FER()
{
  uint8_t fer = 0;
  for (uint8_t i = 0; i < 100; i++) if (FER_buf[i] == 1) fer++;
  return fer;
}

uint8_t SBUS_get_RSSI()
{
  uint8_t rssi = SBUS_get_FER();
  rssi = 100 - rssi;
  return rssi;
}

void SBUS2_print_Raw()
{
  for (uint16_t i = 0; i < 256; i++) { Serial.print(i, HEX); Serial.print(" "); }
  Serial.println();
  for (uint16_t i = 0; i < 256; i++) { Serial.print(telemetryData[i], HEX); Serial.print(" "); }
  Serial.println();
}

bool SBUS2_get_Slot(uint8_t slot, uint8_t *lowbyte, uint8_t *highbyte)
{
  uint8_t slotid = Slot_ID[slot];
  if (telemetryData[slotid] == slotid) {
    *lowbyte  = telemetryData[slotid + 1];
    *highbyte = telemetryData[slotid + 2];
    telemetryData[slotid]     = 0;
    telemetryData[slotid + 1] = 0;
    telemetryData[slotid + 2] = 0;
    return true;
  } else {
    *lowbyte  = 0;
    *highbyte = 0;
    return false;
  }
}

void SBUS_disable()
{
  uart_disable_rx_intr(uart_num);
  timer_set_alarm(TIMER_GROUP_0, TIMER_1, TIMER_ALARM_DIS);
}

void SBUS_enable()
{
  uart_enable_rx_intr(uart_num);
  initialize_slot_status();
  enable_receiving();
}

void SBUS2_disable() { SBUS_disable(); }
void SBUS2_enable()  { SBUS_enable();  }

#endif // ESP32
