/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 * @defgroup nrf_twi_master_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Example Application main file.
 *
 * This file contains the source code for a sample application using TWI.
 */

#include "ADXL345.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "compiler_abstraction.h"
#include "diskio_blkdev.h"
#include "ff.h"
#include "nrf.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <stdarg.h>     //rjw
#include <__vfprintf.h> //rjw
#include <string.h>


#define MAX_PENDING_TRANSACTIONS 5

#define APP_TIMER_PRESCALER 0
#define APP_TIMER_OP_QUEUE_SIZE 2

#define MAX_TEST_DATA_BYTES (15U) /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256      /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256      /**< UART RX buffer size. */
#define DATA_BUFFER_SIZE 252      /**< data buffer for high frequency measurement. */

// added for SPI comms
#define FILE_NAME "A8_LOG.DAT"
#define CFG_FILE_NAME "ADXL_CFG.TXT"

//mod for BLE400
#define SDC_SCK_PIN SPIM0_SCK_PIN   ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN SPIM0_MOSI_PIN ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN SPIM0_MISO_PIN ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN SPIM0_SS_PIN     ///< SDC chip select (CS) pin.

//configure accelerometer data capture
uint32_t sizeFFT = 0x1000;              //no of datapoints for each measurement; power of 2 if want to FFT result
uint32_t timeToNextMeasurement = 8;    //in sec

//configure rtc timer
uint32_t tickFrequency = 900;           // 1111 ms; the filter coefficients are determined for a particular sampling frequency
#define TIME_0 28800                    // the reference duration of eight hours (28,800s)
volatile bool timeOut = false;
volatile bool tick = false;

#define NO_FILES 100      //total number to measure in this sequence
#define MAX_FILE_NO 10000 //last file no available
char filename[] = "adxl0000.DAT";
#define DIG_FILTER 1
#define USE_SD 1
#define TEST false
#define NO_TEST_DATA 512
#define SIZE_BUFF 49
char data[SIZE_BUFF + 1] = {0}; /* Line buffer */
char temp[SIZE_BUFF + 1] = {0}; /* Temporary buffer */


/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
    m_block_dev_sdc,
    NRF_BLOCK_DEV_SDC_CONFIG(
        SDC_SECTOR_SIZE,
        APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)),
    NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00"));

// end of SPI / SDC

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

bool got_callback;
bool button0_pressed = false;
bool button1_pressed = false;

extern int16_t xyz_int[3];
extern double xyz[3];

// Buffer for data read from sensors.
#define BUFFER_SIZE 11
uint8_t m_buffer[BUFFER_SIZE];


#if defined(__GNUC__) && (__LINT__ == 0)
// This is required if one wants to use floating-point values in 'printf'
// (by default this feature is not linked together with newlib-nano).
// Please note, however, that this adds about 13 kB code footprint...
__ASM(".global _printf_float");
#endif

////////////////////////////////////// coefficients for digital filter //////////////////////////////////////////

int32_t calibrationCoeffs[3][2] = {
    621,103,309,105,-455,104}; // [x,y,z] (x - a0) / a1 to convert 10 bit integer to acceleration in ms-2
    // measured for sum of 16 readings

int32_t filterCoeffs[3][6] = {
//{277,-511,237,13,1,-12},      {10222,670,1762,3163,6327,3163},	{2909,-5654,2751,2828,-5657,2828} // fs = 1000 Hz
//{292,-537,248,14,1,-13},	{16911,9440,3842,7548,15097,7548},	{2913,-5653,2746,2828,-5656,2828} // fs = 950 Hz
//{310,-567,260,16,1,-14},	{41084,46550,16780,26103,52207,26103},	{2918,-5652,2742,2828,-5656,2828} // fs = 900 Hz
{1445,-2614,1189,87,10,-76},  {589,1541,1132,815,1631,815},           {3000,-5787,2797,2896,-5792,2896} // fs = 800 Hz not stable
}; // [Hw,Hl,Hh][a0,a1,a2,b0,b1,b2]
// can scale these values by a fixed factor without change to the calculation

int32_t filteredValues[3][3][4] = {
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}},
    {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}};
// [rows][columns] [x,y,z][buffer index][aa,bb,cc,dd] HwHlHh converts successively from aa->dd

////////////////////////////////////// calibrate accelerometer measurements //////////////////////////////////////////
void calibrate(const int32_t coeffs[3][2], int32_t calibratedValues[3][3][4], int16_t measurements[3], uint8_t last) {
  uint8_t i;
  for (i = 0; i < 3; i++) {
    calibratedValues[i][last][0] = (((int32_t)measurements[i] << 8) - coeffs[i][0]) / coeffs[i][1];
  }
}

////////////////////////////////////// applyFilter //////////////////////////////////////////
void applyFilter(const int32_t filterCoeffs[3][6], int32_t filteredValues[3][3][4], uint8_t lastValue) {
  uint8_t i, j, k;
  for (i = 0; i < 3; i++) {   // over x, y, z
    for (j = 0; j < 3; j++) { // over Hw, Hl, Hh
      int32_t result = 0;
      uint8_t l = lastValue;
      for (k = 0; k < 3; k++) { // over cyclic buffer
        result += filterCoeffs[j][k + 3] * filteredValues[i][l][j];
        if (k != 0)
          result -= filterCoeffs[j][k] * filteredValues[i][l][j + 1];
        if (l == 0) {
          l = 2;
        } else {
          --l;
        }
      }
      filteredValues[i][lastValue][j + 1] = result / filterCoeffs[j][0];
    }
  }
}

////////////////////////////////////// uart_printf //////////////////////////////////////////

void uart_printf(const char *fmt, ...) {
#if APP_UART_ENABLED
  char buf[200], *p;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  for (p = buf; *p; ++p)
    app_uart_put(*p);
  va_end(ap);
#endif // APP_UART_ENABLED
}

////////////////////////////////////// uart_error_handle //////////////////////////////////////////

void uart_error_handle(app_uart_evt_t *p_event) {
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_communication);
  } else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_code);
  }
}

////////////////////////////////////// default call back //////////////////////////////////////////

void default_cb(ret_code_t result, void *p_user_data) {

  if (result != NRF_SUCCESS) {
    uart_printf("default_cb - error: %d\r\n", (int)result);
    return;
  }
  got_callback = true;
}
////////////////////////////////////// read_data call back //////////////////////////////////////////
// read_data and call back
// result of read is in m_buffer[]

void read_data_cb(ret_code_t result, void *p_user_data) {
  got_callback = true;
  if (result != NRF_SUCCESS) {
    uart_printf("read_data_cb - error: %d\r\n", (int)result);
    return;
  }
}

/////////////////////////////////////// read_data /////////////////////////////////////////
void read_data(uint8_t address) {
  static uint8_t register_address;
  register_address = address;
  static app_twi_transfer_t const transfers[] =
      {
          ADXL345_READ(&register_address, m_buffer, 1)

      };
  static app_twi_transaction_t const transaction =
      {
          .callback = read_data_cb,
          .p_user_data = NULL,
          .p_transfers = transfers,
          .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])};
  got_callback = false;

  APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}
////////////////////////////////////// read_xyz_data call back //////////////////////////////////////////
// read_data and call back
// result of read is in m_buffer[]

void read_xyz_data_cb(ret_code_t result, void *p_user_data) {
  if (result != NRF_SUCCESS) {
    uart_printf("read_data_cb - error: %d\r\n", (int)result);
    return;
  }
  // Each Axis @ All g Ranges: 10 Bit Resolution (2 Bytes)

  xyz_int[0] = (((int)m_buffer[1]) << 8) | m_buffer[0];
  xyz_int[1] = (((int)m_buffer[3]) << 8) | m_buffer[2];
  xyz_int[2] = (((int)m_buffer[5]) << 8) | m_buffer[4];

  get_Gxyz();

  for (int i = 0; i < 3; i++) {

    uart_printf("%6i = : %6i  ", i, xyz_int[i]);
  }
  uart_printf("\n");

  //  for (int i = 0; i < 3; i++) {
  //    uart_printf("%i = : %d.%d  ", i, NRF_LOG_FLOAT(xyz[i]));
  //    uart_printf("%i = : %8.3f  ", i,xyz[i]);
  //  }
  //  uart_printf("\n");
}
/////////////////////////////////////// read_xyz_data /////////////////////////////////////////
void read_xyz_data(uint8_t address) {

  // [these structures have to be "static" - they cannot be placed on stack
  //  since the transaction is scheduled and these structures most likely
  //  will be referred after this function returns]void read_xyz_data(uint8_t address) {

  static uint8_t register_address;
  register_address = address;
  static app_twi_transfer_t const transfers[] =
      {
          ADXL345_READ(&register_address, m_buffer, ADXL345_TO_READ)
          //         ADXL345_READ_XYZ_AND_TILT(&m_buffer[0])

      };
  static app_twi_transaction_t const transaction =
      {
          .callback = read_xyz_data_cb,
          .p_user_data = NULL,
          .p_transfers = transfers,
          .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])};

  APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

////////////////////////////////////// write_reg2 //////////////////////////////////////////
// value to write is in config
// uses twi_perform
void write_reg2(uint8_t address, uint8_t val) {

  static uint8_t config[2];
  config[0] = address;
  config[1] = val;

  app_twi_transfer_t const write_adxl345_data_format[] =
      {
          APP_TWI_WRITE(ADXL345_DEVICE, config, sizeof(config), 0)

      };
  APP_ERROR_CHECK(app_twi_perform(&m_app_twi, write_adxl345_data_format,
      1, NULL));
}

////////////////////////////////////////////////////////////////////////////////
// Buttons handling (by means of BSP).
// Only used in calibration run
//
static void bsp_event_handler(bsp_event_t event) {
  // Each time the button 1 or 4 is pushed we start a transaction reading
  // values of all registers from LM75B or MMA7660 respectively.
  switch (event) {
  case BSP_EVENT_KEY_0: // Button 0 pushed.
    button0_pressed = true;
    break;

  case BSP_EVENT_KEY_1: // Button 1 pushed.
    button1_pressed = true;
    break;

  default:
    break;
  }
}
static void bsp_config(void) {
  uint32_t err_code;

  APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

  err_code = bsp_init(BSP_INIT_BUTTONS,
      APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
      bsp_event_handler);
  APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////////////////////////////////////////////
// TWI (with transaction manager) initialization.
static void twi_config(void) {
  uint32_t err_code;

  nrf_drv_twi_config_t const config = {
      .scl = ARDUINO_SCL_PIN,
      .sda = ARDUINO_SDA_PIN,
      .frequency = NRF_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
      .clear_bus_init = false};

  APP_TWI_INIT(&m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);
  APP_ERROR_CHECK(err_code);
}

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on COMPARE0 match.
 */
////////////////////////////////////////////////////////////////////////////////
// RTC tick events generation.
static void rtc_handler(nrf_drv_rtc_int_type_t int_type) {
  if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
    timeOut = true;
  }
  if (int_type == NRF_DRV_RTC_INT_TICK) {
    tick = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize RTC instance with default configuration.
static void rtc_config(void) {
  uint32_t err_code;

  nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
  config.prescaler = RTC_FREQ_TO_PRESCALER(tickFrequency); //Set RTC frequency to 32Hz
  err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
  APP_ERROR_CHECK(err_code);

  // Power on RTC instance.
  nrf_drv_rtc_enable(&rtc);
}

////////////////////////////////////////////////////////////////////////////////

static void lfclk_config(void) {
  uint32_t err_code;

  err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);
}
//////////////////////////////////////flash_led()//////////////////////////////////////////

static void flash_led(uint16_t noTimes, bool forever) {
  do {
    for (uint16_t j = 0; j < noTimes; j++) {
      LEDS_INVERT(BSP_LED_0_MASK);
      nrf_delay_ms(100);
      LEDS_INVERT(BSP_LED_0_MASK);
      if (j < noTimes) nrf_delay_ms(150);
    }
    if (forever) {
      nrf_delay_ms(10000 - noTimes * 250);
    }
  } while (forever);
}
//////////////////////////////////////fatfs_init()//////////////////////////////////////////

/**
 * @brief Function for demonstrating FAFTS usage.
 */
static void fatfs_init() {
  FRESULT ff_result;
  static FATFS fs;
  DSTATUS disk_state = STA_NOINIT;

  // Initialize FATFS disk I/O interface by providing the block device.
  static diskio_blkdev_t drives[] =
      {
          DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)};

  diskio_blockdev_register(drives, ARRAY_SIZE(drives));

  uart_printf("Initializing disk 0 (SDC)...\r\n");
  for (uint32_t retries = 3; retries && disk_state; --retries) {
    disk_state = disk_initialize(0);
  }
  if (disk_state) {
    uart_printf("Disk initialization failed.\r\n");
    flash_led(4, true);
    return;
  }

  uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
  uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
  uart_printf("Capacity: %d MB\r\n", capacity);

  uart_printf("Mounting volume...\r\n");
  ff_result = f_mount(&fs, "", 1);
  if (ff_result) {
    uart_printf("Mount failed.\r\n");
    flash_led(5, true);
  }
}
////////////////////////////////////// readLine //////////////////////////////////////////
// read in variables according to counter that tracks which lines read 

bool readLine(char *lineFeed, uint16_t *lineNo) 
{
  bool gotError = false;
  if (*lineNo == 1) {
    int result = sscanf(
    lineFeed, "%i,%i,%i", 
    &sizeFFT, 
    &timeToNextMeasurement,
    &tickFrequency 
    );
    gotError = (result != 3);
  }
  else if  ((*lineNo > 1) && (*lineNo < 5))
    {
    int result = sscanf(
    lineFeed, "%i,%i", 
    &calibrationCoeffs[*lineNo - 2][0],
    &calibrationCoeffs[*lineNo - 2][1]
    );
    gotError = (result != 2);
    }
  else if  ((*lineNo > 4) && (*lineNo < 8))
    {
    int result = sscanf(
    lineFeed, "%i,%i,%i,%i,%i,%i", 
    &filterCoeffs[*lineNo - 5][0],
    &filterCoeffs[*lineNo - 5][1],
    &filterCoeffs[*lineNo - 5][2],
    &filterCoeffs[*lineNo - 5][3],
    &filterCoeffs[*lineNo - 5][4],
    &filterCoeffs[*lineNo - 5][5]
    );
    gotError = (result != 6);
    }
  return (gotError);
}
//////////////////////////////////////read_config()//////////////////////////////////////////

/**
 * @brief function to read in measurement parameters:  sample frequency, file size, repeat time, filter coefficients
 */
static void read_config() {
  FRESULT ff_result;
  static FIL file;
  uint32_t bytes_read;
  bytes_read = 0;
  const char newLine[2] = {"\n\0"};
  const char *srch_str = (char *)newLine;

  ff_result = f_open(&file, CFG_FILE_NAME, FA_READ);
  if (ff_result != FR_OK) {
//    printf("Unable to open or create file: " CFG_FILE_NAME ".\r\n");
    return;
  }

  uint16_t size = f_size(&file);
//  printf("size of the file in bytes = %d\r\n", size);

  uint16_t whichLine;
  whichLine = 1;

  while (file.fptr < size) {
//    printf("file ptr %i\r\n", (uint32_t)file.fptr);
    ff_result = f_read(&file, data, sizeof(data) - 1, (UINT *)&bytes_read); // the last item of data is a NULL terminator
//    printf("file ptr %i\r\n", (uint32_t)file.fptr);
   
    if (ff_result != FR_OK) {
//      printf("Unable to read or create file: " CFG_FILE_NAME ".\r\n");
      return;
    } else {
//      printf("%d bytes read\r\n", bytes_read);
    }
    char *p = (char *)data;
    char **new_data = &p;

    while (**new_data != NULL) // test for end of buffer
    {
      char *val;
      char *old_data = strsep(new_data, srch_str); //read to the next new line and set it to NULL
      if ((*(*new_data - 1)) == NULL)              // found new line
      {
        val = strncat(temp, (const char *)old_data, SIZE_BUFF);
        if(readLine(temp, &whichLine) == true) flash_led(3, true); // trap error 
        ++whichLine;
        *temp = NULL;
//        printf("float read %f,%f\r\n", num1, num2);
      } else {
//        int bytes_left = *new_data - old_data - 1;
//        printf("bytes left %i\n", bytes_left);
        val = strncpy(temp, (const char *)old_data, SIZE_BUFF - 1); //save tail of file
      }
    }
  }
  (void)f_close(&file);
  return;
}

////////////////////////////////////// file_printf //////////////////////////////////////////

void file_printf(FIL *fp, const char *fmt, ...) {
#if USE_SD
  uint32_t bytes_written;
  FRESULT ff_result;
  char buf[200], *p;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  ff_result = f_write(fp, buf, strlen(buf), &bytes_written);
  if (ff_result != FR_OK) {
    flash_led(3, true);
  }

  va_end(ap);
#endif // USE_SD
}
////////////////////////////////////////////////////////////////////////////////

int main(void) {
  uint16_t j = 0; // general counter

  uint32_t err_code; // added for UART
  uint32_t bytes_written;
  FRESULT ff_result;
  static FIL dataFile, a8File;

  //////////////////////////////////////// test digital filter ////////////////////////////////////////
#if (TEST)

uint8_t testData[6 * NO_TEST_DATA] = {
38,0,10,0,154,255,
11,0,233,255,179,255,
11,0,233,255,179,255,
19,0,228,255,229,255,
19,0,243,255,241,255,
19,0,243,255,241,255,
28,0,2,0,223,255,
30,0,12,0,227,255,
34,0,5,0,237,255,
40,0,251,255,216,255,
46,0,7,0,221,255,
46,0,7,0,221,255,
32,0,250,255,211,255,
11,0,250,255,163,255,
6,0,251,255,143,255,
252,255,1,0,165,255,
252,255,1,0,165,255,
243,255,5,0,191,255,
237,255,253,255,163,255,
249,255,6,0,133,255,
224,255,4,0,161,255,
224,255,4,0,161,255,
220,255,218,255,174,255,
222,255,232,255,164,255,
232,255,250,255,153,255,
223,255,247,255,172,255,
237,255,240,255,215,255,
237,255,240,255,215,255,
247,255,215,255,181,255,
250,255,245,255,156,255,
0,0,13,0,191,255,
21,0,238,255,192,255,
21,0,238,255,192,255,
24,0,250,255,184,255,
40,0,10,0,176,255,
36,0,246,255,189,255,
41,0,244,255,174,255,
45,0,19,0,173,255,
45,0,19,0,173,255,
48,0,18,0,197,255,
43,0,245,255,188,255,
39,0,9,0,175,255,
35,0,16,0,192,255,
35,0,16,0,192,255,
26,0,6,0,190,255,
16,0,29,0,206,255,
17,0,20,0,216,255,
9,0,236,255,209,255,
9,0,236,255,209,255,
8,0,9,0,224,255,
0,0,9,0,238,255,
236,255,250,255,221,255,
235,255,4,0,198,255,
223,255,247,255,187,255,
223,255,247,255,187,255,
217,255,8,0,195,255,
189,255,249,255,187,255,
180,255,248,255,195,255,
201,255,12,0,206,255,
201,255,12,0,206,255,
216,255,240,255,195,255,
240,255,7,0,229,255,
242,255,1,0,224,255,
228,255,234,255,183,255,
234,255,22,0,202,255,
234,255,22,0,202,255,
252,255,248,255,207,255,
250,255,236,255,206,255,
2,0,18,0,192,255,
16,0,17,0,195,255,
16,0,17,0,195,255,
40,0,9,0,193,255,
22,0,10,0,188,255,
20,0,17,0,157,255,
43,0,5,0,143,255,
43,0,5,0,143,255,
40,0,0,0,168,255,
28,0,2,0,200,255,
34,0,13,0,199,255,
47,0,242,255,174,255,
45,0,244,255,160,255,
45,0,244,255,160,255,
41,0,21,0,182,255,
28,0,7,0,191,255,
34,0,239,255,163,255,
15,0,249,255,166,255,
7,0,247,255,234,255,
7,0,247,255,234,255,
22,0,0,0,19,0,
19,0,14,0,30,0,
19,0,14,0,30,0,
18,0,13,0,14,0,
21,0,1,0,15,0,
22,0,255,255,12,0,
13,0,29,0,26,0,
10,0,5,0,0,0,
10,0,5,0,0,0,
3,0,252,255,222,255,
7,0,3,0,225,255,
7,0,12,0,238,255,
244,255,8,0,211,255,
244,255,8,0,211,255,
227,255,243,255,154,255,
233,255,12,0,152,255,
240,255,10,0,182,255,
244,255,226,255,168,255,
244,255,226,255,168,255,
237,255,247,255,150,255,
243,255,11,0,159,255,
231,255,241,255,153,255,
8,0,241,255,145,255,
28,0,253,255,151,255,
28,0,253,255,151,255,
31,0,241,255,148,255,
29,0,216,255,140,255,
28,0,227,255,138,255,
38,0,4,0,172,255,
38,0,4,0,172,255,
43,0,228,255,151,255,
33,0,229,255,129,255,
45,0,3,0,125,255,
43,0,237,255,131,255,
43,0,237,255,131,255,
41,0,239,255,155,255,
52,0,4,0,148,255,
52,0,252,255,139,255,
57,0,3,0,154,255,
33,0,244,255,183,255,
33,0,244,255,183,255,
38,0,238,255,190,255,
38,0,2,0,206,255,
29,0,18,0,244,255,
5,0,246,255,238,255,
5,0,246,255,238,255,
249,255,232,255,190,255,
14,0,15,0,203,255,
12,0,247,255,247,255,
252,255,255,255,4,0,
247,255,7,0,236,255,
247,255,7,0,236,255,
249,255,13,0,244,255,
243,255,20,0,24,0,
223,255,227,255,2,0,
253,255,248,255,212,255,
253,255,248,255,212,255,
10,0,24,0,195,255,
1,0,11,0,200,255,
20,0,10,0,215,255,
35,0,228,255,224,255,
35,0,228,255,224,255,
30,0,13,0,238,255,
13,0,254,255,209,255,
22,0,253,255,218,255,
38,0,0,0,225,255,
36,0,5,0,221,255,
36,0,5,0,221,255,
37,0,8,0,221,255,
55,0,224,255,192,255,
50,0,230,255,188,255,
47,0,19,0,217,255,
47,0,19,0,217,255,
33,0,249,255,195,255,
31,0,252,255,203,255,
38,0,2,0,206,255,
23,0,249,255,190,255,
23,0,249,255,190,255,
23,0,10,0,224,255,
23,0,240,255,235,255,
22,0,249,255,248,255,
19,0,12,0,247,255,
0,0,244,255,247,255,
142,255,8,0,50,255,
146,255,208,255,60,255,
151,255,191,255,56,255,
164,255,2,0,98,255,
200,255,248,255,152,255,
200,255,248,255,152,255,
227,255,219,255,213,255,
242,255,223,255,190,255,
245,255,221,255,124,255,
13,0,246,255,149,255,
13,0,246,255,149,255,
41,0,217,255,211,255,
49,0,213,255,212,255,
50,0,246,255,174,255,
62,0,251,255,138,255,
80,0,226,255,151,255,
80,0,226,255,151,255,
83,0,227,255,158,255,
73,0,249,255,122,255,
71,0,19,0,124,255,
89,0,0,0,174,255,
89,0,0,0,174,255,
81,0,240,255,193,255,
58,0,10,0,199,255,
59,0,3,0,219,255,
70,0,247,255,7,0,
73,0,232,255,45,0,
73,0,232,255,45,0,
44,0,215,255,15,0,
14,0,248,255,244,255,
11,0,5,0,19,0,
7,0,247,255,71,0,
7,0,247,255,71,0,
244,255,237,255,61,0,
222,255,236,255,2,0,
216,255,19,0,9,0,
231,255,254,255,54,0,
231,255,254,255,54,0,
218,255,229,255,61,0,
200,255,3,0,30,0,
196,255,247,255,237,255,
215,255,26,0,33,0,
246,255,22,0,74,0,
246,255,22,0,74,0,
229,255,237,255,56,0,
214,255,11,0,247,255,
205,255,22,0,180,255,
214,255,32,0,200,255,
214,255,32,0,200,255,
218,255,1,0,218,255,
201,255,250,255,179,255,
203,255,253,255,125,255,
221,255,7,0,124,255,
221,255,7,0,124,255,
230,255,20,0,153,255,
247,255,1,0,169,255,
7,0,252,255,203,255,
21,0,253,255,224,255,
24,0,7,0,203,255,
24,0,7,0,203,255,
20,0,252,255,124,255,
32,0,18,0,120,255,
43,0,246,255,163,255,
50,0,208,255,145,255,
50,0,208,255,145,255,
63,0,244,255,101,255,
63,0,1,0,99,255,
66,0,4,0,113,255,
54,0,254,255,152,255,
60,0,246,255,162,255,
60,0,246,255,162,255,
73,0,10,0,186,255,
73,0,8,0,228,255,
67,0,2,0,230,255,
72,0,21,0,231,255,
72,0,21,0,231,255,
61,0,37,0,35,0,
42,0,248,255,19,0,
252,255,226,255,215,255,
242,255,28,0,165,255,
242,255,28,0,165,255,
219,255,30,0,172,255,
187,255,231,255,182,255,
161,255,225,255,102,255,
156,255,252,255,249,254,
162,255,33,0,20,255,
16,0,192,255,157,255,
10,0,228,255,90,255,
14,0,7,0,40,255,
9,0,241,255,35,255,
35,0,246,255,82,255,
35,0,246,255,82,255,
61,0,243,255,78,255,
47,0,241,255,43,255,
53,0,3,0,65,255,
70,0,251,255,85,255,
70,0,251,255,85,255,
71,0,240,255,114,255,
74,0,225,255,119,255,
73,0,217,255,153,255,
86,0,217,255,176,255,
79,0,226,255,196,255,
79,0,226,255,196,255,
69,0,223,255,231,255,
78,0,244,255,4,0,
82,0,0,0,37,0,
58,0,255,255,33,0,
58,0,255,255,33,0,
34,0,247,255,17,0,
44,0,3,0,48,0,
53,0,3,0,96,0,
40,0,241,255,105,0,
40,0,241,255,105,0,
22,0,240,255,106,0,
16,0,251,255,108,0,
254,255,254,255,125,0,
251,255,15,0,161,0,
251,255,1,0,153,0,
251,255,1,0,153,0,
228,255,249,255,68,0,
208,255,9,0,209,255,
207,255,42,0,190,255,
217,255,7,0,228,255,
217,255,7,0,228,255,
163,255,211,255,171,255,
103,255,215,255,15,255,
113,255,0,0,187,254,
139,255,16,0,239,254,
139,255,16,0,239,254,
171,255,215,255,29,255,
190,255,185,255,228,254,
220,255,229,255,194,254,
232,255,252,255,215,254,
245,255,242,255,109,255,
245,255,242,255,109,255,
70,0,171,255,214,255,
67,0,136,255,151,255,
87,0,226,255,24,255,
133,0,179,255,20,255,
133,0,179,255,20,255,
201,0,221,255,161,255,
192,0,197,255,195,255,
170,0,189,255,100,255,
192,0,60,0,86,255,
169,0,12,0,115,255,
169,0,12,0,115,255,
145,0,238,255,173,255,
157,0,20,0,153,255,
141,0,52,0,119,255,
114,0,27,0,103,255,
114,0,27,0,103,255,
100,0,16,0,165,255,
87,0,4,0,215,255,
58,0,13,0,229,255,
10,0,32,0,253,255,
10,0,32,0,253,255,
228,255,254,255,27,0,
210,255,224,255,39,0,
184,255,235,255,19,0,
158,255,232,255,29,0,
129,255,170,255,221,255,
129,255,170,255,221,255,
116,255,241,255,231,255,
179,255,255,255,9,0,
153,255,187,255,240,255,
122,255,246,255,233,255,
122,255,246,255,233,255,
190,255,9,0,56,0,
230,255,248,255,125,0,
214,255,12,0,101,0,
230,255,35,0,31,0,
117,0,46,0,89,255,
117,0,46,0,89,255,
80,0,235,255,108,255,
54,0,237,255,70,255,
13,0,214,255,224,254,
228,255,222,255,151,254,
228,255,222,255,151,254,
194,255,252,255,142,254,
178,255,237,255,209,254,
204,255,163,255,234,254,
232,255,211,255,41,255,
232,255,211,255,41,255,
0,0,207,255,94,255,
252,255,124,255,237,254,
215,255,28,0,33,255,
254,255,219,255,51,255,
254,255,219,255,51,255,
243,255,157,255,47,255,
221,255,253,255,78,255,
217,255,246,255,82,255,
230,255,47,0,129,255,
21,0,248,255,173,255,
21,0,248,255,173,255,
66,0,232,255,14,0,
83,0,254,255,62,0,
81,0,232,255,14,0,
118,0,30,0,244,255,
118,0,30,0,244,255,
129,0,1,0,8,0,
146,0,38,0,54,0,
142,0,62,0,54,0,
156,0,66,0,24,0,
156,0,66,0,24,0,
161,0,137,0,23,0,
157,0,104,0,62,0,
147,0,42,0,116,0,
142,0,33,0,133,0,
117,0,20,0,95,0,
117,0,20,0,95,0,
103,0,10,0,59,0,
93,0,2,0,81,0,
91,0,212,255,99,0,
86,0,243,255,81,0,
86,0,243,255,81,0,
87,0,26,0,61,0,
70,0,3,0,60,0,
54,0,241,255,49,0,
41,0,25,0,16,0,
22,0,27,0,5,0,
22,0,27,0,5,0,
35,0,6,0,38,0,
23,0,252,255,64,0,
255,255,255,255,20,0,
232,255,15,0,199,255,
232,255,15,0,199,255,
224,255,33,0,185,255,
220,255,26,0,247,255,
191,255,245,255,3,0,
180,255,0,0,233,255,
180,255,0,0,233,255,
200,255,29,0,235,255,
174,255,3,0,6,0,
146,255,236,255,238,255,
133,255,5,0,231,255,
138,255,253,255,208,255,
138,255,253,255,208,255,
92,255,199,255,115,255,
38,255,242,255,66,255,
57,255,211,255,254,254,
54,255,180,255,234,254,
54,255,180,255,234,254,
64,255,217,255,6,255,
60,255,200,255,36,255,
111,255,177,255,232,254,
166,255,185,255,170,254,
166,255,185,255,170,254,
207,255,197,255,167,254,
4,0,206,255,255,254,
90,0,204,255,76,255,
156,0,219,255,111,255,
146,0,224,255,57,255,
146,0,224,255,57,255,
159,0,4,0,251,254,
246,0,25,0,59,255,
24,1,213,255,157,255,
194,255,11,0,186,255,
194,255,11,0,186,255,
140,255,26,0,216,255,
123,255,229,255,4,0,
106,255,188,255,70,0,
101,255,214,255,22,0,
101,255,214,255,22,0,
62,255,5,0,240,255,
58,255,4,0,46,0,
88,255,153,255,63,0,
77,255,170,255,45,0,
71,255,230,255,12,0,
71,255,230,255,12,0,
96,255,190,255,6,0,
129,255,217,255,44,0,
160,255,208,255,255,255,
177,255,204,255,226,255,
177,255,204,255,226,255,
210,255,235,255,11,0,
239,255,239,255,31,0,
237,255,225,255,251,255,
245,255,20,0,231,255,
245,255,20,0,231,255,
9,0,243,255,208,255,
33,0,231,255,213,255,
41,0,1,0,248,255,
59,0,255,255,249,255,
33,0,249,255,209,255,
33,0,249,255,209,255,
16,0,230,255,159,255,
16,0,13,0,203,255,
22,0,13,0,8,0,
13,0,226,255,3,0,
13,0,226,255,3,0,
3,0,246,255,229,255,
13,0,8,0,204,255,
9,0,246,255,241,255,
251,255,218,255,244,255,
4,0,227,255,220,255,
4,0,227,255,220,255,
12,0,7,0,235,255,
245,255,242,255,242,255,
210,255,220,255,231,255,
217,255,12,0,227,255,
217,255,12,0,227,255,
238,255,251,255,203,255,
245,255,235,255,187,255,
42,0,202,255,144,255,
90,0,207,255,111,255,
90,0,207,255,111,255,
74,0,254,255,110,255,
10,0,237,255,131,255,
25,0,214,255,171,255,
68,0,1,0,202,255,
41,0,234,255,198,255,
41,0,234,255,198,255,
248,255,195,255,204,255,
26,0,235,255,238,255,
46,0,41,0,246,255,
6,0,241,255,176,255,
6,0,241,255,176,255,
224,255,229,255,98,255,
222,255,4,0,92,255,
189,255,234,255,91,255,
162,255,219,255,76,255,
162,255,219,255,76,255,
168,255,254,255,126,255,
200,255,235,255,162,255,
166,255,233,255,144,255,
142,255,3,0,144,255,
146,255,202,255,101,255,
146,255,202,255,101,255,
169,255,19,0,133,255,
182,255,14,0,177,255,
172,255,194,255,153,255,
214,255,222,255,139,255,
214,255,222,255,139,255,
214,255,10,0,156,255,
217,255,3,0,215,255,
227,255,243,255,249,255,
247,255,5,0,232,255,
11,0,12,0,212,255,
11,0,12,0,212,255,
3,0,52,0,19,0,
30,0,35,0,58,0};
for (uint16_t i = 0; i < NO_TEST_DATA; i++) {
uint8_t lastValue = i % 3; //initialise index to head
int8_t* testDataPointer = &testData[6 * i];
if (i < 2) {
  calibrate(calibrationCoeffs, filteredValues, (int16_t *)testDataPointer, i);
  for (uint8_t j = 0; j < 3; j++) {
    for (uint8_t k = 1; k < 4; k++) {
      filteredValues[j][i][k] = filteredValues[j][i][0];
    }
  }
} else {
  calibrate(calibrationCoeffs, filteredValues, (int16_t *)testDataPointer, lastValue);
  applyFilter(filterCoeffs, filteredValues, lastValue);
  printf("%i,%i,%i\n",
filteredValues[0][lastValue][3],
filteredValues[1][lastValue][3],
filteredValues[2][lastValue][3]);
}
}
while (true) {}
#endif
//////////////////////////////////////// end test ////////////////////////////////////////

  bsp_board_leds_init();

#if APP_UART_ENABLED
  // Initialise UART

  const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200};

  APP_UART_FIFO_INIT(&comm_params,
      UART_RX_BUF_SIZE,
      UART_TX_BUF_SIZE,
      uart_error_handle,
      APP_IRQ_PRIORITY_LOWEST,
      err_code);

  APP_ERROR_CHECK(err_code);
#endif // APP_UART_ENABLED

  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

  // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
  // buttons with the use of APP_TIMER and for "read_all" ticks generation
  // (by RTC).
  lfclk_config();

#if CALIBRATE
  bsp_config();
#endif // CALIBRATE

  rtc_config();

  twi_config();

  //    initialisation of ADXL345

  APP_ERROR_CHECK(app_twi_perform(&m_app_twi, adxl345_init_transfers,
      ADXL345_INIT_TRANSFER_COUNT, NULL));

  //*************************** CONFIGURE ADXL345 **************************/
  set_range(8);
  setRegisterBit(ADXL345_INT_MAP, 6, 1);
  uart_printf("INT_map: bit6 = %d\n", getRegisterBit(ADXL345_INT_MAP, 6));

  setTapThreshold(50);     // 62.5 mg per increment
  setTapDuration(15);      // 625 µs per increment
  setDoubleTapLatency(80); // 1.25 ms per increment
  setDoubleTapWindow(200); // 1.25 ms per increment
  setRate(800);            // rate of update of data regs in Hz
  uart_printf("Double Tap Window = %d\n", getDoubleTapWindow());
  uart_printf("TapDuration = %d\n", getTapDuration());

  setActivityXYZ(0, 0, 0);  // Set to activate movement detection in the axes "setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  setActivityThreshold(75); // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  setInactivityXYZ(0, 0, 0);  // Set to detect inactivity in all the axes "setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  setInactivityThreshold(75); // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  setTimeInactivity(10);      // How many seconds of no activity is inactive?

  uart_printf("isActivityXEnabled = %d\n", isActivityXEnabled());
  uart_printf("isActivityYEnabled = %d\n", isActivityYEnabled());

  uart_printf("isInActivityXEnabled = %d\n", isInactivityXEnabled());

  uart_printf("\r\nFATFS example.\r\n\r\n");

  static uint8_t register_address;
  register_address = ADXL345_DATAX0;
  static app_twi_transfer_t const transfers[] =
      {
          ADXL345_READ(&register_address, m_buffer, 6)

      };
  static app_twi_transaction_t const transaction =
      {
          .callback = default_cb,
          .p_user_data = NULL,
          .p_transfers = transfers,
          .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])};

//*************************** CONFIGURE SD CARD **************************/
#if USE_SD
  fatfs_init();

  read_config(); // read parameters from config file on SD card

  //*************************** find number of first file **************************/
  uint16_t firstFile = 0xffff;
  while ((j < MAX_FILE_NO) && (firstFile == 0xffff)) {
    uint16_t temp = j;
    for (uint8_t k = 0; k < 4; k++) {
      filename[7 - k] = temp % 10 + '0';
      temp /= 10;
    }

    ff_result = f_open(&dataFile, filename, FA_READ);
    if (ff_result == FR_NO_FILE) {
      uart_printf("First file: %i.\r\n", j);
      firstFile = j;
    } else {
      (void)f_close(&dataFile);
    }
    ++j;
  }
  if (firstFile == 0xffff) {
    uart_printf("All files written\r\n");
    while (true) {
    }
  }
#else
  uint16_t firstFile = 0;
#endif //USE_SD
  //*************************** start measurements **************************/

  for (j = firstFile; j < MAX_FILE_NO; j++) {
    tick = false;
    timeOut = false;
    nrf_drv_rtc_tick_enable(&rtc, true);
    nrf_drv_rtc_counter_clear(&rtc);
    err_code = nrf_drv_rtc_cc_set(&rtc, 0, timeToNextMeasurement * tickFrequency, true);
    APP_ERROR_CHECK(err_code);

#if USE_SD
    uint16_t temp = j;
    for (uint8_t k = 0; k < 4; k++) {
      filename[7 - k] = temp % 10 + '0';
      temp /= 10;
    }

    ff_result = f_open(&dataFile, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK) {
      uart_printf("Unable to create file: %i.\r\n", j);
      flash_led(6, true);

      while (true) {
      };
    }
    uart_printf("Writing file: %i.\r\n", j);
#endif // USE_SD
#if CALIBRATE
    uint8_t nPositions = 0;
    while (!button1_pressed) {
      if (button0_pressed) {
        LEDS_INVERT(BSP_LED_0_MASK);
        for (uint8_t i = 0; i < 3; i++)
          xyz_int[i] = 0;
        uint32_t count = 0;

        while (count < NUMBER_OF_SAMPLES) {
          got_callback = false;
          APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
          while (!got_callback) {
          }

          for (uint8_t i = 0; i < 3; i++) {
            xyz_int[i] += (((int)m_buffer[i * 2 + 1]) << 8) | m_buffer[i * 2];
          };

          ++count;
        }
        for (uint8_t i = 0; i < 3; i++) {
          m_buffer[i * 2] = xyz_int[i] & 0x00ff;
          m_buffer[i * 2 + 1] = xyz_int[i] >> 8;
          uart_printf("xyz[%i]: %i.\r\n", i, xyz_int[i]);
        };

        ff_result = f_write(&dataFile, m_buffer, 6, (UINT *)&bytes_written);

        LEDS_INVERT(BSP_LED_0_MASK);
        button0_pressed = false;
        ++nPositions;
      }
    }
    uart_printf("Calibrated on %i positions\n\r", nPositions);
    (void)f_close(&dataFile);
    LEDS_INVERT(BSP_LED_4_MASK);
    while (true) {
    }

#else

    flash_led(2, false); // indicate about to start measuring
    uint32_t count = 0;
    uint32_t sum = 0; // sum of frequency-weighted acceleration for xyz axes 
    while (count < sizeFFT) {
      int16_t results[3];
      uint8_t lastValue = (uint8_t)(count % 3); // index on filteredValues which is a cyclic buffer
      while (!tick) {
      };
      tick = false;
      got_callback = false;
      APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
      while (!got_callback) {
      }
#if DIG_FILTER
      if (count < 2) {
        calibrate(calibrationCoeffs, filteredValues, (int16_t *)m_buffer, count);
        for (uint8_t j = 0; j < 3; j++) {
          for (uint8_t k = 1; k < 4; k++) {
            filteredValues[j][count][k] = filteredValues[j][count][0];
          }
        }
      } else {
        calibrate(calibrationCoeffs, filteredValues, (int16_t *)m_buffer, lastValue);
        applyFilter(filterCoeffs, filteredValues, lastValue);
      }
      // save results
      for (uint8_t i = 0; i < 3; i++) {
        results[i] = (int16_t)filteredValues[i][lastValue][3];
      }
#if USE_SD
      ff_result = f_write(&dataFile, results, 6, (UINT *)&bytes_written);
#endif

#else
#if USE_SD
      ff_result = f_write(&dataFile, m_buffer, 6, (UINT *)&bytes_written);
#endif // USE_SD
#endif // DIG_FILTER

      ++count;
      // calculate rms values and ahvSquared
      uint32_t sumThisInterval = 0;
      for (uint8_t i = 0; i < 3; i++) {
        sumThisInterval += (filteredValues[i][lastValue][3] * filteredValues[i][lastValue][3]);
      }
      sum += (sumThisInterval >> 8); //divide result by 256 to convert to ms-2
      // could measure max here
    }
//    flash_led(1, false); // indicate finished measuring for scope timing measurement


#if USE_SD
    (void)f_close(&dataFile); // file with high res ahv data
    // calculate contribution to daily exposure according to ISO5349
    float a8 = sqrtf((float)sum / TIME_0 / tickFrequency);
    ff_result = f_open(&a8File, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        flash_led(3, true);
    }
    file_printf(&a8File, "A8 for file %i = %f\n", j, a8);
    (void)f_close(&a8File); // file with total exposure
#endif
    flash_led(1, false); // indicate finished measuring
    nrf_drv_rtc_tick_disable(&rtc);
    // Make sure any pending events are cleared
    __SEV();
    __WFE();
    while (!timeOut) {
      __WFE();
    }

#endif // CALIBRATE
  }    // end of loop
  while (true) {
  }
} // end of main
  /** @} */