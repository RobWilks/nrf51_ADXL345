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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_twi.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "lm75b.h"
#include "mma7660.h"
#include "ADXL345.h"
#include "compiler_abstraction.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <stdio.h>//rjw
#include <stdarg.h>//rjw
#include <__vfprintf.h>//rjw

#define DEBUG 0

#define MAX_PENDING_TRANSACTIONS    5

#define APP_TIMER_PRESCALER         0
#define APP_TIMER_OP_QUEUE_SIZE     2



#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define DATA_BUFFER_SIZE 252                         /**< data buffer for high frequency measurement. */

// added for SPI comms
#define FILE_NAME   "ADXL_DAT.TXT"
#define TEST_STRING "SD card example.\r\n"

//rjw mod for BLE400
#define SDC_SCK_PIN     SPIM0_SCK_PIN  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    SPIM0_MOSI_PIN  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    SPIM0_MISO_PIN  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      SPIM0_SS_PIN  ///< SDC chip select (CS) pin.


//rjw configure accelerometer data capture
#define SIZE_FFT 0x1000   //no of datapoints for each measurement; power of 2 
#define TIME_TO_NEXT_MEASUREMENT 30000 //in millisec
#define NO_FILES 5  //total number to measure in this sequence

//rjw configure rtc timer
#define TICK_FREQUENCY 1024 //approx 1kHz
volatile bool tick = false;
volatile uint32_t millis = 0;

char filename[] = "adxl0000.DAT";


/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

char data[50]={0}; /* Line buffer */
// end of SPI / SDC

static app_twi_t m_app_twi = APP_TWI_INSTANCE(0);


bool got_callback;
bool button0_pressed = false;
bool button1_pressed = false;

extern int16_t xyz_int[3];
extern double xyz[3];

// Buffer for data read from sensors.
#define BUFFER_SIZE  11
uint8_t m_buffer[BUFFER_SIZE];

// Buffer for high frequency accelerometer data
uint8_t data_buffer[DATA_BUFFER_SIZE];

// Data structures needed for averaging of data read from sensors.
// [max 32, otherwise "int16_t" won't be sufficient to hold the sum
//  of temperature samples]
#define NUMBER_OF_SAMPLES  16
#define CALIBRATE 0
typedef struct
{
    int16_t temp;
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;

static sum_t m_sum = { 0, 0, 0, 0 };



typedef struct
{
    // [use bit fields to fit whole structure into one 32-bit word]
    int16_t temp : 11;
    int8_t  x    : 6;
    int8_t  y    : 6;
    int8_t  z    : 6;
} sample_t;

static sample_t m_samples[NUMBER_OF_SAMPLES] = { { 0, 0, 0, 0 } };

static uint8_t m_sample_idx = 0;

// Value previously read from MMA7660's Tilt register - used to detect change
// in orientation, shake signaling etc.
static uint8_t m_prev_tilt = 0;

#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif


////////////////////////////////////////////////////////////////////////////////
// Reading of data from sensors - current temperature from LM75B and from
// MMA7660: X, Y, Z and tilt status.
#if (BUFFER_SIZE < 6)
    #error Buffer too small.
#endif
#define GET_ACC_VALUE(axis, reg_data) \
    do { \
        if (MMA7660_DATA_IS_VALID(reg_data)) \
        { \
            axis = MMA7660_GET_ACC(reg_data); \
        } \
    } while (0)
    /*
void read_all_cb(ret_code_t result, void * p_user_data)
{
    if (result != NRF_SUCCESS)
    {
        uart_printf("read_all_cb - error: %d\r\n", (int)result);
        return;
    }

    sample_t * p_sample = &m_samples[m_sample_idx];
    m_sum.temp -= p_sample->temp;
    m_sum.x    -= p_sample->x;
    m_sum.y    -= p_sample->y;
    m_sum.z    -= p_sample->z;

    uint8_t temp_hi = m_buffer[0];
    uint8_t temp_lo = m_buffer[1];
    uint8_t x_out   = m_buffer[2];
    uint8_t y_out   = m_buffer[3];
    uint8_t z_out   = m_buffer[4];
    uint8_t tilt    = m_buffer[5];

    p_sample->temp = LM75B_GET_TEMPERATURE_VALUE(temp_hi, temp_lo);
    GET_ACC_VALUE(p_sample->x, x_out);
    GET_ACC_VALUE(p_sample->y, y_out);
    GET_ACC_VALUE(p_sample->z, z_out);
    if (!MMA7660_DATA_IS_VALID(tilt))
    {
        tilt = m_prev_tilt;
    }

    m_sum.temp += p_sample->temp;
    m_sum.x    += p_sample->x;
    m_sum.y    += p_sample->y;
    m_sum.z    += p_sample->z;

    ++m_sample_idx;
    if (m_sample_idx >= NUMBER_OF_SAMPLES)
    {
        m_sample_idx = 0;
    }

    // Show current average values every time sample index rolls over (for RTC
    // ticking at 32 Hz and 16 samples it will be every 500 ms) or when tilt
    // status changes.
    if (m_sample_idx == 0 || (m_prev_tilt && m_prev_tilt != tilt))
    {
        char const * orientation;
        switch (MMA7660_GET_ORIENTATION(tilt))
        {
            case MMA7660_ORIENTATION_LEFT:  orientation = "LEFT";  break;
            case MMA7660_ORIENTATION_RIGHT: orientation = "RIGHT"; break;
            case MMA7660_ORIENTATION_DOWN:  orientation = "DOWN";  break;
            case MMA7660_ORIENTATION_UP:    orientation = "UP";    break;
            default:                        orientation = "?";     break;
        }

        uart_printf("Temp: " NRF_LOG_FLOAT_MARKER " | X: %3d, Y: %3d, Z: %3d ",
            NRF_LOG_FLOAT((float)((m_sum.temp * 0.125) / NUMBER_OF_SAMPLES)),
            m_sum.x / NUMBER_OF_SAMPLES,
            m_sum.y / NUMBER_OF_SAMPLES,
            m_sum.z / NUMBER_OF_SAMPLES);

        NRF_LOG_RAW_INFO("| %s%s%s\r\n",
            (uint32_t)orientation,
            (uint32_t)(MMA7660_TAP_DETECTED(tilt)   ? " TAP"   : ""),
            (uint32_t)(MMA7660_SHAKE_DETECTED(tilt) ? " SHAKE" : ""));
        m_prev_tilt = tilt;
    }
}
*/
////////////////////////////////////// uart_printf //////////////////////////////////////////

void uart_printf(const char *fmt, ...)
{
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


void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
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
/////////////////////////////////////// write_reg call back /////////////////////////////////////////
// write_reg call back function

void write_reg_cb(ret_code_t result, void *p_user_data) {
  if (result != NRF_SUCCESS) {
    uart_printf("write_reg_cb - error: %d\r\n", (int)result);
    return;
  }

  uart_printf("data written\n");
}

////////////////////////////////////// write_reg //////////////////////////////////////////
// value to write is in config
// uses twi_schedule
void write_reg(uint8_t address, uint8_t val) {

  static uint8_t config[2];
  config[0] = address;
  config[1] = val;

  static app_twi_transfer_t const transfers[] =
      {
          APP_TWI_WRITE(ADXL345_DEVICE, config, sizeof(config), 0),

      };
  static app_twi_transaction_t const transaction =
      {
          .callback = write_reg_cb,
          .p_user_data = NULL,
          .p_transfers = transfers,
          .number_of_transfers = sizeof(transfers) / sizeof(transfers[0])};

  APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
}

#if (BUFFER_SIZE < 7)
#error Buffer too small.
#endif


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
//
static void bsp_event_handler(bsp_event_t event)
{
    // Each time the button 1 or 4 is pushed we start a transaction reading
    // values of all registers from LM75B or MMA7660 respectively.
    switch (event)
    {
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
static void bsp_config(void)
{
    uint32_t err_code;

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    err_code = bsp_init(BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////////////////////////////////////////////
// TWI (with transaction manager) initialization.
static void twi_config(void)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

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
//  if (int_type == NRF_DRV_RTC_INT_COMPARE0) {
//    timeOut = true;
//  } 
   if (int_type == NRF_DRV_RTC_INT_TICK) {
    tick = true;
    ++millis;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize RTC instance with default configuration.
static void rtc_config(void)
{
    uint32_t err_code;

    
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = RTC_FREQ_TO_PRESCALER(TICK_FREQUENCY); //Set RTC frequency to 32Hz
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    // Power on RTC instance.
    nrf_drv_rtc_enable(&rtc);
}


static void lfclk_config(void)
{
    uint32_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}
//////////////////////////////////////flash_led()//////////////////////////////////////////

static void flash_led(uint8_t noTimes)
{

      for (uint8_t j = 0; j < noTimes; j++)
      {
      LEDS_INVERT(BSP_LED_0_MASK);
      nrf_delay_ms(200);
      LEDS_INVERT(BSP_LED_0_MASK);
      nrf_delay_ms(200);
      }
}
//////////////////////////////////////fatfs_example()//////////////////////////////////////////

/**
 * @brief Function for demonstrating FAFTS usage.
 */
static void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;

    uint32_t bytes_written;
    uint32_t bytes_read;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    uart_printf("Initializing disk 0 (SDC)...\r\n");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        uart_printf("Disk initialization failed.\r\n");
        return;
    }
    
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    uart_printf("Capacity: %d MB\r\n", capacity);

    uart_printf("Mounting volume...\r\n");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        uart_printf("Mount failed.\r\n");
        return;
    }

    // omit directory listing and dummy read/write
/*
    uart_printf("\r\n Listing directory: /\r\n");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        uart_printf("Directory listing failed!\r\n");
        return;
    }
    
    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            uart_printf("Directory read failed.");
            return;
        }
        
        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s\r\n",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s\r\n", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("\r\n");
    
    uart_printf("Writing to file " FILE_NAME "...\r\n");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        uart_printf("Unable to open or create file: " FILE_NAME ".\r\n");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        uart_printf("Write failed\r\n.");
    }
    else
    {
        uart_printf("%d bytes written.\r\n", bytes_written);
    }

    (void) f_close(&file);
*/


/*
// rjw addition

//read content of the file
ff_result = f_open(&file, FILE_NAME, FA_READ);
if (ff_result != FR_OK)
{
    uart_printf("Unable to open or create file: " FILE_NAME ".\r\n");
    return;
}

uint16_t size = f_size(&file);
uart_printf("size of the file in bytes = %d\r\n", size);

bytes_read = 0;

ff_result = f_read(&file, data, sizeof(data), (UINT *)&bytes_read);

if (ff_result != FR_OK) {
  uart_printf("Unable to read or create file: " FILE_NAME ".\r\n");
  return;
} else {
  uart_printf("%d bytes read\r\n", bytes_read);
}
(void) f_close(&file);
*/
return;
}
////////////////////////////////////////////////////////////////////////////////

int main(void)
{
    uint32_t err_code; // added for UART
    uint32_t bytes_written;
    FRESULT ff_result;
      static FIL dataFile;

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
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
    #endif // APP_UART_ENABLED


    // Start internal LFCLK XTAL oscillator - it is needed by BSP to handle
    // buttons with the use of APP_TIMER and for "read_all" ticks generation
    // (by RTC).
    lfclk_config();

    bsp_config();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

    uart_printf("TWI master example\r\n");
    twi_config();


//    rjw initialisation of ADXL345 

    APP_ERROR_CHECK(app_twi_perform(&m_app_twi, adxl345_init_transfers,
        ADXL345_INIT_TRANSFER_COUNT, NULL));

 
   //*************************** CONFIGURE ADXL345 **************************/
    set_range(8);
    setRegisterBit(ADXL345_INT_MAP, 6, 1);
    uart_printf("INT_map: bit6 = %d\n", getRegisterBit(ADXL345_INT_MAP, 6));

    setTapThreshold(50);     // 62.5 mg per increment
    setTapDuration(15);      // 625 �s per increment
    setDoubleTapLatency(80); // 1.25 ms per increment
    setDoubleTapWindow(200); // 1.25 ms per increment
    setRate(800); // rate of update of data regs in Hz
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

   //*************************** CONFIGURE SD CARD **************************/
    fatfs_example();



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


  //*************************** find number first file **************************/
  
    uint16_t j = 0;
    uint16_t firstFile = 0xffff;
    while ((j < NO_FILES) && (firstFile == 0xffff)) {
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
      if (firstFile == 0xffff)
      {
    uart_printf("All files written\r\n");
    while (true) {
    }
      }

  //*************************** start measurements **************************/

    flash_led(7); // indicate ready to start
    for (j = firstFile; j < firstFile + NO_FILES; j++) {
      uint16_t temp = j;
      for (uint8_t k = 0; k < 4; k++) {
        filename[7 - k] = temp % 10 + '0';
        temp /= 10;
      }

      ff_result = f_open(&dataFile, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
      if (ff_result != FR_OK) {
        uart_printf("Unable to create file: %i.\r\n", j);
        flash_led(3); 

        while (true) {
        };
      }
      uart_printf("Writing file: %i.\r\n", j);
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
      
      LEDS_INVERT(BSP_LED_0_MASK);
    uint32_t count = 0;
  //*************************** CONFIGURE RTC **************************/
    rtc_config();
//    while (millis < TIME_OUT)
//    {
//    while (!tick) {;;};
//    if (!(millis & 0xff)) LEDS_INVERT(BSP_LED_0_MASK);
//    tick = false;
//    }
//      (void)f_close(&dataFile);
//      while(true) {;;}

    while ((count < SIZE_FFT)) {
      while (!tick) {
        ;
        ;
      };
      tick = false;
      got_callback = false;
      APP_ERROR_CHECK(app_twi_schedule(&m_app_twi, &transaction));
      while (!got_callback) {
      }

      ff_result = f_write(&dataFile, m_buffer, 6, (UINT *)&bytes_written);

      ++count;
    }
    (void)f_close(&dataFile);
    LEDS_INVERT(BSP_LED_0_MASK);
#endif
      nrf_delay_ms(TIME_TO_NEXT_MEASUREMENT);
    }
    while (true) {
    }
}
/** @} */