/*
Sparkfun's ADXL345 Library Main Source File
SparkFun_ADXL345.cpp

E.Robert @ SparkFun Electronics
Created: Jul 13, 2016
Updated: Sep 06, 2016

Modified Bildr ADXL345 Source File @ http://code.bildr.org/download/959.zip
to support both I2C and SPI Communication

Hardware Resources:
- Arduino Development Board
- SparkFun Triple Access Accelerometer ADXL345

Development Environment Specifics:
Arduino 1.6.8
SparkFun Triple Axis Accelerometer Breakout - ADXL345
Arduino Uno
*/

#include "ADXL345.h"

#include "app_twi.h"

#include "stdint.h"

#include "stdio.h"

#include "math.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t const adxl345_xout_reg_addr = ADXL345_DATAX0;
uint8_t const adxl345_data_format_reg_addr = ADXL345_DATA_FORMAT;
uint8_t const adxl345_bw_rate_reg_addr = ADXL345_BW_RATE;
uint8_t reg_addr;

// Power on.
static uint8_t const default_config[] = {ADXL345_POWER_CTL, 0};   // Wakeup
static uint8_t const default_config1[] = {ADXL345_POWER_CTL, 16}; // Auto_Sleep
static uint8_t const default_config2[] = {ADXL345_POWER_CTL, 8};  // Measure

app_twi_transfer_t const adxl345_init_transfers[ADXL345_INIT_TRANSFER_COUNT] =
    {
        APP_TWI_WRITE(ADXL345_DEVICE, default_config, sizeof(default_config), 0),
        APP_TWI_WRITE(ADXL345_DEVICE, default_config1, sizeof(default_config1), 0),
        APP_TWI_WRITE(ADXL345_DEVICE, default_config2, sizeof(default_config2), 0)
};


////////////////////////////////////// setRegisterBit //////////////////////////////////////////
void setRegisterBit(uint8_t regAdress, uint8_t bitPos, bool state) {
  read_reg(regAdress);
  if (state) {
    m_buffer[0] |= (1 << bitPos); // Forces nth Bit of _b to 1. Other Bits Unchanged.
  } else {
    m_buffer[0] &= ~(1 << bitPos); // Forces nth Bit of _b to 0. Other Bits Unchanged.
  }
  write_reg2(regAdress, m_buffer[0]);
}
////////////////////////////////////// getRegisterBit //////////////////////////////////////////

bool getRegisterBit(uint8_t regAdress, uint8_t bitPos) {
  read_reg(regAdress);
  return ((m_buffer[0] >> bitPos) & 1);
}

/*************************** SET RANGE **************************/
/*          ACCEPTABLE VALUES: 2g, 4g, 8g, 16g ~ GET & SET          */
void set_range(uint8_t val) {
  uint8_t _s;
  read_reg(ADXL345_DATA_FORMAT);
  while (!got_callback) {
    ;
    ;
  }

  switch (val) {
  case 2:
    _s = 0b00000000;
    break;
  case 4:
    _s = 0b00000001;
    break;
  case 8:
    _s = 0b00000010;
    break;
  case 16:
    _s = 0b00000011;
    break;
  default:
    _s = 0b00000000;
  }

  _s |= (m_buffer[0] & 0b11101100);

  write_reg2(ADXL345_DATA_FORMAT, _s);
}