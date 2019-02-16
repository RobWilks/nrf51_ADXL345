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
//        double gains[3];
//	gains[0] = 0.00376390;
//	gains[1] = 0.00376009;
//	gains[2] = 0.00349265;

        double gains[3] = {0.00376390, 0.00376009, 0.00349265};

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


/*************************** SELF_TEST BIT **************************/
/*                            ~ GET & SET                           */
bool getSelfTestBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}

// If Set (1) Self-Test Applied. Electrostatic Force exerted on the sensor
//  causing a shift in the output data.
// If Set (0) Self-Test Disabled.
void setSelfTestBit(bool selfTestBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}

/*************************** SPI BIT STATE **************************/
/*                           ~ GET & SET                            */
bool getSpiBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}

// If Set (1) Puts Device in 3-wire Mode
// If Set (0) Puts Device in 4-wire SPI Mode
void setSpiBit(bool spiBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}

/*********************** INT_INVERT BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool getInterruptLevelBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}

// If Set (0) Sets the Interrupts to Active HIGH
// If Set (1) Sets the Interrupts to Active LOW
void setInterruptLevelBit(bool interruptLevelBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

/************************* FULL_RES BIT STATE ***********************/
/*                           ~ GET & SET                            */
bool getFullResBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}

// If Set (1) Device is in Full Resolution Mode: Output Resolution Increase with G Range
//  Set by the Range Bits to Maintain a 4mg/LSB Scale Factor
// If Set (0) Device is in 10-bit Mode: Range Bits Determine Maximum G Range
//  And Scale Factor
void setFullResBit(bool fullResBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}

/*************************** JUSTIFY BIT STATE **************************/
/*                           ~ GET & SET                            */
bool getJustifyBit() {
	return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}

// If Set (1) Selects the Left Justified Mode
// If Set (0) Selects Right Justified Mode with Sign Extension
void setJustifyBit(bool justifyBit) {
	setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

/*********************** THRESH_TAP BYTE VALUE **********************/
/*                          ~ SET & GET                             */
// Should Set Between 0 and 255
// Scale Factor is 62.5 mg/LSB
// A Value of 0 May Result in Undesirable Behavior
void setTapThreshold(uint8_t tapThreshold) {
	write_reg2(ADXL345_THRESH_TAP, tapThreshold);  
}

// Return Value Between 0 and 255
// Scale Factor is 62.5 mg/LSB
int getTapThreshold() {
	read_reg(ADXL345_THRESH_TAP);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/****************** GAIN FOR EACH AXIS IN Gs / COUNT *****************/
/*                           ~ SET & GET                            */
void setAxisGains(double_t *_gains){
	int i;
	for(i = 0; i < 3; i++){
		gains[i] = _gains[i];
	}
}
void getAxisGains(double_t *_gains){
	int i;
	for(i = 0; i < 3; i++){
		_gains[i] = gains[i];
	}
}

/********************* OFSX, OFSY and OFSZ BYTES ********************/
/*                           ~ SET & GET                            */
// OFSX, OFSY and OFSZ: User Offset Adjustments in Twos Complement Format
// Scale Factor of 15.6mg/LSB
void setAxisOffset(int8_t x, int8_t y, int8_t z) {
  write_reg2(ADXL345_OFSX, x);
  write_reg2(ADXL345_OFSY, y);
  write_reg2(ADXL345_OFSZ, z);
}

void getAxisOffset(int8_t *x, int8_t *y, int8_t *z) {
  read_reg(ADXL345_OFSX);
  while (!got_callback) {
    ;
    ;
  }
  *x = m_buffer[0];
  read_reg(ADXL345_OFSY);
  while (!got_callback) {
    ;
    ;
  }
  *y = m_buffer[0];
  read_reg(ADXL345_OFSZ);
  while (!got_callback) {
    ;
    ;
  }
  *z = m_buffer[0];
}

/****************************** DUR BYTE ****************************/
/*                           ~ SET & GET                            */
// DUR Byte: Contains an Unsigned Time Value Representing the Max Time 
//  that an Event must be Above the THRESH_TAP Threshold to qualify 
//  as a Tap Event
// The scale factor is 625µs/LSB
// Value of 0 Disables the Tap/Double Tap Funcitons. Max value is 255.
void setTapDuration(uint8_t tapDuration) {
	write_reg2(ADXL345_DUR, tapDuration);  
}

uint8_t getTapDuration() {
	read_reg(ADXL345_DUR);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/************************** LATENT REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains Unsigned Time Value Representing the Wait Time from the Detection
//  of a Tap Event to the Start of the Time Window (defined by the Window 
//  Register) during which a possible Second Tap Even can be Detected.
// Scale Factor is 1.25ms/LSB. 
// A Value of 0 Disables the Double Tap Function.
// It Accepts a Maximum Value of 255.
void setDoubleTapLatency(uint8_t doubleTapLatency) {
	write_reg2(ADXL345_LATENT, doubleTapLatency);  
}

uint8_t getDoubleTapLatency() {
	read_reg(ADXL345_LATENT);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/************************** WINDOW REGISTER *************************/
/*                           ~ SET & GET                            */
// Contains an Unsigned Time Value Representing the Amount of Time 
//  After the Expiration of the Latency Time (determined by Latent register)
//  During which a Second Valid Tape can Begin. 
// Scale Factor is 1.25ms/LSB. 
// Value of 0 Disables the Double Tap Function. 
// It Accepts a Maximum Value of 255.
void setDoubleTapWindow(uint8_t doubleTapWindow) {
	write_reg2(ADXL345_WINDOW, doubleTapWindow);  
}

uint8_t getDoubleTapWindow() {
	read_reg(ADXL345_WINDOW);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/*********************** THRESH_ACT REGISTER ************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Activity.
// Data Format is Unsigned, so the Magnitude of the Activity Event is Compared 
//  with the Value is Compared with the Value in the THRESH_ACT Register. 
// The Scale Factor is 62.5mg/LSB. 
// Value of 0 may Result in Undesirable Behavior if the Activity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void setActivityThreshold(uint8_t activityThreshold) {
	write_reg2(ADXL345_THRESH_ACT, activityThreshold);  
}

// Gets the THRESH_ACT uint8_t
int getActivityThreshold() {
	read_reg(ADXL345_THRESH_ACT);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/********************** THRESH_INACT REGISTER ***********************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value for Detecting Inactivity.
// The Data Format is Unsigned, so the Magnitude of the INactivity Event is 
//  Compared with the value in the THRESH_INACT Register. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Inactivity Interrupt Enabled. 
// It Accepts a Maximum Value of 255.
void setInactivityThreshold(uint8_t inactivityThreshold) {
	write_reg2(ADXL345_THRESH_INACT, inactivityThreshold);  
}

int getInactivityThreshold() {
	read_reg(ADXL345_THRESH_INACT);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/*********************** TIME_INACT RESIGER *************************/
/*                          ~ SET & GET                             */
// Contains an Unsigned Time Value Representing the Amount of Time that
//  Acceleration must be Less Than the Value in the THRESH_INACT Register
//  for Inactivity to be Declared. 
// Uses Filtered Output Data* unlike other Interrupt Functions
// Scale Factor is 1sec/LSB. 
// Value Must Be Between 0 and 255. 
void setTimeInactivity(uint8_t timeInactivity) {
	write_reg2(ADXL345_TIME_INACT, timeInactivity);  
}

int getTimeInactivity() {
	read_reg(ADXL345_TIME_INACT);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/*********************** THRESH_FF Register *************************/
/*                          ~ SET & GET                             */
// Holds the Threshold Value, in Unsigned Format, for Free-Fall Detection
// The Acceleration on all Axes is Compared with the Value in THRES_FF to
//  Determine if a Free-Fall Event Occurred. 
// Scale Factor is 62.5mg/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallThreshold(uint8_t freeFallThreshold) {
	write_reg2(ADXL345_THRESH_FF, freeFallThreshold);  
}

int getFreeFallThreshold() {
	read_reg(ADXL345_THRESH_FF);
while (!got_callback) {;;}  
	return m_buffer[0];
}

/************************ TIME_FF Register **************************/
/*                          ~ SET & GET                             */
// Stores an Unsigned Time Value Representing the Minimum Time that the Value 
//  of all Axes must be Less Than THRES_FF to Generate a Free-Fall Interrupt.
// Scale Factor is 5ms/LSB. 
// Value of 0 May Result in Undesirable Behavior if the Free-Fall Interrupt Enabled.
// Accepts a Maximum Value of 255.
void setFreeFallDuration(uint8_t freeFallDuration) {
		write_reg2(ADXL345_TIME_FF, freeFallDuration);  
}

int getFreeFallDuration() {
	uint8_t _b;
	read_reg(ADXL345_TIME_FF);
while (!got_callback) {;;}  
	return m_buffer[0];
}