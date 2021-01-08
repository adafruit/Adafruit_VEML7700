/*!
 *  @file Adafruit_VEML7700.cpp
 *
 *  @mainpage Adafruit VEML7700 I2C Lux Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the VEML7700 I2C Lux sensor
 *
 * 	This is a library for the Adafruit VEML7700 breakout:
 * 	http://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried (Adafruit Industries)
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_VEML7700.h"

/*!
 *    @brief  Instantiates a new VEML7700 class
 */
// Adafruit_VEML7700::Adafruit_VEML7700(void) {}

/*!
 *    @brief  Setups the hardware for talking to the VEML7700
 *    @param  theWire An optional pointer to an I2C interface
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_VEML7700::begin(TwoWire *theWire) {
  i2c_dev = new Adafruit_I2CDevice(VEML7700_I2CADDR_DEFAULT, theWire);

  if (!i2c_dev->begin()) {
    return false;
  }

  ALS_Config =
      new Adafruit_I2CRegister(i2c_dev, VEML7700_ALS_CONFIG, 2, LSBFIRST);
  ALS_HighThreshold = new Adafruit_I2CRegister(
      i2c_dev, VEML7700_ALS_THREHOLD_HIGH, 2, LSBFIRST);
  ALS_LowThreshold =
      new Adafruit_I2CRegister(i2c_dev, VEML7700_ALS_THREHOLD_LOW, 2, LSBFIRST);
  Power_Saving =
      new Adafruit_I2CRegister(i2c_dev, VEML7700_ALS_POWER_SAVE, 2, LSBFIRST);
  ALS_Data = new Adafruit_I2CRegister(i2c_dev, VEML7700_ALS_DATA, 2, LSBFIRST);
  White_Data =
      new Adafruit_I2CRegister(i2c_dev, VEML7700_WHITE_DATA, 2, LSBFIRST);
  Interrupt_Status =
      new Adafruit_I2CRegister(i2c_dev, VEML7700_INTERRUPTSTATUS, 2, LSBFIRST);

  ALS_Shutdown =
      new Adafruit_I2CRegisterBits(ALS_Config, 1, 0); // # bits, bit_shift
  ALS_Interrupt_Enable = new Adafruit_I2CRegisterBits(ALS_Config, 1, 1);
  ALS_Persistence = new Adafruit_I2CRegisterBits(ALS_Config, 2, 4);
  ALS_Integration_Time = new Adafruit_I2CRegisterBits(ALS_Config, 4, 6);
  ALS_Gain = new Adafruit_I2CRegisterBits(ALS_Config, 2, 11);
  PowerSave_Enable = new Adafruit_I2CRegisterBits(Power_Saving, 1, 0);
  PowerSave_Mode = new Adafruit_I2CRegisterBits(Power_Saving, 2, 1);

  enable(false);
  interruptEnable(false);
  setPersistence(VEML7700_PERS_1);
  setGain(VEML7700_GAIN_1);
  setIntegrationTime(VEML7700_IT_100MS);
  powerSaveEnable(false);
  enable(true);

  return true;
}

float Adafruit_VEML7700::normalize_resolution(float value) {
  // adjust for gain (1x is normalized)
  switch (getGain()) {
  case VEML7700_GAIN_2:
    value /= 2.0;
    break;
  case VEML7700_GAIN_1_4:
    value *= 4;
    break;
  case VEML7700_GAIN_1_8:
    value *= 8;
    break;
  }

  // adjust for integrationtime (100ms is normalized)
  switch (getIntegrationTime()) {
  case VEML7700_IT_25MS:
    value *= 4;
    break;
  case VEML7700_IT_50MS:
    value *= 2;
    break;
  case VEML7700_IT_200MS:
    value /= 2.0;
    break;
  case VEML7700_IT_400MS:
    value /= 4.0;
    break;
  case VEML7700_IT_800MS:
    value /= 8.0;
    break;
  }

  return value;
}

/*!
 *    @brief Read the calibrated lux value. See app note lux table on page 5
 *    @returns Floating point Lux data (ALS multiplied by 0.0576)
 */
float Adafruit_VEML7700::readLux() {
  return (normalize_resolution(ALS_Data->read()) *
          0.0576); // see app note lux table on page 5
}

/*!
 *    @brief Read the lux value with correction for non-linearity at high-lux
 * settings
 *    @returns Floating point Lux data (ALS multiplied by 0.0576 and corrected
 * for high-lux settings)
 */
float Adafruit_VEML7700::readLuxNormalized() {
  float lux = readLux();

  // user-provided correction for non-linearities at high lux/white values:
  // https://forums.adafruit.com/viewtopic.php?f=19&t=152997&p=758582#p759346
  if ((getGain() == VEML7700_GAIN_1_8) &&
      (getIntegrationTime() == VEML7700_IT_25MS)) {
    lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) +
          8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
  }

  return lux;
}

/*!
 *    @brief Read the raw ALS data
 *    @returns 16-bit data value from the ALS register
 */
uint16_t Adafruit_VEML7700::readALS() { return ALS_Data->read(); }

/*!
 *    @brief Read the white light data
 *    @returns Floating point 'white light' data multiplied by 0.0576
 */
float Adafruit_VEML7700::readWhite() {
  // white_corrected= 2E-15*pow(VEML_white,4) + 4E-12*pow(VEML_white,3) +
  // 9E-06*pow(VEML_white,)2 + 1.0179*VEML_white - 11.052;
  return normalize_resolution(White_Data->read()) *
         0.0576; // Unclear if this is the right multiplier
}

/*!
 *    @brief Read the 'white light' value with correction for non-linearity at
 * high-lux settings
 *    @returns Floating point 'white light' data multiplied by 0.0576 and
 * corrected for high-lux settings
 */
float Adafruit_VEML7700::readWhiteNormalized() {
  float white = readWhite();

  // user-provided correction for non-linearities at high lux values:
  // https://forums.adafruit.com/viewtopic.php?f=19&t=152997&p=758582#p759346
  if ((getGain() == VEML7700_GAIN_1_8) &&
      (getIntegrationTime() == VEML7700_IT_25MS)) {
    white = 2E-15 * pow(white, 4) + 4E-12 * pow(white, 3) +
            9E-06 * pow(white, 2) + 1.0179 * white - 11.052;
  }

  return white;
}

/*!
 *    @brief Enable or disable the sensor
 *    @param enable The flag to enable/disable
 */
void Adafruit_VEML7700::enable(bool enable) { ALS_Shutdown->write(!enable); }

/*!
 *    @brief Ask if the interrupt is enabled
 *    @returns True if enabled, false otherwise
 */
bool Adafruit_VEML7700::enabled(void) { return !ALS_Shutdown->read(); }

/*!
 *    @brief Enable or disable the interrupt
 *    @param enable The flag to enable/disable
 */
void Adafruit_VEML7700::interruptEnable(bool enable) {
  ALS_Interrupt_Enable->write(enable);
}

/*!
 *    @brief Ask if the interrupt is enabled
 *    @returns True if enabled, false otherwise
 */
bool Adafruit_VEML7700::interruptEnabled(void) {
  return ALS_Interrupt_Enable->read();
}

/*!
 *    @brief Set the ALS IRQ persistance setting
 *    @param pers Persistance constant, can be VEML7700_PERS_1, VEML7700_PERS_2,
 *    VEML7700_PERS_4 or VEML7700_PERS_8
 */
void Adafruit_VEML7700::setPersistence(uint8_t pers) {
  ALS_Persistence->write(pers);
}

/*!
 *    @brief Get the ALS IRQ persistance setting
 *    @returns Persistance constant, can be VEML7700_PERS_1, VEML7700_PERS_2,
 *    VEML7700_PERS_4 or VEML7700_PERS_8
 */
uint8_t Adafruit_VEML7700::getPersistence(void) {
  return ALS_Persistence->read();
}

/*!
 *    @brief Set ALS integration time
 *    @param it Can be VEML7700_IT_100MS, VEML7700_IT_200MS, VEML7700_IT_400MS,
 *    VEML7700_IT_800MS, VEML7700_IT_50MS or VEML7700_IT_25MS
 */
void Adafruit_VEML7700::setIntegrationTime(uint8_t it) {
  ALS_Integration_Time->write(it);
}

/*!
 *    @brief Get ALS integration time
 *    @returns IT index, can be VEML7700_IT_100MS, VEML7700_IT_200MS,
 * VEML7700_IT_400MS, VEML7700_IT_800MS, VEML7700_IT_50MS or VEML7700_IT_25MS
 */
uint8_t Adafruit_VEML7700::getIntegrationTime(void) {
  return ALS_Integration_Time->read();
}

/*!
 *    @brief Set ALS gain
 *    @param gain Can be VEML7700_GAIN_1, VEML7700_GAIN_2, VEML7700_GAIN_1_8 or
 * VEML7700_GAIN_1_4
 */
void Adafruit_VEML7700::setGain(uint8_t gain) { ALS_Gain->write(gain); }

/*!
 *    @brief Get ALS gain
 *    @returns Gain index, can be VEML7700_GAIN_1, VEML7700_GAIN_2,
 * VEML7700_GAIN_1_8 or VEML7700_GAIN_1_4
 */
uint8_t Adafruit_VEML7700::getGain(void) { return ALS_Gain->read(); }

/*!
 *    @brief Enable power save mode
 *    @param enable True if power save should be enabled
 */
void Adafruit_VEML7700::powerSaveEnable(bool enable) {
  PowerSave_Enable->write(enable);
}

/*!
 *    @brief Check if power save mode is enabled
 *    @returns True if power save is enabled
 */
bool Adafruit_VEML7700::powerSaveEnabled(void) {
  return PowerSave_Enable->read();
}

/*!
 *    @brief Assign the power save register data
 *    @param mode The 16-bit data to write to VEML7700_ALS_POWER_SAVE
 */
void Adafruit_VEML7700::setPowerSaveMode(uint8_t mode) {
  PowerSave_Mode->write(mode);
}

/*!
 *    @brief  Retrieve the power save register data
 *    @return 16-bit data from VEML7700_ALS_POWER_SAVE
 */
uint8_t Adafruit_VEML7700::getPowerSaveMode(void) {
  return PowerSave_Mode->read();
}

/*!
 *    @brief Assign the low threshold register data
 *    @param value The 16-bit data to write to VEML7700_ALS_THREHOLD_LOW
 */
void Adafruit_VEML7700::setLowThreshold(uint16_t value) {
  ALS_LowThreshold->write(value);
}

/*!
 *    @brief  Retrieve the low threshold register data
 *    @return 16-bit data from VEML7700_ALS_THREHOLD_LOW
 */
uint16_t Adafruit_VEML7700::getLowThreshold(void) {
  return ALS_LowThreshold->read();
}

/*!
 *    @brief Assign the high threshold register data
 *    @param value The 16-bit data to write to VEML7700_ALS_THREHOLD_HIGH
 */
void Adafruit_VEML7700::setHighThreshold(uint16_t value) {
  ALS_HighThreshold->write(value);
}

/*!
 *    @brief  Retrieve the high threshold register data
 *    @return 16-bit data from VEML7700_ALS_THREHOLD_HIGH
 */
uint16_t Adafruit_VEML7700::getHighThreshold(void) {
  return ALS_HighThreshold->read();
}

/*!
 *    @brief  Retrieve the interrupt status register data
 *    @return 16-bit data from VEML7700_INTERRUPTSTATUS
 */
uint16_t Adafruit_VEML7700::interruptStatus(void) {
  return Interrupt_Status->read();
}

/*!
 *    @brief  Measure the Lux value (with multiple measurement settings for best result, might take some time)
 *    @return Lux value, negative for error state
 */
float Adafruit_VEML7700::luxAutoSensor() {
  // Music to develope "Yello ~ Lost Again - Extended 12"
  const byte computetime = 20; // veml7700 needs "integrationtime+computetime" to calculate result

  int8_t integrationtime = 0;  // "middle" integration time
  int8_t gain = 1;             // lowest gain
  bool first = true;

  // first try: low light environement
  do {
    setIntegrationTime(ChartIT(integrationtime));
    setGain(ChartGain(gain));
    // trigger new measurement
    enable(false);
    enable(true);
    delay(IntegrationTime(integrationtime) + computetime); // +20 for extra processing else sensor gives no valid result
    uint16_t als = ALS_Data->read();

    if (als <= 100) {
      first = false;
      if (gain < 4) {
        gain++;
      } else if (integrationtime < 3) {
        integrationtime++;
      } else {
        // reached max gain and max integrationtime
        //Serial.println("Return 1");
        return als * ChartResolution(integrationtime, gain);
      }
    } else {
      if (first) {
        //Serial.println("Break 1");
        break; //yeah
      } else {
        //Serial.println("Return 2");
        return als * ChartResolution(integrationtime, gain);
      }
    }
  } while (1);

  // we end here when the first read shows count > 100
  integrationtime--;
  do {
    setIntegrationTime(ChartIT(integrationtime));
    // trigger new measurement
    enable(false);
    enable(true);
    delay(IntegrationTime(integrationtime) + computetime); // +20 for extra processing else sensor gives no valid result
    uint16_t als = ALS_Data->read();

    if (als < 10000) {
      //Serial.println("Return 3");
      return als * (1.0023 + als * (0.000081488 + als * (-9.3924e-9 + 6.0135e-13 * als)));
    } else {
      integrationtime--;
      if (integrationtime <= -3) {
        // Ambient light really ≥ 200 klx?
        return -2.0;   // error
      }
    }
  } while (1);

  return -1.0; // Huh? Error.
}

// helper 1: page 21, Table "Gain selection"
uint8_t Adafruit_VEML7700::ChartGain(const int8_t G) {
    switch (G) {
        case 1 : return VEML7700_GAIN_1_8;
        case 2 : return VEML7700_GAIN_1_4;
        case 3 : return VEML7700_GAIN_1;
        case 4 :
        default : return VEML7700_GAIN_2;
    }
}

// helper 2: page 21, Table "ALS integration time setting"
uint16_t Adafruit_VEML7700::ChartIT(const int8_t IT) {
    switch (IT) {
        case -2 : return VEML7700_IT_25MS;
        case -1 : return VEML7700_IT_50MS;
        case  0 : return VEML7700_IT_100MS;
        case  1 : return VEML7700_IT_200MS;
        case  2 : return VEML7700_IT_400MS;
        case  3 : 
        default : return VEML7700_IT_800MS;
    }
}

// helper 3: page 5, Table RESOLUTION AND MAXIMUM DETECTION RANGE
uint16_t Adafruit_VEML7700::IntegrationTime(const int8_t IT) {
  switch (ChartIT(IT)) {
    case VEML7700_IT_800MS:
      return 800;
    case VEML7700_IT_400MS:
      return 400;
    case VEML7700_IT_200MS:
      return 200;
    case VEML7700_IT_100MS:
      return 100;
    case VEML7700_IT_50MS:
      return 50;
    case VEML7700_IT_25MS:
      return 25;
  }
}

// helper 3: page 5, Table RESOLUTION AND MAXIMUM DETECTION RANGE
float Adafruit_VEML7700::ChartResolution(const int8_t IT, const int8_t G) {
  float base = 0.0036;

  switch (ChartGain(G)) {
    /*
      case VEML7700_GAIN_2 :
      base *= 1;
      break;
    */
    case VEML7700_GAIN_1 :
      base *= 2;
      break;
    case VEML7700_GAIN_1_4 :
      base *= 8;
      break;
    case VEML7700_GAIN_1_8 :
      base *= 16;
      break;
  }

  switch (ChartIT(IT)) {
    /*
    case VEML7700_IT_800MS:
      base *= 1;
      break;
    */  
    case VEML7700_IT_400MS:
      base *= 2;
      break;
    case VEML7700_IT_200MS:
      base *= 4;
      break;
    case VEML7700_IT_100MS:
      base *= 8;
      break;
    case VEML7700_IT_50MS:
      base *= 16;
      break;
    case VEML7700_IT_25MS:
      base *= 32;
      break;
  }

  return base;
}

