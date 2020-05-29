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
 *     v1.1.1  - Commited on 2020-05-26
 *
 *  @section  Changes
 *
 *    2020-05-27  MojaveTom:
 *      Add utility functions to optimize VEML_7700 parameters for a given light
 * level. Refactor polynomial equations to avoid using the pow() function. Add
 * function to return the refresh time as per the App Note:
 *        https://www.vishay.com/docs/84323/designingveml7700.pdf
 *      Add function `convertToLux` that accepts the raw ALS reading and returns
 *        the Lux as a float.  The same raw ALS value can be used to optimize
 *        the parameters; thus avoiding multiple reads of the sensor.
 *      The datasheet for the sensor is:
 *        https://www.vishay.com/docs/84286/veml7700.pdf
 *      The datasheet for the Adafruit breakout board is:
 *        https://www.adafruit.com/product/4162
 */

#include "Adafruit_VEML7700.h"

#include <Wire.h>

#include "Arduino.h"

/*!
 *    @brief  Instantiates a new VEML7700 class
 *    @param  _lowThreshold   (default 1000) If the value of ALSread() {the raw
 * reading} is below this, VEML parameters will be adjusted to raise the raw
 * reading.
 *    @param  _highThreshold  (default 30000) If the value of ALSread() {the raw
 * reading} is above this, VEML parameters will be adjusted to lower the raw
 * reading.
 *    @note   If the low and high trhesholds are too "tight", parameter
 * adjustments are likely to never converge, leading to possible infinite loops.
 */
Adafruit_VEML7700::Adafruit_VEML7700(uint16_t _lowThreshold,
                                     uint16_t _highThreshold)
    : lowThresh(_lowThreshold), hiThresh(_highThreshold) {}

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

/*!
 *    @brief Normalize the lux value. See app note lux table on page 5
 *    @returns Floating point Lux data (ALS multiplied by 0.0576)
 */
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

// gain mult coeffs for VEML7700_GAIN_1=0, VEML7700_GAIN_2=1,
// VEML7700_GAIN_1_8=2, VEML7700_GAIN_1_4=3
static const double gainCoeff[] = {1.0, 0.5, 8.0, 4.0};
/* integration coeffs for  VEML7700_IT_100MS=0, VEML7700_IT_200MS=1,
  VEML7700_IT_400MS=2, VEML7700_IT_800MS=3 unused 4,5,6,7; VEML7700_IT_50MS=8,
  unused 9,10,11; VEML7700_IT_25MS=12, unused 13,14,15  */
static const double integrationTimeCoeff[] = {1.0, 0.5, 0.25, 0.125, 1.0, 1.0,
                                              1.0, 1.0, 2.0,  1.0,   1.0, 1.0,
                                              4.0, 1.0, 1.0,  1.0};
static const uint8_t gainIndex[] = {
    2, 3, 0, 1}; // converts gain from getGain() to gainVals index
static const uint8_t integrationTimeIndex[] = {
    2, 3, 4, 5, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0}; // converts from getInte...() to integrationTimeVals
                          // index

/*!
 *    @brief Convert the 7700 ALS value to lux. See app note lux table on page 5
 *    @param value  The raw ALS value from readALS().
 *    @returns Floating point Lux data corrected for gain, integration time, and
 * non-linearity.
 */
float Adafruit_VEML7700::convertToLux(uint16_t value) {
  uint8_t g = getGain();
  uint8_t i = getIntegrationTime();
  // see app note lux table on page 5
  double ret = value * gainCoeff[g] * integrationTimeCoeff[i] * 0.0576;
  if ((g == VEML7700_GAIN_1_8) && (i == VEML7700_IT_25MS))
    ret = (((6.0135e-13 * ret - 9.3924e-9) * ret + 8.1488e-5) * ret + 1.0023) *
          ret;
  return float(ret);
}

/*!
 *    @brief Adjust gain and integration time to optimize resolution. See app
 * note flow chart on page 21
 *    @param  raw  The raw ALS value from readALS().
 *    @returns Boolean status; true if any parameters changed.
 */
bool Adafruit_VEML7700::optimizeParams(uint16_t raw) {
  static const uint8_t gainVals[] = {VEML7700_GAIN_1_8, VEML7700_GAIN_1_4,
                                     VEML7700_GAIN_1, VEML7700_GAIN_2};
  static const uint8_t gainValsMaxIndex =
      sizeof(gainVals) / sizeof(gainVals[0]) - 1;
  static const uint8_t integrationTimeVals[] = {
      VEML7700_IT_25MS,  VEML7700_IT_50MS,  VEML7700_IT_100MS,
      VEML7700_IT_200MS, VEML7700_IT_400MS, VEML7700_IT_800MS};
  static const uint8_t integrationValsMaxIndex =
      sizeof(integrationTimeVals) / sizeof(integrationTimeVals[0]) - 1;

  uint8_t gIndex = gainIndex[getGain()];
  uint8_t iIndex = integrationTimeIndex[getIntegrationTime()];

  bool somethingChanged = false;

  if (raw <= lowThresh) { // We have a small number for the raw ALS value.
    // increase sensitivity
    if (gIndex < gainValsMaxIndex) {
      gIndex++;
      somethingChanged = true;
    } else {
      if (iIndex < integrationValsMaxIndex) {
        iIndex++;
        somethingChanged = true;
      }
    }
  } else if (raw > hiThresh) { // We have a high number for raw ALS value.
    // decrease sensitivity
    if (iIndex > 2) { // try to stick with VEML7700_IT_100MS
      iIndex--;       // reduce integration time till we get to 100ms
      somethingChanged = true;
    } else {
      if (gIndex > 0) {
        gIndex--; // reduce gain if possible
        somethingChanged = true;
      } else { // (not possible to reduce gain)
        if (iIndex > 0) {
          iIndex--; // reduce integration time anyway
          somethingChanged = true;
        }
      }
    }
  }

  if (somethingChanged) {
    bool wasEnabled = enabled();
    enable(false);
    setIntegrationTime(integrationTimeVals[iIndex]);
    setGain(gainVals[gIndex]);
    enable(wasEnabled);
  }
  return somethingChanged;
}

/*!
 *    @brief Compute the refresh time.  See App note page 16.
 *    @returns Unsigned number of millised to wait for refresh to be valid.
 */
uint16_t Adafruit_VEML7700::getRefreshTime() {
  // Refresh time constants  -- the app note does not give times for
  // VEML7700_IT_25MS, and VEML7700_IT_50MS; just assume they are same as IT.
  static const uint16_t integrationTimes[] = {25, 50, 100, 200, 400, 800};
  static const uint16_t powerSaverTimes[] = {
      500, 1000, 2000, 4000}; // deduced from app note table on page 16
  static const uint16_t powerOnWait =
      3; // the app note says 2.5 ms; round it up to 3
  uint8_t psm = getPowerSaveMode();
  uint8_t iIndex = integrationTimeIndex[getIntegrationTime()];
  // powerOnWait may not be needed in return value, but it is so
  //    small with respect to the others, we may as well include it.
  return powerSaverTimes[psm] + integrationTimes[iIndex] + powerOnWait;
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
    // equation refactored to avoid pow() function.
    lux = (((6.0135e-13 * lux - 9.3924e-9) * lux + 8.1488e-5) * lux + 1.0023) *
          lux;
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
    // Equation refactored to avoit pow() function.
    white =
        (((2E-15 * white + 4E-12) * white + 9E-06) * white + 1.0179) * white -
        11.052;
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
 *    @brief Get ALS integration time factor
 *    @returns IT factor, can be VEML7700_IT_100MS, VEML7700_IT_200MS,
 * VEML7700_IT_400MS, VEML7700_IT_800MS, VEML7700_IT_50MS or VEML7700_IT_25MS
 */
double Adafruit_VEML7700::getIntegrationTimeFactor(void) {
  uint8_t itIndex = ALS_Integration_Time->read();
  if (itIndex >= sizeof(integrationTimeCoeff) / sizeof(integrationTimeCoeff[0]))
    itIndex = 0;
  return 1.0 / integrationTimeCoeff[itIndex];
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
 *    @brief Get ALS gain value
 *    @returns Gain Value, can be VEML7700_GAIN_1 = 1 , VEML7700_GAIN_2 = 2,
 * VEML7700_GAIN_1_8 = 1/8 or VEML7700_GAIN_1_4 = 1/4
 */
double Adafruit_VEML7700::getGainValue(void) {
  return 1.0 / gainCoeff[ALS_Gain->read()];
}

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
