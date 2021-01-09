/*!
 *  @file Adafruit_VEML7700.h
 *
 * 	I2C Driver for VEML7700 Lux sensor
 *
 * 	This is a library for the Adafruit VEML7700 breakout:
 * 	http://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_VEML7700_H
#define _ADAFRUIT_VEML7700_H

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Wire.h>

#define VEML7700_I2CADDR_DEFAULT 0x10 ///< I2C address

#define VEML7700_ALS_CONFIG 0x00        ///< Light configuration register
#define VEML7700_ALS_THREHOLD_HIGH 0x01 ///< Light high threshold for irq
#define VEML7700_ALS_THREHOLD_LOW 0x02  ///< Light low threshold for irq
#define VEML7700_ALS_POWER_SAVE 0x03    ///< Power save regiester
#define VEML7700_ALS_DATA 0x04          ///< The light data output
#define VEML7700_WHITE_DATA 0x05        ///< The white light data output
#define VEML7700_INTERRUPTSTATUS 0x06   ///< What IRQ (if any)

#define VEML7700_INTERRUPT_HIGH 0x4000 ///< Interrupt status for high threshold
#define VEML7700_INTERRUPT_LOW 0x8000  ///< Interrupt status for low threshold

#define VEML7700_GAIN_1 0x00   ///< ALS gain 1x
#define VEML7700_GAIN_2 0x01   ///< ALS gain 2x
#define VEML7700_GAIN_1_8 0x02 ///< ALS gain 1/8x
#define VEML7700_GAIN_1_4 0x03 ///< ALS gain 1/4x

#define VEML7700_IT_100MS 0x00 ///< ALS intetgration time 100ms
#define VEML7700_IT_200MS 0x01 ///< ALS intetgration time 200ms
#define VEML7700_IT_400MS 0x02 ///< ALS intetgration time 400ms
#define VEML7700_IT_800MS 0x03 ///< ALS intetgration time 800ms
#define VEML7700_IT_50MS 0x08  ///< ALS intetgration time 50ms
#define VEML7700_IT_25MS 0x0C  ///< ALS intetgration time 25ms

#define VEML7700_PERS_1 0x00 ///< ALS irq persisance 1 sample
#define VEML7700_PERS_2 0x01 ///< ALS irq persisance 2 samples
#define VEML7700_PERS_4 0x02 ///< ALS irq persisance 4 samples
#define VEML7700_PERS_8 0x03 ///< ALS irq persisance 8 samples

#define VEML7700_POWERSAVE_MODE1 0x00 ///< Power saving mode 1
#define VEML7700_POWERSAVE_MODE2 0x01 ///< Power saving mode 2
#define VEML7700_POWERSAVE_MODE3 0x02 ///< Power saving mode 3
#define VEML7700_POWERSAVE_MODE4 0x03 ///< Power saving mode 4

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            VEML7700 Temp Sensor
 */
class Adafruit_VEML7700 {
public:
  Adafruit_VEML7700()
      : ALS_Config(NULL), ALS_Data(NULL), White_Data(NULL),
        ALS_HighThreshold(NULL), ALS_LowThreshold(NULL), Power_Saving(NULL),
        Interrupt_Status(NULL), ALS_Shutdown(NULL), ALS_Interrupt_Enable(NULL),
        ALS_Persistence(NULL), ALS_Integration_Time(NULL), ALS_Gain(NULL),
        PowerSave_Enable(NULL), PowerSave_Mode(NULL), i2c_dev(NULL){};
  bool begin(TwoWire *theWire = &Wire);

  void enable(bool enable);
  bool enabled(void);

  void interruptEnable(bool enable);
  bool interruptEnabled(void);
  void setPersistence(uint8_t pers);
  uint8_t getPersistence(void);
  void setIntegrationTime(uint8_t it);
  uint8_t getIntegrationTime(void);
  void setGain(uint8_t gain);
  uint8_t getGain(void);
  void powerSaveEnable(bool enable);
  bool powerSaveEnabled(void);
  void setPowerSaveMode(uint8_t mode);
  uint8_t getPowerSaveMode(void);

  void setLowThreshold(uint16_t value);
  uint16_t getLowThreshold(void);
  void setHighThreshold(uint16_t value);
  uint16_t getHighThreshold(void);
  uint16_t interruptStatus(void);

  float readLux();
  float readLuxNormalized();

  uint16_t readALS();
  float readWhite();
  float readWhiteNormalized();

  /* will measure and calculate the current white lux value
     might use multiple measurements to optimize settings and gives the "best"
     result therefor it might need some time, worst case can be around 2000 (!)
     ms uses delay(), so cooperative multitasking is ensured Follows the
     guidelines from Vishay in "Designing the VEML7700 Into an Application"
                                             Revision: 20-Sep-2019 Document
     Number: 84323 returns negative value in case of error
  */
  float luxAutoSensor();

private:
  Adafruit_I2CRegister *ALS_Config, *ALS_Data, *White_Data, *ALS_HighThreshold,
      *ALS_LowThreshold, *Power_Saving, *Interrupt_Status;
  Adafruit_I2CRegisterBits *ALS_Shutdown, *ALS_Interrupt_Enable,
      *ALS_Persistence, *ALS_Integration_Time, *ALS_Gain, *PowerSave_Enable,
      *PowerSave_Mode;

  float normalize_resolution(float value);

  // hlprs for luxAutoSensor()
  uint8_t ChartGain(const int8_t G);
  uint16_t ChartIT(const int8_t IT);
  float ChartResolution(const int8_t IT, const int8_t G);
  uint16_t IntegrationTime(const int8_t IT);

  Adafruit_I2CDevice *i2c_dev;
};

#endif
