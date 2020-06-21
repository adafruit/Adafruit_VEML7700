#ifndef VEML7700_AUTO_GAIN_H
#define VEML7700_AUTO_GAIN_H

#include "Adafruit_VEML7700.h"
#include <Wire.h>

#define VEML7700_GAIN_MIN VEML7700_GAIN_1_8
#define VEML7700_GAIN_MAX VEML7700_GAIN_2

#define VEML7700_IT_MAX VEML7700_IT_800MS
#define VEML7700_IT_MIN VEML7700_IT_25MS

enum VemlResultType { GOOD, TOO_LOW, TOO_HIGH };

/**
 * @brief return type of readLuxAuto()
 *
 */
struct VemlAutoResult {
  /**
   * @brief ALS reading in lux.
   *
   */
  float val;
  /**
   * @brief If the ALS reading is good, too dark, or too bright.
   *
   */
  VemlResultType type;
};

/**
 * @brief Compatible with Adafruit_VEML7700, with the added function of
 * automatically setting gain and integration time.
 *
 */
class VEML7700AutoGain : public Adafruit_VEML7700 {
public:
  boolean begin(TwoWire *theWire = &Wire);
  void setIntegrationTime(uint8_t it);
  void setGain(uint8_t gain);
  void setAutoThreshold(uint16_t low, uint16_t high);
  uint16_t getIntegrationMs();
  VemlAutoResult readLuxAuto(bool apply_correction = true);

private:
  uint8_t veml_it, veml_gain;
  uint16_t low_threshold, high_threshold;
  static uint8_t prevIt(uint8_t it);
  static uint8_t nextIt(uint8_t it);
  static uint8_t prevGain(uint8_t gain);
  static uint8_t nextGain(uint8_t gain);
  float lazyNormalize(float value);
};

#endif