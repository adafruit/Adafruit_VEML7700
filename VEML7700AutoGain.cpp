#include "VEML7700AutoGain.h"
using namespace std;

/**
 * @brief Setups the hardware, initializes local gain, intergration time,
 * and auto gain thresholds.
 *
 * @param theWire An optional pointer to an I2C interface
 * @return boolean. True if initialization was successful, otherwise false.
 */
boolean VEML7700AutoGain::begin(TwoWire *theWire) {
  if (!Adafruit_VEML7700::begin(theWire))
    return false;
  veml_gain = getGain();
  veml_it = getIntegrationTime();
  setAutoThreshold(100, 10000);
  return true;
}

/**
 * @brief Set ALS integration time. Overloaded to keep track of intergration
 * time
 *
 * @param it Can be VEML7700_IT_100MS, VEML7700_IT_200MS, VEML7700_IT_400MS,
 *    VEML7700_IT_800MS, VEML7700_IT_50MS or VEML7700_IT_25MS
 */
void VEML7700AutoGain::setIntegrationTime(uint8_t it) {
  veml_it = it;
  Adafruit_VEML7700::setIntegrationTime(it);
}

/**
 * @brief Set ALS gain. Overloaded to keep track of gain
 *
 * @param gain Can be VEML7700_GAIN_1, VEML7700_GAIN_2, VEML7700_GAIN_1_8 or
 * VEML7700_GAIN_1_4
 */
void VEML7700AutoGain::setGain(uint8_t gain) {
  veml_gain = gain;
  Adafruit_VEML7700::setGain(gain);
}

/**
 * @brief Sets auto gain thresholds. See app note page 21.
 * https://www.vishay.com/docs/84323/designingveml7700.pdf
 *
 * @param low Low ALS count threshold. When ALS count is less than low,
 * gain or intergration time is increased.
 * @param high High ALS count threshold. When ALS count is greater than high,
 * gain or intergration time is reduced.
 */
void VEML7700AutoGain::setAutoThreshold(uint16_t low, uint16_t high) {
  low_threshold = low;
  high_threshold = high;
}

/**
 * @brief Normalize ALS count, scale using local gain and integration time.
 *
 * @param value ALS count
 * @return float. Normalized ALS count. (normalized: gain = 1x, intergration
 * time = 100ms)
 */
float VEML7700AutoGain::lazyNormalize(float value) {
  {
    // adjust for gain (1x is normalized)
    switch (veml_gain) {
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
    switch (veml_it) {
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
}

/**
 * @brief Read the lux value and change gain or intergration time automatically
 * if the reading is too dark or too bight. Next call will use the changed gain
 * and intergration time. Implements a modified version of the flow chart on app
 * note page 21.
 *
 * https://www.vishay.com/docs/84323/designingveml7700.pdf
 *
 * @param apply_correction If non-linearity correction is applied.
 * @return A struct containing ALS reading in lux, and "type" indicating if the
 * reading is good, too dark, or too bright
 */
VemlAutoResult VEML7700AutoGain::readLuxAuto(bool apply_correction) {
  // return normalize(ALS_Data->read()) * 0.0576;  // see app note lux table on
  // page 5
  uint16_t count = readALS();
  VemlResultType type = GOOD;
  float lux = lazyNormalize(count) * 0.0576f;
  if (apply_correction && veml_gain == VEML7700_GAIN_1_8 &&
      veml_it == VEML7700_IT_25MS) {
    lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) +
          8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
  }
  if (count < low_threshold) {
    if (getIntegrationMs() < 100) {
      enable(false);
      setIntegrationTime(nextIt(veml_it));
      enable(true);
    } else if (veml_gain != VEML7700_GAIN_MAX) {
      enable(false);
      setGain(nextGain(veml_gain));
      enable(true);
    } else if (veml_it != VEML7700_IT_MAX) {
      enable(false);
      setIntegrationTime(nextIt(veml_it));
      enable(true);
    }
    type = TOO_LOW;
  } else if (count > high_threshold) {
    if (getIntegrationMs() > 100) {
      enable(false);
      setIntegrationTime(prevIt(veml_it));
      enable(true);
    } else if (veml_gain != VEML7700_GAIN_MIN) {
      enable(false);
      setGain(prevGain(veml_gain));
      enable(true);
    } else if (veml_it != VEML7700_IT_MIN) {
      enable(false);
      setIntegrationTime(prevIt(veml_it));
      enable(true);
    }
    type = TOO_HIGH;
  }

  return {lux, type};
}

/**
 * @brief Returns current integration time in milliseconds
 *
 * @return uint16_t. Current integration time in milliseconds
 */
uint16_t VEML7700AutoGain::getIntegrationMs() {
  switch (veml_it) {
  case VEML7700_IT_100MS:
    return 100;
  case VEML7700_IT_200MS:
    return 200;
  case VEML7700_IT_400MS:
    return 400;
  case VEML7700_IT_800MS:
    return 800;
  case VEML7700_IT_50MS:
    return 50;
  case VEML7700_IT_25MS:
    return 25;
  }
  return 0;
}

/**
 * @brief Return the integration time that is one step below the current one.
 *
 * @param it Current integration time (VEML7700_IT_*)
 * @return uint8_t. Integration time one step below "it" if "it" is valid and
 * isn't VEML7700_IT_25MS, returns MAX uint8_t (255) otherwise.
 */
uint8_t VEML7700AutoGain::prevIt(uint8_t it) {
  switch (it) {
  case VEML7700_IT_50MS:
    return VEML7700_IT_25MS;
  case VEML7700_IT_100MS:
    return VEML7700_IT_50MS;
  case VEML7700_IT_200MS:
    return VEML7700_IT_100MS;
  case VEML7700_IT_400MS:
    return VEML7700_IT_200MS;
  case VEML7700_IT_800MS:
    return VEML7700_IT_400MS;
  }
  return ~0;
}

/**
 * @brief Return the integration time that is one step above the current one.
 *
 * @param it Current integration time (VEML7700_IT_*)
 * @return uint8_t. Integration time one step above "it" if "it" is valid and
 * isn't VEML7700_IT_800MS, returns MAX uint8_t (255) otherwise.
 */
uint8_t VEML7700AutoGain::nextIt(uint8_t it) {
  switch (it) {
  case VEML7700_IT_25MS:
    return VEML7700_IT_50MS;
  case VEML7700_IT_50MS:
    return VEML7700_IT_100MS;
  case VEML7700_IT_100MS:
    return VEML7700_IT_200MS;
  case VEML7700_IT_200MS:
    return VEML7700_IT_400MS;
  case VEML7700_IT_400MS:
    return VEML7700_IT_800MS;
  }
  return ~0;
}

/**
 * @brief Return the gain that is one step below the current one.
 *
 * @param it Current gain (VEML7700_GAIN_*)
 * @return uint8_t. Gain one step below "gain" if "gain" is valid and isn't
 * VEML7700_GAIN_1_8, returns MAX uint8_t (255) otherwise.
 */
uint8_t VEML7700AutoGain::prevGain(uint8_t gain) {
  switch (gain) {
  case VEML7700_GAIN_1_4:
    return VEML7700_GAIN_1_8;
  case VEML7700_GAIN_1:
    return VEML7700_GAIN_1_4;
  case VEML7700_GAIN_2:
    return VEML7700_GAIN_1;
  }
  return ~0;
}

/**
 * @brief Return the gain that is one step above the current one.
 *
 * @param it Current gain (VEML7700_GAIN_*)
 * @return uint8_t. Gain one step above "gain" if "gain" is valid and isn't
 * VEML7700_GAIN_1_8, returns MAX uint8_t (255) otherwise.
 */
uint8_t VEML7700AutoGain::nextGain(uint8_t gain) {
  switch (gain) {
  case VEML7700_GAIN_1_8:
    return VEML7700_GAIN_1_4;
  case VEML7700_GAIN_1_4:
    return VEML7700_GAIN_1;
  case VEML7700_GAIN_1:
    return VEML7700_GAIN_2;
  }
  return ~0;
}