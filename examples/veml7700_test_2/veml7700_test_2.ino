/*!
 *  @file veml7700_test_2.ino
 *
 *  @mainpage Adafruit VEML7700 I2C Lux Sensor example/test
 *
 *  @section intro_sec Introduction
 *
 * 	Test sketch for the VEML7700 I2C Lux sensor
 *
 * 	This is a test sketch for the Adafruit VEML7700 breakout:
 * 	http://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried (Adafruit Industries)
 *  MojaveTom
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
 *  Added initialization loop to optimize VEML parameters
 *    for current light conditions in setup().
 *  More compact logging.
 *  loop() function:
 *    Re-optimizes VEML parameters if needed.
 *    Periodically changes the power save mode.
 *    Waits for the refresh time before re-sampling.
 */

#include "Adafruit_VEML7700.h"

Adafruit_VEML7700 veml = Adafruit_VEML7700();

void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1);
  }
  Serial.println("Sensor found");

  veml.setGain(VEML7700_GAIN_1);
  veml.setIntegrationTime(VEML7700_IT_800MS);

  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  //veml.powerSaveEnable(true);
  //veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE4);

  Serial.print(F("Default Ambient Light Sensor gain is:  ")); Serial.println(veml.getGainValue());
  // According to App Note, if you are waiting for the sensor to refresh at its desired refresh rate,
  // VEML7700_POWERSAVE_MODE1 uses the least ENERGY per sample.
  veml.setPowerSaveMode(VEML7700_POWERSAVE_MODE1);    // most power and shortest delay.
  veml.setGain(VEML7700_GAIN_1_8);           // Set gain and integration time to ensure
  veml.setIntegrationTime(VEML7700_IT_25MS); // sensor will not be saturated.
  uint16_t refreshTime;
  veml.enable(true);
  uint16_t raw;
  float lux;
  do
  {
    refreshTime = veml.getRefreshTime();
    Serial.print(F("  ...  Refresh time for veml is: ")); Serial.print(refreshTime);
    Serial.print(F(", gain: ")); Serial.print(veml.getGainValue());
    Serial.print(F(", IntegrationTimeFactor: ")); Serial.print(veml.getIntegrationTimeFactor());
    Serial.print(F(", PowerSaveMode: ")); Serial.print(veml.getPowerSaveMode());
    Serial.println();
    delay(refreshTime);
    raw = veml.readALS();
    lux = veml.convertToLux(raw);
    Serial.print(F("Light sample is (raw, lux): ")); Serial.print(raw);
    Serial.print(F(", ")); Serial.print(lux);
  } while  (veml.optimizeParams(raw));
  Serial.println(F("  ... VEML_7700 params optimized for current light conditions."));

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(true);
  Serial.println();
  Serial.println(F("Enter loop function now."));
}

void loop() {
  static uint8_t loopCount = 0;
  static uint8_t currentPowerSaveMode = VEML7700_POWERSAVE_MODE1;
  static const uint8_t numPSModes = 4;
  Serial.print("  Lux: "); Serial.print(veml.readLux());
  Serial.print("  White: "); Serial.print(veml.readWhite());
  uint16_t raw = veml.readALS();
  Serial.print("  Raw ALS: "); Serial.print(raw);
  bool optimalParams = !veml.optimizeParams(raw);

  /* When we have optimal parameters, change the power save mode periodically. */
  if (optimalParams) {
     if (++loopCount > 20) {
        loopCount = 0;
        currentPowerSaveMode++;
        currentPowerSaveMode %= numPSModes;
        Serial.println();
        Serial.print("Setting Power Save Mode to: "); Serial.println(currentPowerSaveMode);
        veml.setPowerSaveMode(currentPowerSaveMode);
     }
  } else {
      Serial.println();
      Serial.print("VEML parameters RE-optimized.");
      currentPowerSaveMode = VEML7700_POWERSAVE_MODE1;
      veml.setPowerSaveMode(currentPowerSaveMode);
      Serial.print(" ... Setting Power Save Mode to: "); Serial.println(currentPowerSaveMode);
      Serial.print(F("New parameters are gain: ")); Serial.print(veml.getGainValue());
      Serial.print(F(", IntegrationTimeFactor: ")); Serial.print(veml.getIntegrationTimeFactor());
      Serial.print(F(", PowerSaveMode: ")); Serial.print(veml.getPowerSaveMode());
      Serial.println();
      loopCount = 0;
  }

  uint16_t irq = veml.interruptStatus();
  if (irq & VEML7700_INTERRUPT_LOW) {
    Serial.println("** Low threshold");
  }
  if (irq & VEML7700_INTERRUPT_HIGH) {
    Serial.println("** High threshold");
  }

  uint16_t refreshTime = veml.getRefreshTime();
  Serial.print("  ... Wait for VEML to be ready again: "); Serial.println(refreshTime);
  delay(max(refreshTime, 500));   // delay at least 500ms
}
