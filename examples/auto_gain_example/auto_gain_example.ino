#include "VEML7700AutoGain.h"

VEML7700AutoGain veml = VEML7700AutoGain();

void setup() {
  while (!Serial) {
    delay(10);
  }
  Serial.begin(115200);
  Serial.println("Adafruit VEML7700 Test");

  if (!veml.begin()) {
    Serial.println("Sensor not found");
    while (1)
      ;
  }
  Serial.println("Sensor found");

  veml.enable(false);
  veml.setGain(VEML7700_GAIN_1_8);
  veml.setIntegrationTime(VEML7700_IT_100MS);
  // when ALS count is lower than 100 or higher than 10000, calling
  // readLuxAuto() will automatically change gain and integration time.
  veml.setAutoThreshold(100, 10000);
  veml.enable(true);
}

char buffer[200];

float lux2ev(float lux) { return log(lux / 2.5) / log(2); }

void loop() {
  delay(veml.getIntegrationMs() + 50); // wait for the result to be avaliable
  VemlAutoResult result = veml.readLuxAuto();

  char const *gain_str = "";

  switch (veml.getGain()) {
  case VEML7700_GAIN_1:
    gain_str = "1";
    break;
  case VEML7700_GAIN_2:
    gain_str = "2";
    break;
  case VEML7700_GAIN_1_4:
    gain_str = "1/4";
    break;
  case VEML7700_GAIN_1_8:
    gain_str = "1/8";
    break;
  }

  sprintf(buffer, "%6d.%02d lux;\t%3sx gain;\t%3ds intergration time; %s%s",
          (int)result.val, (int)(result.val * 100) % 100, gain_str,
          veml.getIntegrationMs(), (result.type == TOO_LOW ? "LOW" : ""),
          (result.type == TOO_HIGH ? "HIGH" : ""));

  Serial.println(buffer);
}