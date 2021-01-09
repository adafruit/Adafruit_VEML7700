#include <Wire.h>
#include <Adafruit_VEML7700.h>  // https://github.com/adafruit/Adafruit_VEML7700

/*
   objective:
     * show use of luxAutoSensor()
     * show dynamic class creation at runtime

   2020 Holger Lembke
*/

Adafruit_VEML7700 * veml7700 = NULL;

//==========================================================
bool findI2CDevice(const byte addr) {
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}

//==========================================================
void die() {
  while (1) {
    delay(10);  
  }
}

//==========================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\nVEML7700 test"));

  Wire.begin();

  // Look out for device...
  if (! findI2CDevice(0x10)) {
    Serial.println("VEML7700: Device not found.");
    die(); 
  }

  veml7700 = new Adafruit_VEML7700();

  if (! veml7700->begin()) {
    Serial.println("VEML7700: Sensor not found.");
    delete veml7700;
    veml7700 = NULL;
    die(); 
  }
}

//==========================================================
void loop() {
  unsigned long start = millis();
  float lux = veml7700->luxAutoSensor();
  unsigned long ende = millis();

  Serial.print("VEML7700: ");
  Serial.print(lux);
  Serial.print(" lux, duration: ");
  Serial.print(ende-start);
  Serial.print(" ms ");
  Serial.println();
  delay(1000);
}
