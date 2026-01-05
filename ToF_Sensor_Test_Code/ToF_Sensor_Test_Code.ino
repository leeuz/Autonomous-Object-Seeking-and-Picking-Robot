#include <Wire.h>
#include <Adafruit_VL6180X.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10); // wait for serial monitor
  }

  Serial.println("VL6180X Distance Test");

  if (!vl.begin()) {
    Serial.println("Failed to find VL6180X sensor");
    while (1);
  }

  Serial.println("VL6180X sensor detected");
}

void loop() {
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Distance: ");
    Serial.print(range);
    Serial.println(" mm");
  } else {
    Serial.print("Range error: ");
    Serial.println(status);
  }

  delay(200);
}
