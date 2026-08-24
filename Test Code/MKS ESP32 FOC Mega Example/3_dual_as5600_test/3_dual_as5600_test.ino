// MKS ESP32 FOC Mega AS5600 Encoder I2C Example Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC Mega & MKS AS5600
#include <SimpleFOC.h>

MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);


void setup() {
  Serial.begin(115200);

  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL); // AS5600_M1
  
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
}

void loop() {
  sensor0.update();  // If the SimpleFOC library version is 2.2.0 or higher, uncomment these two lines.
  sensor1.update();
  Serial.print(sensor0.getAngle());
  Serial.print(" - ");
  Serial.println(sensor1.getAngle());
  Serial.println();
}
