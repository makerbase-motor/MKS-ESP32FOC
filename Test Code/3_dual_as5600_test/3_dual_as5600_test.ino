// MKS ESP32 FOC AS5600 Text Example; Text Hardware：MKS ESP32 FOC V1.0
// Turn the motor manually, and you can receive the motor position displayed on the serial monitor
// The first and second columns are the position coordinates of the M0 and M1 motors respectively

#include <SimpleFOC.h>


MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);


void setup() {

  
  Serial.begin(115200);
  _delay(750);
  I2Cone.begin(19,18, 400000);   //SDA0,SCL0
  I2Ctwo.begin(23,5, 400000);
  
  //For the latest version ESP-Arduino 2.0.2, use the following two lines of code
  //I2Cone.begin(19,18, 400000UL);   //SDA0,SCL0
  //I2Ctwo.begin(23,5, 400000UL);
  
  sensor0.init(&I2Cone);
  sensor1.init(&I2Ctwo);
}

void loop() {
  // sensor0.update(); // If the simplefoc library version is 2.20 and above, uncomment these two lines
  // sensor1.update();
  //_delay(200);
  Serial.print(sensor0.getAngle()); 
  Serial.print(" - "); 
  Serial.print(sensor1.getAngle());
  Serial.println();
}
