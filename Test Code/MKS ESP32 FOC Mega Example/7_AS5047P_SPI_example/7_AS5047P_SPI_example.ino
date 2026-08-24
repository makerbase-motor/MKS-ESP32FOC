// MKS ESP32 FOC Mega AS5047P Encoder ABI Example Library: SimpleFOC 2.2.1 Hardware：MKS ESP32 FOC Mega & MKS AS5047P
// Encoder: AS5047P, SPI


#include <SimpleFOC.h>

#define HSPI_MISO 19 // SDA_0
#define HSPI_MOSI 23 // SDA_1
#define HSPI_SCLK 18 // SCL_0
#define HSPI_SS 5 // SCL_1


MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, HSPI_SS);
SPIClass SPI_2(HSPI);


void setup() {
  Serial.begin(115200);
  SPI_2.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS); //SCLK, MISO, MOSI, SS 
  sensor.init(&SPI_2);
  

  Serial.println("Encoder ready");
  _delay(1000);
}


void loop() {
//  sensor.update(); // If the simpleFOC library version is 2.2.0 or higher, you need to uncomment this line.
  // Output angle and angular velocity
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
