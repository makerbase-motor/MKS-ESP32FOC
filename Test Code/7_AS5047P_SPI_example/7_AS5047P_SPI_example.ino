// MKS ESP32 FOC V2.0 | AS5047P Encoder SPI Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0 & MKS AS5047P
// Test Encoder AS5047P,SPI Interface


#include <SimpleFOC.h>

#define HSPI_MISO 19 // Corresponding to the SDA_0 pin on the dual line
#define HSPI_MOSI 23 // Corresponding to the SDA_1 pin on the dual line
#define HSPI_SCLK 18 // Corresponding to the SCL_0 pin on the dual line
#define HSPI_SS 5 // Corresponding to the SCL_1 pin on the dual line


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
//  sensor.update(); // If the simpleFOC library version is 2.2.0 or above, you need to uncomment this line
  // Output angle and angular velocity
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
