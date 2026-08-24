// MKS ESP32 FOC Mega Hall Encoder Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC PLUS

#include <SimpleFOC.h>

//Encoder
//18——SCL_0
//19——SDA_0
//15——I_0
//1——Pole Pairs
HallSensor sensor1 = HallSensor(18, 19, 15, 1);

void doA(){sensor1.handleA();}
void doB(){sensor1.handleB();}
void doC(){sensor1.handleC();}

void setup() {
//  // Encoder Setting
//  sensor1.pullup = Pullup::USE_EXTERN;
  
  // Initialize magnetic sensor hardware.
  sensor1.init();
  // Enable hardware interrupts
  sensor1.enableInterrupts(doA, doB, doC);
  
  Serial.begin(115200);
  
  Serial.println("Sensor ready");
  _delay(1000);


}

void loop() {

  Serial.print(sensor1.getAngle());
  Serial.print("\t");

}
