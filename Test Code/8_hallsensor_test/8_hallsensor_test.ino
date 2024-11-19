// MKS ESP32 FOC V2.0 | Five-wire Hallsensor Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0

#include <SimpleFOC.h>

//Encoder Example
//18——Corresponding to the pin SCL_0
//19——Corresponding to the pin SDA_0
//15——Corresponding to the pin GH
//1——Pole pairs
HallSensor sensor1 = HallSensor(18, 19, 15, 1);
//5——Corresponding to the pin SCL_0
//23——Corresponding to the pin SDA_0
//14——Corresponding to the pin GH
//1——Pole pairs
HallSensor sensor2 = HallSensor(5, 23, 14, 1);

void doA(){sensor1.handleA();}
void doB(){sensor1.handleB();}
void doC(){sensor1.handleC();}
void doA1(){sensor2.handleA();}
void doB1(){sensor2.handleB();}
void doC1(){sensor2.handleC();}

void setup() {
//  // Encoder Settings
//  sensor1.pullup = Pullup::USE_EXTERN;
  
  // Initialize the magnetic sensor hardware
  sensor1.init();
  sensor2.init();
  // Enable hardware interrupts
  sensor1.enableInterrupts(doA, doB, doC);
  sensor2.enableInterrupts(doA1, doB1, doC1);

  
  Serial.begin(115200);
  

  Serial.println("Sensor ready");
  _delay(1000);


}

void loop() {

  Serial.print(sensor1.getAngle());
  Serial.print("\t");
  Serial.println(sensor2.getAngle());

}
