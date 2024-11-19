// MKS ESP32 FOC V2.0 | AS5047P Encoder ABI Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0 & MKS AS5047P
// Test encoder AS5047P,CPR=4000
// Driver board corresponding interface sequence:3.3V GND A B I


#include <SimpleFOC.h>

//For ABI interface (0) number
//Encoder encoder = Encoder(19,18,1000,15); //A0;B0;EncoderPPR,PPR=CPR/4;I0

//For ABI interface (1)
Encoder encoder = Encoder(23,5,1000,13); //A1;B1;EncoderPPR,PPR=CPR/4;I1

void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

void setup() {
  Serial.begin(115200);
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;

  encoder.init();
  // Hardware interrupt enable
  encoder.enableInterrupts(doA, doB);

  Serial.println("Encoder ready");
  _delay(1000);
}

void loop() {
  // Output angle and angular velocity
  Serial.print(encoder.getAngle());
  Serial.print("\t");
  Serial.println(encoder.getVelocity());
}
