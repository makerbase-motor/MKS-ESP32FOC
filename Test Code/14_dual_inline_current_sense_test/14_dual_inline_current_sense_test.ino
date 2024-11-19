//MKS ESP32 FOC V2.0 | Inline Current Sensing Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0

#include <SimpleFOC.h>

// Current Sensing
// Sampling Resistor Value Gain ADC Pin
InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 35, 34);


void setup() {
  // Current Sensing
  current_sense0.init();
  current_sense1.init();

  Serial.begin(115200);
  Serial.println("Current sense ready.");
}

int count;
void loop() {

  PhaseCurrent_s currents0 = current_sense0.getPhaseCurrents();
  float current_magnitude0 = current_sense0.getDCCurrent();
  PhaseCurrent_s currents1 = current_sense1.getPhaseCurrents();
  float current_magnitude1 = current_sense1.getDCCurrent();

  count++;
  if (count >= 50) {

    //Use the serial port assistant to draw waveforms
    Serial.printf("%.2f,%.2f,%.2f,%.2f\n", currents0.a * 1000, currents0.b * 1000, currents0.c * 1000, current_magnitude0 * 1000);  // milli Amps
    // Serial.printf("%.2f,%.2f,%.2f,%.2f\n", currents1.a * 1000, currents1.b * 1000, currents1.c * 1000, current_magnitude1 * 1000);  // milli Amps

    count = 0;
  }
}
