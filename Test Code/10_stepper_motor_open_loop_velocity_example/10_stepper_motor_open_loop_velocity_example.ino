// ESP32 FOC Stepper Motor Open Loop Velocity Example; Test Library：SimpleFOC 2.2.1; Test Hardware：MKS ESP32 FOC V1.0;
// Wiring Relationship of Stepper Motor：A+:C0 A-:B0 B+:C1 B-:B1
// Input "T+number" in the serial port window to control the speed of the motor.
// For example, to set the motor to rotate at 10rad/s, input "T10".
// When the motor is powered on, it will rotate at 5rad/s by default.
// When using your own motor, remember to modify the default number of pole pairs, the value in StepperMotor(50).
// The default power supply voltage set by the program is 12V.
// Remember to modify the value of the voltage_power_supply variable if you use other voltages for power supply.

#include <SimpleFOC.h>

// Stepper motor instance
StepperMotor motor = StepperMotor(50);     //Modify the value in StepperMotor() here according to the number of pole pairs of the selected motor
// Stepper driver instance
StepperDriver4PWM driver = StepperDriver4PWM(33, 32,26,27, 22, 12); 


// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  driver.pwm_frequency = 40000;

  driver.voltage_power_supply = 12;               //Modify the value of voltage_power_supply here according to the supply voltage
  driver.init();
  motor.linkDriver(&driver);

  // Open Loop Control Mode Setting
  motor.controller = MotionControlType::velocity_openloop;

  // Initialize the Serial Port
  Serial.begin(115200);
  motor.useMonitoring(Serial);

  // Initialization
  motor.init();

  // Initial Target Value Setting
  motor.target = 5;

  // Add T Instruction
  command.add('T', onMotor, "motor");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  
  _delay(1000);
}


void loop() {
  motor.move();

  command.run();

}
