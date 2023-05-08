/*
MKS ESP32 FOC Closed Loop Position Torque Mutual Control Example; Test library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
Turn one of the motors after power on, and the other motor will follow.
The two motors maintain position and torque mutual control at all times.
When using your own motor, be sure to remember the value of the number of pole pairs, the value in BLDCMotor(7).
The default power supply voltage of the program is 12V.
Please remember to modify the voltage_power_supply , voltage_limit variable values when using other voltages for power supply.
The motor targeted by the default PID is 2804 gimbal motor, and the encoder used is AS5600.
When using your own motor, you need to modify the PID parameters to achieve better results.
 */
 
#include <SimpleFOC.h>


MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor Instance
BLDCMotor motor = BLDCMotor(7);                             //Modify the value in BLDCMotor() according to the number of pole pairs of the selected motor
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);    //Also modify the value of BLDCMotor() here


void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  // Link the Motor to the Sensor
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // Driver Config
  // Power Supply Voltage [V]
  driver.voltage_power_supply = 12;               //According to the supply voltage, modify the value of voltage_power_supply here
  driver.init();

  driver1.voltage_power_supply = 12;              //Also modify the value of voltage_power_supply here
  driver1.init();
  // Link the Motor and the Driver
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // Set Motion Control Loop to be Used
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::torque;

  // Maximal Voltage to be Set to the Motor
  motor.voltage_limit = 12;                       //According to the supply voltage, modify the value of voltage_limit here
  motor1.voltage_limit = 12;                      //Also modify the value of voltage_limit here
  // Use Monitoring with Serial 
  Serial.begin(115200);
  // Comment Out if not Needed
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  
  //Initialize the Motor
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move( 5*(motor1.shaft_angle - motor.shaft_angle));
  motor1.move( 5*(motor.shaft_angle - motor1.shaft_angle));
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}
