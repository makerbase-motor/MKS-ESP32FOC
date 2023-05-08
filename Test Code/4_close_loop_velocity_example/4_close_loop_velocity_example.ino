/*
MKS ESP32 FOC Closed Loop Speed Control Example; Test Library：SimpleFOC 2.1.1; Test Hardware：MKS ESP32 FOC V1.0
Enter the "T+number" command in the serial port window to make the two motors rotate in closed loop.
For example, to have both motors turn at 10rad/s, enter: T10.
When using your own motor, please remember to modify the default number of pole pairs, the value in BLDCMotor()
The default power supply voltage set by the program is 12V.
Please remember to modify the values in voltage_power_supply and voltage_limit if you use other voltages for power supply.
The default PID is for the YT2804 motor. When using your own motor.
You need to modify the PID parameters to achieve better results.
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor Parameters
BLDCMotor motor = BLDCMotor(7);                           //Modify the value in BLDMotor() according to the number of pole pairs of the selected motor
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);                          //Also modify the value in BLDMotor() here
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//Command Settings
float target_velocity = 0;                                //Enter "T+speed" in the serial monitor to make the two motors rotate in closed loop
Commander command = Commander(Serial);                    //For example, to make both motors rotate at a speed of 10rad/s, input "T10"
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone); 
  sensor1.init(&I2Ctwo);
  //Connecting Motor Objects to Sensor Objects
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply Voltage Setting [V]
  driver.voltage_power_supply = 12;               //When using other supply voltages, modify the value of voltage_power_supply here
  driver.init();

  driver1.voltage_power_supply = 12;              //Also modify the value of voltage_power_supply here
  driver1.init();
  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  //PID Settings
  motor.PID_velocity.P = 0.1;
  motor1.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1;
  motor1.PID_velocity.I = 1;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;
  //Maximum Motor Voltage Limit
  motor.voltage_limit = 12;                   //When using other power supply voltages, modify the value of voltage_limit here
  motor1.voltage_limit = 12;                  //Also modify the value of voltage_limit here
  
  //Speed Low-pass Filter Time Constant
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //Maximum Speed Limit Settings
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  
  //Initialize the Motor
  motor.init();
  motor1.init();
  //Initialize FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  
}



void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);

  command.run();
}
