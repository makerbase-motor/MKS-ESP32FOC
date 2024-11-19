// MKS ESP32 FOC V2.0 | Current Close Loop Velocity Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0 & MKS AS5600

// !!!Notice!!!
// ①Enter "A+number" in the serial port to set the position of the M0 motor. For example, enter "A3.14" to rotate the M0 motor to 180° (radians). Enter "B+number" to set the speed of the M1 motor.
// ②When using your own motor, be sure to modify the default number of pole pairs, that is, the value in BLDCMotor(7), to the number of pole pairs of your own motor.
// ③Please set the correct voltage_limit and current_limit values ​​according to the selected motor. It is recommended to set the values ​​between 0.5 and 1.0 for the aircraft model motor and below 4 for the gimbal motor. Excessive voltage and current may burn out the driver board!
// ④The pid parameters of this routine can control the 2808 model aircraft motor. If you want to achieve better results or use other motors, please adjust the pid parameters yourself.

#include <SimpleFOC.h>

//Motor Instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

//Encoder Instance
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);

// Inline Current sensing Instance
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01, 50.0, 35, 34);

// commander Communication Instance
Commander command = Commander(Serial);
void doMotor1(char* cmd) {
  command.motor(&motor1, cmd);
}
void doMotor2(char* cmd) {
  command.motor(&motor2, cmd);
}

//Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1
void board_init();
float get_vin_Volt();

void setup() {
  Serial.begin(115200);
  board_init();

  // Encoder Settings
  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL); // AS5600_M1

  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

  //Connect the motor object and the sensor object
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // Drive Settings
  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = get_vin_Volt();
  driver2.init();
  motor2.linkDriver(&driver2);

  // Current Limitation
  motor1.current_limit = 0.5;
  motor2.current_limit = 0.5;

  // Voltage Limitation
  motor1.voltage_limit = 0.5;
  motor2.voltage_limit = 0.5;


  // Current Sensing
  current_sense1.init();
  // current_sense1.gain_b *= 1;
  // current_sense1.gain_a *= 1;
  //  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);

  // current sense init and linking
  current_sense2.init();
  // current_sense2.gain_b *= 1;
  // current_sense2.gain_a *= 1;
  //  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // Control loop
  // Other modes TorqueControlType::voltage TorqueControlType::dc_current
  motor1.torque_controller = TorqueControlType::foc_current;
  motor1.controller = MotionControlType::angle;
  motor2.torque_controller = TorqueControlType::foc_current;
  motor2.controller = MotionControlType::angle;

  motor1.voltage_sensor_align = 5;
  motor2.voltage_sensor_align = 5;

  // FOC current control PID parameters
  motor1.PID_current_q.P = 1;
  motor1.PID_current_q.I = 500;
  motor1.PID_current_d.P = 1;
  motor1.PID_current_d.I = 500;
  motor1.LPF_current_q.Tf = 0.002;  // 1ms default
  motor1.LPF_current_d.Tf = 0.002;  // 1ms default

  motor2.PID_current_q.P = 1;
  motor2.PID_current_q.I = 500;
  motor2.PID_current_d.P = 1;
  motor2.PID_current_d.I = 500;
  motor2.LPF_current_q.Tf = 0.002;  // 1ms default
  motor2.LPF_current_d.Tf = 0.002;  // 1ms default

  // Speed ​​loop PID parameters
  motor1.PID_velocity.P = 0.021;
  motor1.PID_velocity.I = 0.12;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.021;
  motor2.PID_velocity.I = 0.12;
  motor2.PID_velocity.D = 0;
  // default voltage_power_supply
  motor1.P_angle.P = 20;
  motor2.P_angle.P = 20;
  // Speed ​​Limit
  motor1.velocity_limit = 20;
  motor2.velocity_limit = 20;


  // monitor Interface Settings
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // monitor Related settings
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  //Motor initialization
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC();

  motor2.init();
  // align encoder and start FOC
  motor2.initFOC();

  // Initial target value
  motor1.target = 0.0;
  motor2.target = 0.0;

  // Mapping motors to commanders
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  Serial.println(F("Double motor sketch ready."));

  _delay(1000);
}


void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move();
  motor2.move();

  command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);

  analogReadResolution(12);  //12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power on, current voltage%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor...Current voltage%.2f\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}
