// MKS ESP32 FOC Mega Current Close Loop Velocity Example Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC PLUS & MKS AS5600

// ！！！Cautions！！！
// ① Enter "A" followed by a number via the serial port to set the rotation speed of the M0 motor; for example, entering "A10" sets the M0 motor speed to 10 rad/s.
// ② When using your own motor, be sure to update the default pole pair count—specifically the value within `BLDCMotor(7)`—to match your motor's actual pole pair count.
// ③ Configure the `voltage_limit` and `current_limit` values ​​appropriately for your chosen motor; recommended settings are 0.5–1.0 for RC aircraft motors and below 4.0 for gimbal motors. Excessive voltage or current may damage the driver board.
// ④ The PID parameters in this example are suitable for the 2804 RC aircraft motor; please adjust the PID parameters yourself to achieve better performance or when using a different motor.
#include <SimpleFOC.h>

//Motor instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 12);

//Encoder instance
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

// In-line current sensing instance
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);

// commander instance
Commander command = Commander(Serial);
void doMotor1(char* cmd) {
  command.motor(&motor1, cmd);
}

//Set alarm voltage
#define UNDERVOLTAGE_THRES 11.1
void board_init();
float get_vin_Volt();

void setup() {
  Serial.begin(115200);
  board_init();

  // Encoder Settings
  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  sensor1.init(&I2Cone);

  //Connect the motor object and the sensor object.
  motor1.linkSensor(&sensor1);

  // Drive Settings
  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  motor1.linkDriver(&driver1);

  // Current limiting
  motor1.current_limit = 0.5;

  // Voltage limit
  motor1.voltage_limit = 0.5;

  // Current sensing
  current_sense1.init();
  current_sense1.gain_b *= 1;
  current_sense1.gain_a *= 1;
  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);

  // Control loop
  // Other modes TorqueControlType::voltage TorqueControlType::dc_current
  motor1.torque_controller = TorqueControlType::foc_current;
  motor1.controller = MotionControlType::velocity;

  motor1.voltage_sensor_align = 5;

  // FOC Current Control PID Parameters
  motor1.PID_current_q.P = 1;
  motor1.PID_current_q.I = 500;
  motor1.PID_current_d.P = 1;
  motor1.PID_current_d.I = 500;
  motor1.LPF_current_q.Tf = 0.002;  // 1ms default
  motor1.LPF_current_d.Tf = 0.002;  // 1ms default

  // Speed ​​loop PID parameters
  motor1.PID_velocity.P = 0.021;
  motor1.PID_velocity.I = 0.12;
  motor1.PID_velocity.D = 0;

  // default voltage_power_supply

  // Speed ​​limit
  motor1.velocity_limit = 20;

  // monitor Interface Settings
  // comment out if not needed
  motor1.useMonitoring(Serial);

  // monitor Related settings
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  //Motor initialization
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC();

  // Initial target value
  motor1.target = 0.0;

  // Map motor tocommander
  command.add('A', doMotor1, "motor 1");

  Serial.println(F("Double motor sketch ready."));

  _delay(1000);
}


void loop() {
  motor1.loopFOC();

  motor1.move();

  command.run();
}

void board_init() {
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);

  analogReadResolution(12);  //12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES) {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power-up, current voltage:%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor... Current voltage:%.2f\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}
