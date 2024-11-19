// MKS ESP32 FOC V2.0 | Close Loop Velocity Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0 & MKS AS5600

// !!!Notice!!!
// ①Enter "T+number" in the serial port to set the speed of the motor. For example, if you enter "T10", the motor speed will be set to 10rad/s.
// ②When using your own motor, be sure to modify the default number of pole pairs, that is, the value in BLDCMotor(7), to the number of pole pairs of your own motor.
// ③Please set the correct voltage_limit value according to the selected motor. It is recommended to set it between 0.5 and 1.0 for the aircraft model motor and below 4 for the gimbal motor. Excessive voltage and current may burn out the driver board!
// ④The pid parameters of this routine can control the 2808 model aircraft motor. If you want to achieve better results or use other motors, please adjust the pid parameters yourself.

#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

//Motor parameters
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//Command settings
float target_velocity = 0;
uint32_t prev_millis;

//Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1

void board_check();
float get_vin_Volt();
void board_init();
bool flag_under_voltage = false;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  Serial.begin(115200);
  board_init();
  
  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  I2Ctwo.begin(23, 5, 400000UL); // AS5600_M1
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //Connect the motor object and the sensor object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply voltage setting [V]
  driver.voltage_power_supply = get_vin_Volt();
  driver.init();

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  //Connect the motor and driver objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion control mode settings
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;


  //Speed ​​PI loop settings
  motor.PID_velocity.P = 0.021;
  motor1.PID_velocity.P = 0.021;
  motor.PID_velocity.I = 0.12;
  motor1.PID_velocity.I = 0.12;
  //Maximum motor limiting voltage
  motor.voltage_limit = 0.5;    // [V] Please modify and check this value carefully, excessive voltage and current may cause the driver board to burn out!!!
  motor1.voltage_limit = 0.5;    // [V] Please modify and check this value carefully, excessive voltage and current may cause the driver board to burn out!!!
  
  //Speed ​​low pass filter time constant
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //Set a maximum speed limit
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  
  //Initialize the motor
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

  //When the voltage is lower than the set value, the motor will be disabled.
  board_check();

  //User Communications
  if (!flag_under_voltage)
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
    delay(500);
    Serial.printf("Waiting for power on, current voltage%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor...Current voltage%.2f\n", VIN_Volt);
}

float get_vin_Volt() {
  return analogReadMilliVolts(13) * 8.5 / 1000;
}

void board_check() {

  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;

  if (curr_millis - prev_millis >= 1000) {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES) {
      flag_under_voltage = true;
      enableState = 0;
      uint8_t count = 5;
      while (count--) {
        vin_Volt = get_vin_Volt();
        if (vin_Volt > UNDERVOLTAGE_THRES) {
          flag_under_voltage = false;
          break;
        }
      }
    } else {
      flag_under_voltage = false;
    }
    if (flag_under_voltage) {
      motor.disable();
      motor1.disable();
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      motor.enable();
      motor1.enable();
    }
    prev_millis = curr_millis;
  }
}
