// MKS ESP32 FOC V2.0 | Open Loop Velocity Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0

// !!!Notice!!!
// ①Enter "T+number" in the serial port to set the speed of the two motors. For example, if you want to set the motor to rotate at 10 rad/s, enter "T10". The motor will rotate at 5 rad/s by default when powered on.
// ②When using your own motor, be sure to modify the default number of pole pairs, that is, the value in BLDCMotor(7), to the number of pole pairs of your own motor.
// ③Please set the correct voltage_limit value according to the selected motor. It is recommended to set it between 0.5 and 1.0 for the aircraft model motor and below 4 for the gimbal motor. Excessive voltage and current may burn out the driver board!
// ④Open-loop control inevitably causes heating, so do not run this routine for more than one minute, otherwise overheating will cause the motor or driver board to burn out!

#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//Target variable
float target_velocity = 5;
uint32_t prev_millis;

//Setting the alarm voltage
#define UNDERVOLTAGE_THRES 11.1

//Serial port command settings
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void board_check();
float get_vin_Volt();
void board_init();
bool flag_under_voltage = false;


void setup() {
  Serial.begin(115200);
  board_init();

  driver.voltage_power_supply = get_vin_Volt();
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 0.5;    // [V]  Please modify and check this value carefully, excessive voltage and current may cause the driver board to burn out!!!
  motor.velocity_limit = 30;  // [rad/s]

  driver1.voltage_power_supply = get_vin_Volt();
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = 0.5;    // [V] Please modify and check this value carefully, excessive voltage and current may cause the driver board to burn out!!!
  motor1.velocity_limit = 30;  // [rad/s]

  //Open loop control mode setting
  motor.controller = MotionControlType::velocity_openloop;
  motor1.controller = MotionControlType::velocity_openloop;

  //Initialize the hardware
  motor.init();
  motor1.init();

  //Add T command
  command.add('T', doTarget, "target velocity");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
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
