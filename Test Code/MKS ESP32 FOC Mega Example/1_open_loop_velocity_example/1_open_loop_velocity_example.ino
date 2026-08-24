// MKS ESP32 FOC Mega Open Loop Speed Contorl Example Library: SimpleFOC 2.2.1 Hardware：MKS ESP32 FOC Mega

// ！！！Caution！！！
// ①Enter "T" followed by a number via the serial port to set the motor speed; for example, enter "T10" to set the motor to rotate at 10 rad/s. By default, the motor rotates at 5 rad/s upon power-up.
// ②When using your own motor, be sure to modify the default pole pair count—specifically the value in `BLDCMotor(7)`—to match the pole pair count of your motor.
// ③Please set the correct `voltage_limit` value based on the motor being used. It is recommended to set the value between 0.5 and 1.0 for RC model motors and below 4 for gimbal motors; excessive voltage or current may damage the driver board.
// ④Open-loop control inevitably generates heat; therefore, do not run this routine for more than one minute, as overheating could burn out the motor or the driver board.

#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

//Target variable
float target_velocity = 5;
uint32_t prev_millis;

//Set alarm voltage
#define UNDERVOLTAGE_THRES 11.1

//Serial Port Command Configuration
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
  motor.voltage_limit = 0.5;    // [V]  Please exercise caution when modifying and checking this value; excessive voltage or current may cause the driver board to burn out!!!
  motor.velocity_limit = 30;  // [rad/s]

  //Open-loop control mode setting
  motor.controller = MotionControlType::velocity_openloop;

  //Initialize hardware
  motor.init();

  //Add T command
  command.add('T', doTarget, "target velocity");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move(target_velocity);

  //The motor is disabled when the voltage falls below the set value.
  board_check();

  //User Communications
  if (!flag_under_voltage)
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
    delay(500);
    Serial.printf("Waiting for power-up, current voltage:%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor... Current voltage:%.2f\n", VIN_Volt);
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
    } else if (0 == enableState && flag_under_voltage == false) {
      enableState = 1;
      motor.enable();
    }
    prev_millis = curr_millis;
  }
}
