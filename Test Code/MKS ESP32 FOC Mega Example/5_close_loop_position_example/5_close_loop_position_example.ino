// MKS ESP32 FOC Mega Close Loop Position Example Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC Mega & MKS AS5600

// ！！！Caution！！！
// ①Enter "T" followed by a number via the serial port to set the motor's rotation speed; for example, entering "T10" sets the motor speed to 10 rad/s.
// ②When using your own motor, be sure to modify the default pole pair count—specifically the value in `BLDCMotor(7)`—to match the pole pair count of your motor.
// ③Please set the correct `voltage_limit` value based on the motor being used. It is recommended to set the value between 0.5 and 1.0 for RC model motors and below 4 for gimbal motors; excessive voltage or current may damage the driver board.
// ④The PID parameters in this routine can control the 2808 RC aircraft motor; however, if you wish to achieve better performance or use a different motor, please adjust the PID parameters yourself.

#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

//Motor parameters
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 12);

//Command Settings
float target_angle = 0;
uint32_t prev_millis;

//Set alarm voltage
#define UNDERVOLTAGE_THRES 11.1

Commander command = Commander(Serial);
void doTarget(char *cmd)
{
  command.scalar(&target_angle, cmd);
}

void board_check();
float get_vin_Volt();
void board_init();
bool flag_under_voltage = false;

void setup()
{
  Serial.begin(115200);
  board_init();

  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  sensor.init(&I2Cone);
  //Connect the motor object and the sensor object.
  motor.linkSensor(&sensor);

  //Supply voltage setting [V]
  driver.voltage_power_supply = get_vin_Volt();
  driver.init();

  //Connect the motor and driver objects
  motor.linkDriver(&driver);

  // FOC Model Selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //Motion Control Mode Settings
  motor.controller = MotionControlType::angle;

  //Speed ​​PI Loop Settings
  motor.PID_velocity.P = 0.021;
  motor.PID_velocity.I = 0.12;
  //Angle P-loop setup
  motor.P_angle.P = 30;
  //Maximum motor voltage limit
  motor.voltage_limit = 1;  // [V] Please exercise caution when modifying and checking this value; excessive voltage or current may cause the driver board to burn out!!!

  //Speed ​​low-pass filter time constant
  motor.LPF_velocity.Tf = 0.01;

  //Set maximum speed limit
  motor.velocity_limit = 20;

  motor.useMonitoring(Serial);

  //Initialize the motor
  motor.init();
  //Initialize FOC
  motor.initFOC();
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void loop()
{

  motor.loopFOC();

  motor.move(target_angle);

  //The motor is disabled when the voltage falls below the set value.
  board_check();

  //User Communications
  if (!flag_under_voltage)
    command.run();

  // Serial.print(sensor.getAngle());
  // Serial.print(" - ");
  // Serial.print(sensor1.getAngle());
  // Serial.println();
}

void board_init()
{
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);

  analogReadResolution(12); // 12bit

  float VIN_Volt = get_vin_Volt();
  while (VIN_Volt <= UNDERVOLTAGE_THRES)
  {
    VIN_Volt = get_vin_Volt();
    delay(100);
    Serial.printf("Waiting for power-up... Current voltage:%.2f\n", VIN_Volt);
  }
  Serial.printf("Calibrating motor... Current voltage:%.2f\n", VIN_Volt);
}

float get_vin_Volt()
{
  return analogReadMilliVolts(13) * 8.5 / 1000;
}

void board_check()
{

  uint32_t curr_millis = millis();
  static uint8_t enableState = 0;

  if (curr_millis - prev_millis >= 1000)
  {
    float vin_Volt = get_vin_Volt();

    if (vin_Volt < UNDERVOLTAGE_THRES)
    {
      flag_under_voltage = true;
      enableState = 0;
      uint8_t count = 5;
      while (count--)
      {
        vin_Volt = get_vin_Volt();
        if (vin_Volt > UNDERVOLTAGE_THRES)
        {
          flag_under_voltage = false;
          break;
        }
      }
    }
    else
    {
      flag_under_voltage = false;
    }
    if (flag_under_voltage)
    {
      motor.disable();
    }
    else if (0 == enableState && flag_under_voltage == false)
    {
      enableState = 1;
      motor.enable(); 
    }
    prev_millis = curr_millis;
  }
}
