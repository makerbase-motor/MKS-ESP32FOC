// MKS ESP32 FOC Mega Bluetooth Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC PLUS & MKS AS5600

// ！！！Cautions！！！
// ① Enter "M" followed by a number via the serial port to command the M0 motor to rotate to a specific position; for example, entering "M3.14" moves the M0 motor to the 180° position (in radians).
// ② When using your own motor, be sure to update the default pole pair count—specifically the value within `BLDCMotor(7)`—to match your motor's actual pole pair count.
// ③ Configure the `voltage_limit` and `current_limit` values ​​according to your specific motor; recommended settings are 0.5–1.0 for RC aircraft motors and below 4.0 for gimbal motors. Excessive voltage or current may damage the driver board!
// ④ The PID parameters in this example are tuned for the 2808 RC aircraft motor; please adjust the PID parameters yourself if you wish to achieve better performance or are using a different motor.

#include <SimpleFOC.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,12);

// encoder instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);


// inline current sensor instance
// check if your board has R010 (0.01 ohm resistor) or R006 (0.006 mOhm resistor)
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 39, 36);

// commander communication instance
Commander command = Commander(SerialBT);
void doMotor(char* cmd){ command.motor(&motor, cmd); }

//I2C initialization
void I2C_init();

void setup() {
  I2C_init();
  // driver config
  // power supply voltage [V]
  //driver.pwm_frequency = 100000;
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // TorqueControlType::foc_current
  // Other Modes TorqueControlType::voltage TorqueControlType::dc_current 
  motor.torque_controller = TorqueControlType::foc_current; 
  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration based on the controll type 
  motor.PID_velocity.P = 0.021;
  motor.PID_velocity.I = 0.12;
  motor.PID_velocity.D = 0;
  
  // voltage limit
  motor.voltage_limit = 0.5;
  // Current limit
  //motor.current_limit = 0.5;
  
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle loop controller
  motor.P_angle.P = 20;
  // angle loop velocity limit
  motor.velocity_limit = 20;

  // use monitoring with serial for motor init
  // monitoring port
  //Serial.begin(115200);
  SerialBT.begin("MKS ESP32 FOC"); //Bluetooth device name
  // comment out if not needed
  motor.useMonitoring(SerialBT);
  motor.monitor_downsample = 0; // disable intially
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle
  
  // current sense init and linking
  current_sense.init();
  current_sense.gain_b *= 1;
  current_sense.gain_a *= 1;
  motor.linkCurrentSense(&current_sense);

  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC(); 

  // set the inital target value
  motor.target = 0.05;

  // subscribe motor to the commander
  command.add('M', doMotor, "motor");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)  
  SerialBT.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));
  
  _delay(1000);
}


void loop() {
  //SerialBT.println(F("111"));
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  motor.move();

  // motor monitoring
  motor.monitor();

  // user communication
  command.run();
}



void I2C_init(){
  pinMode(32, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);

  I2Cone.begin(19, 18, 400000UL); // AS5600_M0
  sensor.init(&I2Cone);
  motor.linkSensor(&sensor);
}
