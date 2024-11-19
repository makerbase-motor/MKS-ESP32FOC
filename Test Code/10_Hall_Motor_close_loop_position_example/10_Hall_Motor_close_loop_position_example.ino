// MKS ESP32 FOC V2.0 | Hall Motor Close Loop Position Example | Library:SimpleFOC 2.2.1 | Hardware:MKS ESP32 FOC V2.0

// !!!Notice!!!
//①Enter T+position in the serial port window to make the two motors rotate in a closed loop. For example, if you want both motors to rotate 180°, enter the radian system: T3.14
//②When using your own motor, please remember to modify the default pole pair number, that is, the value in BLDCMotor() and HallSensor(), and set it to your own pole pair number.
//③The default power supply voltage of the program is 24V. If you use other voltages, please remember to modify the values ​​in the voltage_power_supply and voltage_limit variables.
//④If you want to achieve better results or use other motors, please adjust the PID parameters yourself

#include <SimpleFOC.h>
//18——Corresponding to the pin SCL_0
//19——Corresponding to the pin SDA_0
//15——Corresponding to the pin I_0
//1——Pole pairs
HallSensor sensor = HallSensor(18, 19, 15, 1);// U V W Pole pairs
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
//5——Corresponding to the pin SCL_0
//23——Corresponding to the pin SDA_0
//13——Corresponding to the pin I_0
//1——Pole pairs
HallSensor sensor1 = HallSensor(5, 23, 13, 1); // U V W Pole pairs
void doA1(){sensor1.handleA();}
void doB1(){sensor1.handleB();}
void doC1(){sensor1.handleC();}

//Motor parameters Set the number of pole pairs according to the motor
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(1);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 21);

//Command settings
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();
  sensor1.init();
  sensor.enableInterrupts(doA, doB, doC);
  sensor1.enableInterrupts(doA1, doB1, doC1);

  
  //Connect the motor object and the sensor object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply voltage setting [V]
  driver.voltage_power_supply = 24;
  driver.init();

  driver1.voltage_power_supply = 24;
  driver1.init();
  //Connect the motor and driver objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  
  //Motion control mode settings
  motor.controller = MotionControlType::angle;
  motor1.controller = MotionControlType::angle;

  //Speed ​​PI loop settings
  motor.PID_velocity.P = 0.005;
  motor1.PID_velocity.P = 0.005;
  motor.PID_velocity.I = 0.1;
  motor1.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;
  //Angle P loop setting 
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //Maximum motor limiting voltage  
  motor.voltage_limit = 6;
  motor1.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;
  motor1.PID_velocity.output_ramp = 1000;
  
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01f;
  motor1.LPF_velocity.Tf = 0.01f;

  //设置最大速度限制
  motor.velocity_limit = 45;
  motor1.velocity_limit = 45;

  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //初始化电机
  motor.init();
  motor1.init();
  //初始化 FOC
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
//  sensor.update();
//  sensor1.update();

//  Serial.print(sensor1.getAngle());
//  Serial.print("\t");
//  Serial.println(sensor1.getVelocity());
}
