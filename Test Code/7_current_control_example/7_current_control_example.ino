
/**
MKS ESP32 FOC; FOC电流控制例程 测试库：SimpleFOC 2.1.1 测试硬件：MKS ESP32 FOC V1.0
在串口窗口中输入：A+电流控制M0，B+电流控制M1，电流单位为A
可更改或用注释取消代码中设置的电压限制与电流限制
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是YT2804 ，使用自己的电机需要修改PID参数，才能实现更好效果
*/

#include <SimpleFOC.h>

//电机实例
BLDCMotor motor1 = BLDCMotor(7);                        //根据选用电机的极对数修改此处的BLCDMotor()中的值
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);

BLDCMotor motor2 = BLDCMotor(7);                        //同样修改此处BLCDMotor()中的值
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

//编码器实例
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1);


// 在线电流检测实例
InlineCurrentSense current_sense1 = InlineCurrentSense(0.01, 50.0, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01, 50.0, 35, 34);

// commander通信实例
Commander command = Commander(Serial);
void doMotor1(char* cmd){ command.motor(&motor1, cmd); }
void doMotor2(char* cmd){ command.motor(&motor2, cmd); }

void setup() {
  // 编码器设置
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  

  // 驱动器设置
  driver1.voltage_power_supply = 12;            //根据供电电压，修改此处voltage_power_supply的值
  driver1.init();
  motor1.linkDriver(&driver1);
  driver2.voltage_power_supply = 12;            //同样修改此处voltage_power_supply的值
  driver2.init();
  motor2.linkDriver(&driver2);

  // 电流限制
    motor1.current_limit = 2;         //根据情况修改电压电流限制，也可注释取消此处代码
    motor2.current_limit = 2;
  // 电压限制
    motor1.voltage_limit = 12;        //根据供电电压，修改此处voltage_limit的值
    motor2.voltage_limit = 12;


  // 电流检测
  current_sense1.init();
  current_sense1.gain_b *= -1;
  current_sense1.gain_a *= -1;
//  current_sense1.skip_align = true;
  motor1.linkCurrentSense(&current_sense1);
  // current sense init and linking
  current_sense2.init();
  current_sense2.gain_b *= -1;
  current_sense2.gain_a *= -1;
//  current_sense2.skip_align = true;
  motor2.linkCurrentSense(&current_sense2);

  // 控制环
  // 其他模式 TorqueControlType::voltage TorqueControlType::dc_current 
  motor1.torque_controller = TorqueControlType::foc_current; 
  motor1.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::foc_current; 
  motor2.controller = MotionControlType::torque;

  // FOC电流控制PID参数
   motor1.PID_current_q.P = 2;                      //修改合适的PID参数以实现更好的效果
   motor1.PID_current_q.I= 800;                     //电机出现抖动、转速不稳定的情况，很有可能就是PID参数没调到合适的
   motor1.PID_current_d.P= 2;
   motor1.PID_current_d.I = 800;
   motor1.LPF_current_q.Tf = 0.002; // 1ms default
   motor1.LPF_current_d.Tf = 0.002; // 1ms default

   motor2.PID_current_q.P = 2;
   motor2.PID_current_q.I= 800;
   motor2.PID_current_d.P= 2;
   motor2.PID_current_d.I = 800;
   motor2.LPF_current_q.Tf = 0.002; // 1ms default
   motor2.LPF_current_d.Tf = 0.002; // 1ms default

    // 速度环PID参数
    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;
    // default voltage_power_supply
  
    // 速度限制
    motor1.velocity_limit = 40;
    motor2.velocity_limit = 40;


  // monitor接口设置
  Serial.begin(115200);
  // comment out if not needed
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // monitor相关设置
  motor1.monitor_downsample = 0;
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_downsample = 0;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  


  //电机初始化
  motor1.init();
  // align encoder and start FOC
  motor1.initFOC(); 
  
  motor2.init();
  // align encoder and start FOC
  motor2.initFOC(); 

  // 初始目标值
  motor1.target = 0.05;
  motor2.target = 0.05;

  // 映射电机到commander
  command.add('A', doMotor1, "motor 1");
  command.add('B', doMotor2, "motor 2");

  Serial.println(F("Double motor sketch ready."));
  
  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // iterative function setting the outter loop target
  motor1.move();
  motor2.move();

  // user communication
  command.run();
  motor1.monitor();
  motor2.monitor();
}
