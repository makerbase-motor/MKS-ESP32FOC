/**
MKS ESP32 FOC 电机旋钮例程 测试库：SimpleFOC 2.1.1 测试硬件:MKS ESP32 FOC
上电后转动其中一个电机，可以对另一个电机调速
在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
程序默认设置的供电电压为 12V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
默认PID针对的电机是YT2804，使用的编码器是AS5600，使用自己的电机需要修改PID参数，才能实现更好效果
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// 电机详情
BLDCMotor motor = BLDCMotor(7);                         //根据选用电机的极对数修改此处BLDCMotor()中的值
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(7);                        //同样修改此处BLDCMotor()中的值
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

//定义 TROT 步态变量
void setup() {
  I2Cone.begin(19, 18, 400000); 
  I2Ctwo.begin(23, 5, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  // 驱动配置
  // 供电电压 [V]
  driver.voltage_power_supply = 12;               //需根据供电电压修改此处voltage_power_supply的值
  driver.init();

  driver1.voltage_power_supply = 12;              //同样修改此处voltage_power_supply的值
  driver1.init();
  // 连接驱动
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  // 控制环模式设置
  motor.controller = MotionControlType::torque;
  motor1.controller = MotionControlType::velocity;

  // 电机电压限制
  motor.voltage_limit = 12;                 //根据供电电压修改此处voltage_limit的值
  motor1.voltage_limit = 12;                //同样修改此处voltage_limit的值
  
  motor1.LPF_velocity.Tf = 0.01;            //根据选用电机修改这两行参数以实现更好的效果
  motor1.PID_velocity.I = 1;

  // 串口设置
  Serial.begin(115200);
  // 不需要以下两行可以注释取消掉
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);
  //记录无刷初始位置

  
  //初始化电机
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready.");
  _delay(1000);
  
}

void loop() {

  motor.loopFOC();
  motor1.loopFOC();

  motor.move(5*(motor1.shaft_velocity/10 - motor.shaft_angle));
  motor1.move(10*dead_zone(motor.shaft_angle));
}

float dead_zone(float x){
  return abs(x) < 0.2 ? 0 : x;
}
