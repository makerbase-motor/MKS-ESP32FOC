//MKS ESP32 FOC Mega Hall Close Position Contorl Example Library: SimpleFOC 2.2.1 Hardware: MKS ESP32 FOC V2.0

// ！！！Cautions！！！
//① Enter "T + position" in the serial monitor to drive both motors in closed-loop mode; for example, to rotate both motors by 180°, enter the value in radians: T3.14
//② When using your own motors, be sure to update the default pole pair count—specifically the values ​​within BLDCMotor() and HallSensor()—to match your motor's specifications
//③ The default supply voltage is set to 24V; if using a different voltage, remember to update the values ​​for the voltage_power_supply and voltage_limit variables
//④ To achieve better performance or when using different motors, please adjust the PID parameters accordingly

#include <SimpleFOC.h>
//18——SCL_0
//19——SDA_0
//15——I_0
//1——Pole Pairs
HallSensor sensor = HallSensor(18, 19, 15, 1);// U V W Pole Paris
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

//Motor Parameters: Configure the number of pole pairs based on the motor settings.
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);


//Command Settings
float target_velocity = 0;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  
  //Connect the motor object and the sensor object.
  motor.linkSensor(&sensor);

  //Supply voltage setting [V]
  driver.voltage_power_supply = 24;
  driver.init();

  //Connect the motor and driver objects.
  motor.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  
  //Motion Control Mode Settings
  motor.controller = MotionControlType::angle;

  //Speed ​​PI Loop Settings
  motor.PID_velocity.P = 0.005;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0;
  //Angle P-loop setting
  motor.P_angle.P = 20;
  //Maximum motor voltage limit  
  motor.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;
  
  //Speed ​​low-pass filter time constant
  motor.LPF_velocity.Tf = 0.01f;

  //Set maximum speed limit
  motor.velocity_limit = 45;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  
  //Initialize the motor
  motor.init();
  //Initialize FOC
  motor.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}



void loop() {
  motor.loopFOC();

  motor.move(target_velocity);

  command.run();
//  sensor.update();
//  sensor1.update();

//  Serial.print(sensor1.getAngle());
//  Serial.print("\t");
//  Serial.println(sensor1.getVelocity());
}
