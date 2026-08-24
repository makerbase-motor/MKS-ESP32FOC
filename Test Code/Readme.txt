!!! Important Information Before Use !!!

① The parameters in these example sketches are configured for the 2804 gimbal motor. If using a different motor, please adjust the following parameter values ​​accordingly:

BLDCMotor()    Pole pairs;
voltage_limit    Motor voltage limit    Recommended setting: 0.5–1.0 for RC motors; below 4.0 for gimbal motors;
current_limit    Motor current limit    This value should exceed the current loop parameter but remain below the motor's maximum safe operating current;
If a specific example sketch lacks input voltage sensing, you must also modify the "voltage_power_supply" value;
Exercise great caution with these parameters; incorrect values ​​are highly likely to damage the board!

② Open-loop control mode is intended for testing purposes; significant motor heating is inevitable—please be careful to avoid burns!
Do not run open-loop examples for more than one minute; doing so may cause overheating and damage the motor or circuit board!

③ The PID parameters in the closed-loop examples have been tested and are suitable for driving 2808 RC motors and 2804 gimbal motors.
For optimal control performance or when using different motors, please tune the PID parameters yourself.

④ The MKS ESP32 FOC V2.0 can be powered and programmed via USB; it is recommended to flash the firmware before connecting the main power supply.

⑤ MKS ESP32 FOC series boards come pre-flashed with a test program for the AS5600 encoder; please do not be alarmed if you notice this!
While the presence of signal activity on certain components and interfaces will not damage the board, we still recommend flashing your own firmware before connecting the main power supply for the first time.

Thank you for purchasing a Makerbase product!