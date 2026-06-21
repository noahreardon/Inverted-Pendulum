// Open loop motor control example
#include <SimpleFOC.h>

// BLDC motor & driver instance
// BLDCMotor( pp number , phase resistance, KV rating)
BLDCMotor motor = BLDCMotor(11 , 13.7, 20); 
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void doLimitCurrent(char* cmd) { command.scalar(&motor.current_limit, cmd); }

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;
  // torque control mode 
  motor.torque_controller = TorqueControlType::estimated_current;

  // setting target velocity
  motor.target = 25;  // [rad/s]
  // limiting motor current (provided resistance)
  motor.updateCurrentLimit(0.5);   // [Amps]
 
  // init motor hardware
  motor.init();
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('C', doLimitCurrent, "current limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  // torque control loop
  motor.loopFOC();
  // open loop velocity movement
  motor.move();

  // user communication
  command.run();
}

