#include <SimpleFOC.h>

// DEFINE CONSTANTS
double zero_offset = 0;
// double k[4] = {-0.1, 7.7067, -0.1213, 0.7476};
// double k[4] = {-0.0071, 0.5524, -0.0086, 0.0530}; // pretty good performance
// double k[4] = {-0.04, 0.5524, -0.0086, 0.0530}; // test for controlling motor angle better
// double k[4] = {-0.0030, 0.4900, -0.0035, 0.0465};
// double k[4] = {-0.0033, 0.5366, -0.0039, 0.0510};
double k[4] = {-0.0045, 0.4900, -0.0058, 0.0621};


//motor parameters
const double motor_Kv = 20;
const double motor_Kt = 60/(TWO_PI * motor_Kv);
const double motor_Ke = motor_Kt;
const double motor_R = 13.7;

static double filt_pend_vel = 0.0;
static double filt_motor_vel = 0.0;

// initialize motor and driver
BLDCMotor motor = BLDCMotor(11, motor_R, motor_Kv);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// E4T encoder
Encoder encoder = Encoder(2, 3, 720);

// AS5600 sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// interrupt functions for the encoder
void doA() {
  encoder.handleA();
}

void doB() {
  encoder.handleB();
}

double torque_to_voltage(double torque_cmd, double motor_vel) {
  // return (motor_R / motor_Kt) * torque_cmd + motor_Ke * motor_vel;
  return (motor_R / motor_Kt) * torque_cmd;
  // return torque_cmd * 30;

}


void setup() {
  Serial.begin(115200);

  // I2C for AS5600
  Wire.begin();

  // driver
  driver.voltage_power_supply = 18;
  driver.init();

  motor.linkDriver(&driver);

  // motor encoder
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);


  // sensor
  sensor.init();
  // get the zero offset for the sensor
  sensor.update();
  zero_offset = sensor.getAngle();

  // motor
  motor.init();
  motor.initFOC();

  // set motor voltage limit
  motor.voltage_limit = 8;

  // set torque control mode
  motor.torque_controller = TorqueControlType::estimated_current;
  // set motion control mode
  motor.controller = MotionControlType::torque;


  delay(1000);
}

unsigned long lastTime = 0;
const unsigned long sampleTime = 1000;
double last_pend_angle = 0.0;
double last_motor_angle = 0.0;

void loop() {
  // loop the FOC algorithm as fast as possible
  motor.loopFOC();

  unsigned long currentTime = micros();

  if (currentTime - lastTime >= sampleTime) {
    double dt = (currentTime - lastTime) / 1000000.0;

    lastTime = currentTime;

    // get the new sensor reading
    sensor.update();
    double current_angle = (sensor.getAngle() - zero_offset);
    // current_angle = fmod(current_angle + TWO_PI, TWO_PI);

    // get the new motor position
    double motor_angle = encoder.getAngle();

    // get motor velocity
    double motor_velocity = motor.shaft_velocity;
    // double motor_velocity = (motor_angle - last_motor_angle) /  dt;
    // last_motor_angle = motor_angle;

    // get pendulum velocity
    // double pendulum_velocity = sensor.getVelocity();

    double pendulum_velocity = (current_angle - last_pend_angle) / dt;
    last_pend_angle = current_angle;


    filt_pend_vel = 0.7 * filt_pend_vel + 0.3 * pendulum_velocity;
    // filt_motor_vel = 0.7 * filt_motor_vel + 0.3 * motor_velocity;

    double voltage_command = 0.0;
    double torque_command = 0.0;
  


    if (current_angle > 2.967 && current_angle < 3.316) {
      // if the pendulum is within the bounds, update the torque and command the motor
      torque_command = -1 * ((k[0] * motor_angle) + (k[1] * (current_angle - PI)) + (k[2] * motor_velocity) + (k[3] * filt_pend_vel));
      voltage_command = torque_to_voltage(torque_command, motor_velocity);

      // voltage_command = torque_command * 20;


      voltage_command = constrain(
        voltage_command,
        -motor.voltage_limit,
        motor.voltage_limit);

      // filt_v_cmd = 0.9 * filt_v_cmd + 0.1 * voltage_command;


      // uncomment to display states
      // double u1 = -k[0] * motor_angle;
      // double u2 = -k[1] * (current_angle - PI);
      // double u3 = -k[2] * motor_velocity;
      // double u4 = -k[3] * filt_pend_vel;
      // Serial.print(u1); Serial.print(",");
      // Serial.print(u2); Serial.print(",");
      // Serial.print(u3); Serial.print(",");
      // Serial.print(u4); Serial.print(",");


      // uncomment to display variables on the serial monitor with bounds
      // Serial.print(-10);
      // Serial.print(",");
      // Serial.print(10);
      // Serial.print(",");
      // Serial.println(voltage_command / 3);
      // Serial.print(",");
      // filt_v_cmd = constrain(filt_v_cmd, -motor.voltage_limit, motor.voltage_limit);
      // Serial.println(voltage_command);
  
      // motor.move(voltage_command);
      // voltage_command = 2*sin(2*PI*currentTime*0.000001);
      // motor.move(voltage_command);


      double error = PI - current_angle;
      // Serial.println(error * 50);
      motor.move(error * 50);

      
      // delay(300);
    } else {

      // set motor torque to zero if it falls outside the range
      motor.move(0);
      // voltage_command = 2*sin(2*PI*currentTime*0.000001);
      // motor.move(voltage_command);
    }
    
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(motor_angle);
    Serial.print(",");
    Serial.print(current_angle - PI);
    Serial.print(",");
    Serial.print(motor_velocity);
    Serial.print(",");
    Serial.print(filt_pend_vel);
    Serial.print(",");
    Serial.print(voltage_command);
    Serial.print(",");
    Serial.println(torque_command);


  }
}
