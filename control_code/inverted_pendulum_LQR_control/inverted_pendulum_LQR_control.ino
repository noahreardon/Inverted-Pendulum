#include <SimpleFOC.h>

// DEFINE CONSTANTS
double zero_offset = 0;
// double k[4] = {-0.1, 7.7067, -0.1213, 0.7476};
double k[4] = {-0.0071, 0.5524, -0.0086, 0.0530};


    //motor parameters
const double motor_Kv = 20;
const double motor_Kt = 60/(TWO_PI * motor_Kv);
const double motor_Ke = motor_Kt;
const double motor_R = 13.7;

static double filt_v_cmd = 0.0;
static double filt_pend_vel = 0.0;

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
  return (motor_R / motor_Kt) * torque_cmd + motor_Ke * motor_vel;

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

  // set motor current limit
  motor.current_limit = 1.0;
  // set motor voltage limit
  motor.voltage_limit = 8.0;

  // set torque control mode
  motor.torque_controller = TorqueControlType::estimated_current;

  Serial.println("Motor ready");
  delay(1000);
}

unsigned long lastTime = 0;
const unsigned long sampleTime = 200;

void loop() {
  // loop the FOC algorithm as fast as possible
  motor.loopFOC();

  unsigned long currentTime = micros();

  if (currentTime - lastTime >= sampleTime) {
    lastTime = currentTime;

    // get the new sensor reading
    sensor.update();
    double current_angle = (sensor.getAngle() - zero_offset);
    current_angle = fmod(current_angle + TWO_PI, TWO_PI);

    // get the new motor position
    double motor_angle = encoder.getAngle();

    // get motor velocity
    double motor_velocity = motor.shaft_velocity;

    // get pendulum velocity
    double pendulum_velocity = sensor.getVelocity();

    filt_pend_vel = 0.9 * filt_pend_vel + 0.1 * pendulum_velocity;


    if (current_angle > 2.967 && current_angle < 3.316) {
      // if the pendulum is within the bounds, update the torque and command the motor
      double torque_command = -1 * ((k[0] * motor_angle) + (k[1] * (current_angle - PI)) + (k[2] * motor_velocity) + (k[3] * pendulum_velocity));
      motor.target = torque_command / motor_Kt;
      // Serial.println(torque_command / motor_Kt);
      motor.move();

    } else {

      // set motor torque to zero if it falls outside the range
      motor.target = 0;
      motor.move();
    }
    
  

  }
}
