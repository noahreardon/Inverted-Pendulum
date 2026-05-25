#include <Wire.h>
#include <AS5600.h>
#include <AccelStepper.h>

const int stepPin = 3;
const int dirPin = 5;
const int analogPin = A1;
#define motorInterfaceType 1


// Initialize the stepper motor and encoder objects
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
AS5600 encoder;

float theta = 0;
float thetaDot = 0;


// Use this to store the offset of the original encoder measurement so that straigt down corresponds to 0 degrees
long zeroOffset = 0;

// Set the lower and upper bounds on the motor: if the pendulum is outside of [160, 200], it will just stop acting on commmands.
// There is no swing-up action currently, so this makes sense. If it drops, it would never get it back up.
float lowerGuard = 160;
float upperGuard = 200;

int maxD = 200; // clamp the derivative so that it doesn't go huge

// Variables needed for the PID control loop:
double dt, lastTime, integral, previous;
double derivativePrevious = 0;
double kp, ki, kd; // controller gains
double setpoint = 180; // the target point at which to balance the pendulum. This will always be 180, becuase that's the only place that the pendulum can balance.
int output; // the command to the stepper motor in number of steps
double dt_vel = 0.02;

float pid(double error, float currentAngle) { // function for calculating new motor command based on error
  double proportional = error;
  double derivative;
  integral = integral + (dt * error); // may need to add bounds/antiwindup, but probably not

  // if (currentAngle < 181 && currentAngle > 179) {  // deadband for small angles to reduce jitter near upright
  //   derivative = 0;
  //   integral = 0;
  // } else {
    // derivative = (error - previous) / dt;
    // derivative = derivative * 0.2 + derivativePrevious*(0.8);
    // derivative = constrain(derivative, -maxD, maxD);
    // derivativePrevious = derivative;
  // }
  
  
  previous = error;
  float output = kp*proportional + ki*integral + kd*thetaDot;
  return output;
}

void setup() {
  // PID values
  kp = 280.0; // 160 was GOOD (on 1/4 microstepping)
  ki = 0.0;
  kd = 20.0;
  lastTime = millis();


  Serial.begin(115200);
  Wire.begin();
  
  //setup for the ENCODER
  float currentRaw = encoder.readAngle();
  zeroOffset = currentRaw;

  // setup for the STEPPER MOTOR
  // Set maximum speed, acceleration, and initial speed
  myStepper.setMaxSpeed(6000.0);    // Steps per second
  myStepper.setSpeed(0);
  // myStepper.setAcceleration(1000.0); // Steps per second per second

}

void loop() {
  // start off by reading the encoder angle
  uint16_t raw = encoder.readAngle();       // 0–4095

  float calibratedRaw = (raw - zeroOffset + 4096) % 4096;
  float angleDeg = calibratedRaw * 360.0 / 4096.0;

  myStepper.runSpeed();

  if (angleDeg < lowerGuard || angleDeg > upperGuard) { // if the pendulum is outside of the set range, wait
    integral = 0;
    myStepper.setSpeed(0);
    myStepper.runSpeed();
  } else {

    double now = millis();
    static double slowLast = 0;
    static float prevAngleForVel = 180.0;
    if (now - slowLast >= 2) {
      dt_vel = (now - slowLast) / 1000.0;
      thetaDot = (angleDeg - prevAngleForVel) / dt_vel;
      prevAngleForVel = angleDeg;

      dt = (now - slowLast) / 1000.0;

      double error = (setpoint - angleDeg);
      myStepper.setSpeed(pid(error, angleDeg));

      slowLast = now;
    }

    // static double fastLast = 0;
    
    // if (now - fastLast >=2) {
    //   dt = (now - fastLast) / 1000.0;

    //   fastLast = now;
    //   double error = (setpoint - angleDeg);
    //   myStepper.setSpeed(pid(error, angleDeg));
    // }

    
    // otherwise, do everything else!
    // double now = millis();
    // if (now - lastTime < 2) return;
    // dt = (now - lastTime) / 1000.0;
    // lastTime = now;

    // double error = (setpoint - angleDeg);
    // float speedCommand = pid(error, angleDeg);
    // myStepper.setSpeed(speedCommand);
    
    
  }    

  

  // printing so we can see everything happening
  // Serial.print(160);      // Lower bound
  // Serial.print(" "); 
  // Serial.print(200);   // Upper bound
  // Serial.print(" ");
  // Serial.print(180);
  // Serial.print(" ");
  // Serial.print(a);
  // Serial.println(" ");

}

