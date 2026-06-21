#include <SimpleFOC.h>

// motor and driver
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);

// AS5600 sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  Serial.begin(115200);

  // I2C for AS5600
  Wire.begin();

  // driver
  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);

  // sensor
  sensor.init();
  motor.linkSensor(&sensor);

  // motor
  motor.init();
  motor.initFOC();

  Serial.println("Motor ready");
  delay(1000);
}

void loop() {
  sensor.update();

  Serial.print("Angle: ");
  Serial.println(sensor.getAngle(), 4);

  delay(100);
}
