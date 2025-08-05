/**
 * ESP32 position motion control example with magnetic sensor
 */
#include <SimpleFOC.h>
#include "SPI.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#include "SerialPlotter.h"

#define PIN_A_HI GPIO_NUM_12
#define PIN_A_LO GPIO_NUM_13
#define PIN_B_HI GPIO_NUM_27
#define PIN_B_LO GPIO_NUM_14
#define PIN_C_HI GPIO_NUM_25
#define PIN_C_LO GPIO_NUM_26

SerialPlotter<2> plotter;

#define SENSOR1_CS GPIO_NUM_5
MagneticSensorMT6701SSI sensor(SENSOR1_CS);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_A_HI, PIN_A_LO, PIN_B_HI, PIN_B_LO, PIN_C_HI, PIN_C_LO);

void setup()
{

  // use monitoring with serial
  Serial.begin(115200);
  Serial.setTimeout(5);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // init the sensor
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 22.2;
  driver.voltage_limit = 22.2f / 2;
  motor.velocity_limit = 5;
  motor.voltage_limit = 1.0f;

  motor.P_angle.P = 20;

  motor.PID_velocity.P = 0.0f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;
  // driver.pwm_frequency = 7000;
  // driver.dead_zone = 0.1f;
  // init the driver
  if (!driver.init())
  {
    Serial.println("Driver init failed!");
    return;
  }
  // link driver
  motor.linkDriver(&driver);

  motor.LPF_velocity.Tf = 0.005f;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.P_angle.limit = 100.0f;
  motor.PID_velocity.limit = 100.0f;

  // use monitoring with serial
  // comment out if not needed
  motor.useMonitoring(Serial);

  auto &plot1 = plotter.add_plot<2, float>("P");
  plot1.attach_parameter("angle", []()
                         { return sensor.getAngle(); });
  plot1.attach_parameter("vel", []()
                         { return sensor.getVelocity(); });

  auto &plot2 = plotter.add_plot<2, float>("K");
  plot2.attach_parameter("P", &motor.PID_velocity.P);
  plot2.attach_parameter("I", &motor.PID_velocity.I);

  // initialize motor

  // set the initial motor target
  motor.target = 0;

  if (!motor.init())
  {
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  if (!motor.initFOC())
  {
    Serial.println("FOC init failed!");
    return;
  }

  Serial.println(F("Motor ready."));
  _delay(1000);
}

void loop()
{
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  if (Serial.available() > 1)
  {
    char key = Serial.read();
    float value = Serial.parseFloat();

    switch (key)
    {
    case 'P':
      motor.PID_velocity.P = value;
      Serial.println(value);
      break;

    case 'I':
      motor.PID_velocity.I = value;
      Serial.println(value);
      break;

    case 'T':
      motor.target = value;
      Serial.println(value);
      break;

    case 'L':
      motor.voltage_limit = value;
      motor.PID_velocity.limit = value;
      Serial.println(value);
      break;

    case 'V':
      motor.velocity_limit = value;
      motor.P_angle.limit = value;
      Serial.println(value);
      break;

    case 'R':
      motor.PID_velocity.output_ramp = value;
      Serial.println(value);
      break;

    case 'A':
      motor.P_angle.P = value;
      Serial.println(value);
      break;

    case 'E':
      motor.feed_forward_velocity = value;
      Serial.println(value);
      break;

    case 'O':
      motor.current_sp = value;
      Serial.println(value);
      break;

    default:
      break;
    }
  }

  plotter.plot();
}