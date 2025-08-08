/**
 * ESP32 position motion control example with magnetic sensor
 */
#include <Arduino.h>
#include <SimpleFOC.h>
#include "SPI.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"
#ifdef PLATFORM_ESP32
#include "SerialPlotter.h"
#endif

#ifdef PLATFORM_STM32
#define PIN_A_HI A3
#define PIN_A_LO A4
#define PIN_B_HI A7
#define PIN_B_LO A2
#define PIN_C_HI A5
#define PIN_C_LO A6
#else
#define PIN_A_HI GPIO_NUM_12
#define PIN_A_LO GPIO_NUM_13
#define PIN_B_HI GPIO_NUM_27
#define PIN_B_LO GPIO_NUM_14
#define PIN_C_HI GPIO_NUM_25
#define PIN_C_LO GPIO_NUM_26
#endif

#ifdef PLATFORM_ESP32
// SerialPlotter<2> plotter;
#endif

#ifdef PLATFORM_STM32
#define SENSOR1_CS PA15
#else 
#define SENSOR1_CS GPIO_NUM_5
#endif
MagneticSensorMT6701SSI sensor(SENSOR1_CS);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_A_HI, PIN_A_LO, PIN_B_HI, PIN_B_LO, PIN_C_HI, PIN_C_LO);

InlineCurrentSense current_sense = InlineCurrentSense(40.0f, 39, NOT_SET, 36);

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

  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 12.0f / 2;
  motor.velocity_limit = 5;
  motor.voltage_limit = 1.0f;

  motor.P_angle.P = 20;

  motor.PID_velocity.P = 0.0f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  motor.PID_current_q.P = 0.5;
  motor.PID_current_q.I= 300;
  motor.LPF_current_q.Tf = 0.01; 
	// Low pass filtering time constant
	motor.LPF_current_d.Tf = 0.01;
  motor.PID_current_q.limit = motor.voltage_limit; 

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

  // motor.voltage_sensor_align = 2.0f;

  current_sense.linkDriver(&driver);

  // current sense init hardware
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  motor.LPF_velocity.Tf = 0.005f;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle;

  motor.P_angle.limit = 100.0f;
  motor.PID_velocity.limit = 100.0f;

  // use monitoring with serial
  // comment out if not needed
  motor.useMonitoring(Serial);

  #ifdef PLATFORM_ESP32
  auto &plot1 = plotter.add_plot<1, float>("P");
  // plot1.attach_parameter("d", []()
  //                        { return (float)analogRead(39); });
  // plot1.attach_parameter("q", []()
  //                        { return (float)analogRead(36); });
  plot1.attach_parameter("pos", []() {return sensor.getAngle();});

  auto &plot2 = plotter.add_plot<2, float>("K");
  plot2.attach_parameter("P", &motor.PID_velocity.P);
  plot2.attach_parameter("I", &motor.PID_velocity.I);
  #endif

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
  Serial.println("pizda");
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

    case 'C':
      motor.current_limit = value;
      Serial.println(value);
      break;

    default:
      break;
    }
  }
  #ifdef PLATFORM_ESP32
  plotter.plot();
  #endif
}