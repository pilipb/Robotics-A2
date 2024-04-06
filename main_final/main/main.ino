#include "motors.h"
#include "buzzer.h"
#include "line_sensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "state.h"

State_c State;     // Create instance of the State_c class

// TIME INTERVALS:
#define LINE_SENSOR_UPDATE 15
#define KINEMATIC_UPDATE 10
#define PID_UPDATE 25
#define MOTOR_UPDATE 30
#define PAUSE 1000
#define STATE_UPDATE 50

// GLOBAL VARIABLES:
// TIMESTAMP VARIABLES:
unsigned long line_sensor_ts;
unsigned long PID_ts;
unsigned long kinematic_ts;
unsigned long motor_ts;
unsigned long ramp_ts;
unsigned long state_ts;


void setup() {

  kinematics.initialise(); // Initialise the kinematic model

  // Initialise PID controllers
  r_motor_PID.initialise(0.40, 0.7, 0.4);
  l_motor_PID.initialise(0.40, 0.7, 0.4);
  heading_PID.initialise(22, 0.15, 0.2);

  line_sensors.initialise_digital(); // Initialise the line sensors
  motors.initialise(); // Initialise the motors
  buzzer.initialise(); // Initialise the buzzer

  setupEncoder0(); // Right encoder
  setupEncoder1(); // Left encoder

  Serial.begin(9600); // Start the serial communication

  // Set the initial timestamp
  line_sensor_ts = millis();
  PID_ts = millis();
  kinematic_ts = millis();
  motor_ts = millis();
  ramp_ts = millis();
  state_ts = millis();

  // Set system state to intial
  state = STATE_SQUARE;  // STATE_SQUARE, STATE_LINE for testing, LINE_CALIBRATION for calibration
  timer = millis();
}


void loop() {

  // Save the current timestamp
  unsigned long current_ts;
  current_ts = millis();
 

  // LINE SENSOR UPDATE
  if ((current_ts - line_sensor_ts) >= LINE_SENSOR_UPDATE) {

    line_sensor_ts = millis(); // record the current timestamp
    line_sensors.read_all_sensors_digital(sensor_readings); // Read the sensor values
    line_sensors.line_threshold(sensor_readings, line); // Decide if a line is below each sensor
    
  }

  // KINEMATIC UPDATE
  if ((current_ts - kinematic_ts) >= KINEMATIC_UPDATE) {

    kinematic_ts = current_ts; // Update the timestamp
    kinematics.update(); // Update the kinematic model
    
  }

  // PID UPDATE
  if ((current_ts - PID_ts) >= PID_UPDATE) {

    PID_ts = millis(); // Update the timestamp

    // Calculate the rotation speed of the motors
    r_motor_PID.calculate_rotation_speed();
    heading_control = heading_PID.update(0, heading_error); // Update the heading control signal

    // Update the motor control using the PID
    right_control = r_motor_PID.update(right_motor_demand, rotation_speed_r_smoothed);
    left_control = l_motor_PID.update(left_motor_demand, rotation_speed_l_smoothed);
    
  }

  // MOTOR UPDATE
  if ((current_ts - motor_ts) >= MOTOR_UPDATE) {

    motor_ts = millis(); // Update the timestamp
    motors.set_motor(left_control, right_control, MAX_MOTOR_STEP_SIZE); // Set the motor speeds
    
  }

  float side_theta = side_count * PI / 2;

  // UPDATE STATE MACHINE
  if ((current_ts - state_ts ) >= STATE_UPDATE) {

    state_ts = millis(); // Update the timestamp
    State.update();
    State.state_actions();
    
  }

};
