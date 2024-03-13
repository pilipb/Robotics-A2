#include "motors.h"
#include "buzzer.h"
#include "line_sensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

Buzzer_c buzzer;           // Create instance of the Buzzer_c class
Motors_c motors;           // Create instance of the Motors_c class
LineSensor_c line_sensors; // Create instance of the LineSensor_c class
Kinematics_c kinematics;   // Create instance of the Kinematics_c class
PID_c r_motor_PID;         // Create instance of the PID_c class for the right motor
PID_c l_motor_PID;         // Create instance of the PID_c class for the left motor
PID_c heading_PID;         // Create instance of the PID_c class for the heading

// TIME INTERVALS:
#define LINE_SENSOR_UPDATE 15
#define KINEMATIC_UPDATE 10
#define PID_UPDATE 25
#define MOTOR_UPDATE 30
#define PAUSE 1000

// GLOBAL VARIABLES:
// TIMESTAMP VARIABLES:
unsigned long timer;
unsigned long line_sensor_ts;
unsigned long PID_ts;
unsigned long kinematic_ts;
unsigned long motor_ts;
unsigned long ramp_ts;

// Line sensors
unsigned long sensor_readings[5]; // Array to store the sensor readings
bool line[5];                     // Array to store if a line has been detected

// Kinematics
float heading_error;              // Value between 1 and -1 to represent the error of following the line
float heading_control;            // Control signal for the heading
float left_control = 0;           // Control signal for the left motor
float right_control = 0;          // Control signal for the right motor
float right_motor_demand;         // Demand for the right motor
float left_motor_demand;          // Demand for the left motor

// Motor speed paramters
float MOTOR_SPEED = 75;   // Deafult forwards motor speed
float MAX_MOTOR_STEP_SIZE = 5; // Maximum step size for ramping the motor speed
float TURN_SPEED = 20;    // Default turning speed
float TURN_ON_SPOT_SPEED = 10; // Default turning speed on the spot

// Definition of states
#define STATE_INITIAL 0
#define STATE_DRIVE_FORWARD 1
#define TURN 2
#define STATE_FINISHED 3
#define STATE_DEBUG 4
int state; // Variable to store the current state

// square stuff:
float SQUARE_SIZE = 200; // Size of the square in mm
int side_count = 0; // Count of the number of sides of the square
float side

void drive_along_angle(float angle) {

    heading_error = theta_i - angle; // Calculate the heading error

    // Set the motor demand accordingly
    right_motor_demand = MOTOR_SPEED + heading_control;
    left_motor_demand = MOTOR_SPEED - heading_control;
}

void rotate_on_spot(float angle) {

    heading_error = theta_i - angle; // Calculate the heading error

    // Set the motor demand accordingly
    right_motor_demand =  heading_error * (-TURN_ON_SPOT_SPEED);
    left_motor_demand = heading_error * TURN_ON_SPOT_SPEED;
}

void rotate_on_spot_relative(float angle) {

    float target_angle = theta_i + angle;

    heading_error = theta_i - target_angle; // Calculate the heading error

    // Set the motor demand accordingly
    right_motor_demand =  TURN_SPEED * (-heading_error);
    left_motor_demand = TURN_SPEED * heading_error;
}

void rotate_around_wheel(float angle) {

    heading_error = theta_i - angle; // Calculate the heading error

    // Set the motor demand accordingly
    if (heading_error < 0) {
        right_motor_demand = 2*TURN_SPEED * (-heading_error);
        left_motor_demand = 0;
    }
    else {
      right_motor_demand = 0;
      left_motor_demand = 2*TURN_SPEED * heading_error;
    }
}

void join_line() {
  
  if (!line_detected) {
    drive_along_angle(0); // Drive forwards
  }

  if (!line_detected && line[1] && line[2] && line[3]) {
    last_line_detected = millis();
    line_detected = true;
  }

  if (line_detected) {
    drive_along_angle(0);
    if (millis() - last_line_detected >= REJOIN_LINE_TIME) {
      rotate_on_spot(PI);
    }
  }
}

void follow_line() {

  // Calculate the heading error
  heading_error = line_sensors.normalise_sensor_readings(sensor_readings);

  if (line[0] && !line[4]) {
    heading_error = 0.8;
  }
  else if (!line[0] && line[4]) {
    heading_error = -0.8;
  }

  // Set the motor demand accordingly
  left_motor_demand = MOTOR_SPEED + heading_control;
  right_motor_demand = MOTOR_SPEED - heading_control;

  // Save the global angle of the line
  last_line_angle = theta_i;
}

void rejoin_line() {

  left_motor_demand = TURN_SPEED;
  right_motor_demand = -TURN_SPEED;
}

void integral_reset() {
    // Reset the integral sum of the PID controllers
    heading_PID.integral_sum = 0;
    r_motor_PID.integral_sum = 0;
    l_motor_PID.integral_sum = 0;
}

void intersection() {
  if (intersection_count > 3) {
    turn_right = false;
  }

  if (millis() - intersection_ts < 10) {
    drive_along_angle(last_line_angle);
  }

  else {
    if (turn_right) {
      left_motor_demand = TURN_SPEED;
      right_motor_demand = -TURN_SPEED;
    }
    else {
      left_motor_demand = -TURN_SPEED;
      right_motor_demand = TURN_SPEED;
    }
  }
}

void crossroads() {
    
  if (millis() - intersection_ts < 500) {
    drive_along_angle(last_line_angle);
  } else {
    rotate_on_spot(-PI/2);
  }
}


float angle_to_home() {
  float angle = atan2(-y_i, -x_i);
  return angle;
}

void face_home() {
  float angle = angle_to_home();
  rotate_on_spot(angle);
}

void return_home() {
    drive_along_angle(angle_to_home());
}


void setup() {

  // Initialise PID controllers
  r_motor_PID.initialise(0.60, 0.7, 0.4);
  l_motor_PID.initialise(0.50, 0.7, 0.4);
  heading_PID.initialise(20, 0.1, 0.2);

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

  // Set system state to intial
  state = STATE_INITIAL;  // STATE_INITIAL STATE_FOLLOW_LINE STATE_DEBUG
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

  side_theta = side_count * PI/2;

  // DECIDE STATE:
  if (state == STATE_INITIAL && (millis() - timer >= 1500)) {
    state = STATE_DRIVE_FORWARD;
  } 

  else if (state == STATE_DRIVE_FORWARD && side_count == 0 && (x_i >= SQUARE_SIZE)) {
    buzzer.beep(1000,200);
    state = TURN;
  }

  else if (state == TURN && theta_i <= ((side_count+1)*PI/2)) {
    buzzer.beep(1000,200);
    state = STATE_DRIVE_FORWARD;
    side_count++;
    }

  else if (side_count > 3) {
    state = STATE_INITIAL
  }  

  // STATE MACHINE:
  if (state == STATE_INITIAL) {
      // Do nothing

  } else if (state == STATE_DRIVE_FORWARD) {
    float angle = side_count * PI/2;
    drive_along_angle(angle);

  } else if (state == TURN) {

    right_motor_demand =  TURN_ON_SPOT_SPEED;
    left_motor_demand = -TURN_ON_SPOT_SPEED;

  } else if (state == STATE_FOLLOW_LINE) {

    follow_line();

  } else if (state == STATE_REJOIN_LINE) {

    rejoin_line();

  } else if (state == STATE_INTERSECTION) {
      
    intersection();

  } else if (state == STATE_CROSSROADS) {
        
      crossroads();

  } else if (state == STATE_FACE_HOME) {
      
      face_home();

  } else if (state == STATE_RTH) {
      
    return_home();

  } else if (state == STATE_FINISHED) {

    // Stop the motors
    motors.set_motor(0, 0, MAX_MOTOR_STEP_SIZE);
    if (millis() - timer <= 1500) {
      buzzer.beep(1000,10);
    }

  } else if (state == STATE_DEBUG) {



    // rotate_on_spot(PI);
    // drive_along_angle(0);
// MOTOR CONTROL:
    // Serial.print("left:");
    // Serial.print(left_control);
    // Serial.print(" ");
    // Serial.print("right:");
    // Serial.println(right_control);

    // Serial.print("demand:");
    // Serial.print(right_motor_demand);
    // Serial.print(" ");
    // Serial.print("rotation_speed_r:");
    // Serial.println(rotation_speed_r_smoothed);

    Serial.print("rotation_speed_r:");
    Serial.print(rotation_speed_r_smoothed);
    Serial.print(" ");
    Serial.print("rotation_speed_l:");
    Serial.println(rotation_speed_l_smoothed);

// KINEMATICS:
  //   Serial.print(x_i);
  //   Serial.print(" ");
  //   Serial.print(y_i);
  //   Serial.print(" ");
  //   Serial.println(theta_i);
 }

};

