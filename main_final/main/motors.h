#ifndef _MOTORS_H
#define _MOTORS_H

#include "kinematics.h"
#include "line_sensor.h"

LineSensor_c line_sensors; // Create instance of the LineSensor_c class

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

// Motor speed paramters
float MOTOR_SPEED = 75;   // Deafult forwards motor speed
float MAX_MOTOR_STEP_SIZE = 4; // Maximum step size for ramping the motor speed
float TURN_SPEED = 20;    // Default turning speed
float TURN_ON_SPOT_SPEED = 15; // Default turning speed on the spot

// Kinematics
float heading_error;              // Value between 1 and -1 to represent the error of following the line
float heading_control;            // Control signal for the heading
float left_control = 0;           // Control signal for the left motor
float right_control = 0;          // Control signal for the right motor
float right_motor_demand;         // Demand for the right motor
float left_motor_demand;          // Demand for the left motor
float last_line_angle;
int intersection_count;

// Linesensor
unsigned long sensor_readings[5]; // Array to store the sensor readings
bool line[5];                     // Array to store if a line has been detected

// Class to operate the motors:
class Motors_c
{

  private:
    float last_left_pwm = 0;
    float last_right_pwm = 0;

  public:
    static const int MAX_PWM = 170;
    float current_speed = 0;

    void initialise()
    {
      // Set all the motor pins as outputs.
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

      // Set initial direction (HIGH/LOW) for the motor pins.
      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);

      // Set initial power values for the PWM Pins.
      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);
    }

    void set_motor(float left_pwm, float right_pwm, float max_delta_pwm)
    {

      // Limit the chagne in pwm to max_delta_pwm
      left_pwm = ramp_motor(last_left_pwm, left_pwm, max_delta_pwm);
      right_pwm = ramp_motor(last_right_pwm, right_pwm, max_delta_pwm);

      // Limit the input values to the max_pwm
      left_pwm = constrain(left_pwm, -MAX_PWM, MAX_PWM);
      right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

      // Set the direction based on the sign of the input
      digitalWrite(L_DIR_PIN, left_pwm < 0 ? REV : FWD);
      digitalWrite(R_DIR_PIN, right_pwm < 0 ? REV : FWD);

      // Set the power based on the absolute value of the input
      analogWrite(L_PWM_PIN, abs(left_pwm));
      analogWrite(R_PWM_PIN, abs(right_pwm));

      // Save the pwm values for the next iteration
      last_left_pwm = left_pwm;
      last_right_pwm = right_pwm;
    }

    float ramp_motor(float last_pwm, float demand_pwm, float max_delta_pwm) {
      float delta = demand_pwm - last_pwm;
      if (delta > max_delta_pwm) {
        demand_pwm = last_pwm + max_delta_pwm;
      }
      else if (delta < -max_delta_pwm) {
        demand_pwm = last_pwm - max_delta_pwm;
      }
      return demand_pwm;
    }

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
        right_motor_demand = 2 * TURN_SPEED * (-heading_error);
        left_motor_demand = 0;
      }
      else {
        right_motor_demand = 0;
        left_motor_demand = 2 * TURN_SPEED * heading_error;
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

//    void join_line() {
//
//      if (!line_detected) {
//        drive_along_angle(0); // Drive forwards
//      }
//
//      if (!line_detected && line[1] && line[2] && line[3]) {
//        last_line_detected = millis();
//        line_detected = true;
//      }
//
//      if (line_detected) {
//        drive_along_angle(0);
//        if (millis() - last_line_detected >= REJOIN_LINE_TIME) {
//          rotate_on_spot(PI);
//        }
//      }
//    }

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



//    void intersection() {
//      if (intersection_count > 3) {
//        turn_right = false;
//      }
//
//      if (millis() - intersection_ts < 10) {
//        drive_along_angle(last_line_angle);
//      }
//
//      else {
//        if (turn_right) {
//          left_motor_demand = TURN_SPEED;
//          right_motor_demand = -TURN_SPEED;
//        }
//        else {
//          left_motor_demand = -TURN_SPEED;
//          right_motor_demand = TURN_SPEED;
//        }
//      }
//    }
//
//    void crossroads() {
//
//      if (millis() - intersection_ts < 500) {
//        drive_along_angle(last_line_angle);
//      } else {
//        rotate_on_spot(-PI / 2);
//      }
//    }






};

#endif
