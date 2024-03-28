#ifndef _STATE_H
#define _STATE_H

#include "motors.h"
#include "buzzer.h"
#include "line_sensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

Buzzer_c buzzer;           // Create instance of the Buzzer_c class
Motors_c motors;           // Create instance of the Motors_c class
Kinematics_c kinematics;   // Create instance of the Kinematics_c class
PID_c r_motor_PID;         // Create instance of the PID_c class for the right motor
PID_c l_motor_PID;         // Create instance of the PID_c class for the left motor
PID_c heading_PID;         // Create instance of the PID_c class for the heading

// Definition of states
#define STATE_DEBUG 0
#define STATE_INITIAL 1
#define STATE_DRIVE_FORWARD 2
#define TURN 3
#define STATE_FINISHED 4
#define STATE_FINAL 5
#define STATE_SQUARE 6
#define STATE_CALIBRATION 7
#define ON_CROSS 8
#define CALIBRATION_LINE 9
#define DISPLAY_RESULTS 10

int state; // Variable to store the current state

unsigned long timer;

// square stuff:
float SQUARE_SIZE = 200; // Size of the square in mm
int side_count = 0; // Count of the number of sides of the square
int num_squares = 1; // Number of squares to complete
int dir = 1;

// calibration stuff:
int straight_line_dist = 100; // Length of measured straight line section in mm
long e0_start = 0;
long e1_start = 0;
float new_r0;
float new_r1;

// DECIDE STATE:

// Class to track robot position and angle
class State_c {

  public:
    State_c() {

    }

    void integral_reset() {
      // Reset the integral sum of the PID controllers
      heading_PID.integral_sum = 0;
      r_motor_PID.integral_sum = 0;
      l_motor_PID.integral_sum = 0;
    }

    void update() {


      // Square-related states:
      if (state == STATE_SQUARE && (millis() - timer >= 1500)) {
        state = STATE_DRIVE_FORWARD;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 0 && (x_i >= SQUARE_SIZE)) {
        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 1 && (y_i >= SQUARE_SIZE)) {
        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 2 && (x_i <= 0)) {
        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 3 && (y_i <= 0)) {
        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == TURN && theta_i >= (((side_count + 1)*PI) / 2)) {
        buzzer.beep(1000, 200);
        state = STATE_DRIVE_FORWARD;
        integral_reset();
        side_count++;
      }

      else if (state == TURN && side_count >= (( 4 * num_squares) - 1)) {
        buzzer.beep(1000, 400);
        //        state = STATE_FINAL;
        state = DISPLAY_RESULTS;
      }



      // Calibration-related states
      else if (state == STATE_CALIBRATION && line[0] && line[4]) {

        state = ON_CROSS;

      }
      else if (state == ON_CROSS && !line[0] && !line[4]) {

        // TAKE COUNT MEASURE
        e0_start = count_e0;
        e1_start = count_e1;

        // change to folow line
        state = CALIBRATION_LINE;

      }
      else if (state == CALIBRATION_LINE && line[0] && line[4]) {

        // Update radius geometries
        new_r0 = kinematics.get_radius(0, straight_line_dist, e0_start);
        new_r1 = kinematics.get_radius(1, straight_line_dist, e1_start);

        // print result
        state = DISPLAY_RESULTS;

      }
    }

    void state_actions() {
      // STATE MACHINE:

      if (state == STATE_INITIAL || state == STATE_FINAL) {
        // Do nothing
        right_motor_demand =  0;
        left_motor_demand = 0;

      } else if (state == STATE_DRIVE_FORWARD) {

        float angle = side_count * PI / 2;
        motors.drive_along_angle(angle);

      } else if (state == TURN) {

        motors.rotate_on_spot(((side_count + 1)*PI) / 2);

        // right_motor_demand =  dir * TURN_ON_SPOT_SPEED;
        // left_motor_demand =  - dir * TURN_ON_SPOT_SPEED;

      } else if (state == STATE_CALIBRATION) {

        motors.follow_line();

      } else if (state == ON_CROSS) {

        motors.follow_line();

      } else if (state == CALIBRATION_LINE) {

        motors.follow_line();

      } else if (state == DISPLAY_RESULTS) {

        // STOP
        right_motor_demand =  0;
        left_motor_demand = 0;

        Serial.print("new_r0: ");
        Serial.print(new_r0);
        Serial.print(" ");
        Serial.print("new_r1: ");
        Serial.println(new_r1);


      } else if (state == STATE_DEBUG) {

        right_motor_demand =  TURN_ON_SPOT_SPEED;
        left_motor_demand = -TURN_ON_SPOT_SPEED;

      }
    }
};

#endif
