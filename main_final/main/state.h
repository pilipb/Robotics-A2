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
#define LINE_CALIBRATION 7
#define ON_CROSS_1 8
#define CALIBRATION_LINE 9
#define DISPLAY_RESULTS 10
#define ANGLE_CALIBRATION 11
#define CALIBRATION_ARC 12
#define ON_CROSS_2 13
#define STATE_LINE 14
#define ON_CROSS_3 15
#define ON_CROSS_4 16
#define DISPLAY_LINEAR 17
#define DRIVE 18
#define STATE_ANTI_SQUARE 19
#define STATE_ANTI_DRIVE_FORWARD 20
#define ANTI_TURN 21


int state; // Variable to store the current state

unsigned long timer;

// square stuff:
float SQUARE_SIZE = 500; // Size of the square in mm
int side_count = 0; // Count of the number of sides of the square
int num_squares = 2; // Number of squares to complete
int dir = 1;

// straight line stuff
int tick_count = 0; // keep track of which tick is crossed
float distance_1; // distance measurement 1
float distance_2; // distance measurement 2

// calibration stuff:
float straight_line_dist = 500; // Length of measured straight line section in mm
long e0_start = 0;
long e1_start = 0;
float new_r0;
float new_r1;

float calibration_angle = PI; // the angle of the arc
float new_L;

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


      // SQUARE state decisions:
      //---------------------
      if (state == STATE_SQUARE && (millis() - timer >= 1500)) {
        state = STATE_DRIVE_FORWARD;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 0 && ((x_i) >= SQUARE_SIZE)) {
//        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 1 && ((y_i) >= SQUARE_SIZE)) {
//        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 2 && ((x_i) <= 0)) {
//        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == STATE_DRIVE_FORWARD && side_count % 4 == 3 && ((y_i) <= 0)) {
//        buzzer.beep(1000, 200);
        state = TURN;
      }

      else if (state == TURN && theta_i >= (((side_count + 1)*PI) / 2)) {
//        buzzer.beep(1000, 200);
        state = STATE_DRIVE_FORWARD;
        integral_reset();
        side_count++;
      }

      else if (state == TURN && side_count >= (( 4 * num_squares) - 1)) {
//        buzzer.beep(1000, 400);
        //        state = STATE_FINAL;
        state = DISPLAY_RESULTS;
      }

      

      // ANTI SQUARE state decisions:
      //---------------------
      if (state == STATE_ANTI_SQUARE && (millis() - timer >= 1500)) {
        state = STATE_ANTI_DRIVE_FORWARD;
      }

      else if (state == STATE_ANTI_DRIVE_FORWARD && side_count % 4 == 0 && ((x_i) >= SQUARE_SIZE)) {
//        buzzer.beep(1000, 200);
        state = ANTI_TURN;
      }

      else if (state == STATE_ANTI_DRIVE_FORWARD && side_count % 4 == 1 && (abs(y_i) >= SQUARE_SIZE)) {
//        buzzer.beep(1000, 200);
        state = ANTI_TURN;
      }

      else if (state == STATE_ANTI_DRIVE_FORWARD && side_count % 4 == 2 && ((x_i) <= 0)) {
//        buzzer.beep(1000, 200);
        state = ANTI_TURN;
      }

      else if (state == STATE_ANTI_DRIVE_FORWARD && side_count % 4 == 3 && ((y_i) >= 0)) {
//        buzzer.beep(1000, 200);
        state = ANTI_TURN;
      }

      else if (state == ANTI_TURN && theta_i <= -(((side_count + 1)*PI) / 2)) {
//        buzzer.beep(1000, 200);
        state = STATE_ANTI_DRIVE_FORWARD;
        integral_reset();
        side_count++;
      }

      else if (state == ANTI_TURN && side_count >= (( 4 * num_squares) - 1)) {
//        buzzer.beep(1000, 400);
        //        state = STATE_FINAL;
        state = DISPLAY_RESULTS;
      }



      // STRAIGHT_LINE state decisions:
      //-------------------------------
      else if (state == STATE_LINE && (millis() - timer >= 1500)) {
        // drive forward in a straight line 
        state = DRIVE;

      }

      else if (state == DRIVE && tick_count == 0 && line[0] && line[4]) {

        tick_count ++;
        distance_1 = x_i; // Record the x position of the rising edge of the first tick
        state = ON_CROSS_3;

      }

      else if (state == ON_CROSS_3 && !line[0] && !line[4]) {
        state = DRIVE;

      }

      else if (state == DRIVE && tick_count == 1 && line[0] && line[4]) {

        tick_count ++;
        distance_2 = x_i; // Record the x position of the rising edge of the second tick
        state = ON_CROSS_4;

      }

      else if (state == ON_CROSS_4 && !line[0] && !line[4]) {
        state = DISPLAY_LINEAR;

      }


      // CALIBRATION state decisions:
      //-----------------------------
      else if (state == LINE_CALIBRATION && tick_count == 0 && line[0] && line[4]) {

        tick_count ++;
        // TAKE COUNT MEASURE
        e0_start = count_e0;
        e1_start = count_e1;

        state = ON_CROSS_1;

      }
      else if (state == ON_CROSS_1 && !line[0] && !line[4]) {

        // change to folow line
        state = CALIBRATION_LINE;

        buzzer.beep(1000, 200);

      }
      else if (state == CALIBRATION_LINE && tick_count == 1 && line[0] && line[4]) {

        tick_count ++;

        // Update radius geometries
        new_r0 = kinematics.get_radius(0, straight_line_dist, e0_start);
        new_r1 = kinematics.get_radius(1, straight_line_dist, e1_start);

        buzzer.beep(1000, 200);

        // print result
        // state = DISPLAY_RESULTS;
        // continue calibration
        state = ON_CROSS_2;

      }


      // Angle calibration
      else if (state == ANGLE_CALIBRATION && line[0] && line[4]) {

        state = ON_CROSS_2;

      }
      else if (state == ON_CROSS_2 && !line[0] && !line[4]) {

        // TAKE COUNT MEASURE
        e0_start = count_e0;
        e1_start = count_e1;

        // change to follow line
        state = CALIBRATION_ARC;

        buzzer.beep(1000, 200);

      }
      else if (state == CALIBRATION_ARC && line[0] && line[4]) {

        // Update width geometry
        new_L = kinematics.get_L(calibration_angle, e0_start, e1_start, new_r0, new_r1);

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

        float angle = dir * side_count * PI / 2;
        motors.drive_along_angle(angle);

      } else if (state == STATE_ANTI_DRIVE_FORWARD) {

        float angle = -side_count * PI / 2;
        motors.drive_along_angle(angle);

      } else if (state == TURN) {

        motors.rotate_on_spot((dir * (side_count + 1)*PI) / 2);

        // right_motor_demand =  dir * TURN_ON_SPOT_SPEED;
        // left_motor_demand =  - dir * TURN_ON_SPOT_SPEED;

      } else if (state == ANTI_TURN) {

        motors.rotate_on_spot((-1 * (side_count + 1)*PI) / 2);

        // right_motor_demand =  dir * TURN_ON_SPOT_SPEED;
        // left_motor_demand =  - dir * TURN_ON_SPOT_SPEED;

      } else if (state == LINE_CALIBRATION || state == ON_CROSS_1 || state == ON_CROSS_2 || state == ON_CROSS_3 ||state == ON_CROSS_4 || state == CALIBRATION_LINE || state == ANGLE_CALIBRATION || state == CALIBRATION_ARC ) {

        motors.follow_line();

      } else if (state == DISPLAY_RESULTS) {

        // STOP
        right_motor_demand =  0;
        left_motor_demand = 0;

        Serial.print("new_r0: ");
        Serial.print(new_r0);
        Serial.print(" ");
        Serial.print("new_r1: ");
        Serial.print(new_r1);
        Serial.print(" ");
        Serial.print("new_L: ");
        Serial.println(new_L);

      } else if (state == DISPLAY_LINEAR) {

        // STOP
        right_motor_demand =  0;
        left_motor_demand = 0;

        // calculate distance
        float distance = distance_2 - distance_1;

        Serial.print("DISTANCE: ");
        Serial.println(distance);

      } else if (state == DRIVE) {
        motors.drive_along_angle(0);

      } else if (state == STATE_DEBUG) {

        float angle = -side_count * PI / 2;
        motors.drive_along_angle(angle);

        // right_motor_demand =  TURN_ON_SPOT_SPEED;
        // left_motor_demand = -TURN_ON_SPOT_SPEED;
        Serial.print(line[0]);
        Serial.print(line[1]);
        Serial.print(line[2]);
        Serial.print(line[3]);
        Serial.println(line[4]);



      }
    }
};

#endif
