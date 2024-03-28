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

int state; // Variable to store the current state

unsigned long timer;

// square stuff:
float SQUARE_SIZE = 200; // Size of the square in mm
int side_count = 0; // Count of the number of sides of the square
int num_squares = 1; // Number of squares to complete
int dir = 1;


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

      if (state == STATE_INITIAL && (millis() - timer >= 1500)) {
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
        state = STATE_FINAL;
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

        //  } else if (state == STATE_FOLLOW_LINE) {
        //
        //    follow_line();
        //
        //  } else if (state == STATE_REJOIN_LINE) {
        //
        //    rejoin_line();
        //
        //  } else if (state == STATE_INTERSECTION) {
        //
        //    intersection();
        //
        //  } else if (state == STATE_CROSSROADS) {
        //
        //    crossroads();
        //
        //  } else if (state == STATE_FACE_HOME) {
        //
        //    face_home();
        //
        //  } else if (state == STATE_RTH) {
        //
        //    return_home();
        //
        //  } else if (state == STATE_FINISHED) {
        //
        //    // Stop the motors
        //    motors.set_motor(0, 0, MAX_MOTOR_STEP_SIZE);
        //    if (millis() - timer <= 1500) {
        //      buzzer.beep(1000, 10);
        //    }

      } else if (state == STATE_DEBUG) {

        right_motor_demand =  TURN_ON_SPOT_SPEED;
        left_motor_demand = -TURN_ON_SPOT_SPEED;

      }
    }
};

#endif
