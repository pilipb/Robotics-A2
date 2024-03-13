// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _STATE_H
#define _STATE_H


#include "encoders.h"
#include "kinematics.h"
#include "motors.h"
#include "linesensor.h"

LineSensor_c linesensor;
Motors_c motor;

// define states
#define DEBUG 0
#define TO_LINE 1
#define JOIN_LINE 2
#define FOLLOW_LINE 3
#define TURN_AROUND 4
#define RIGHT_ANGLE_R 5
#define RIGHT_ANGLE_L 6
#define RETURN_HOME 7
#define OUT 8
#define CROSS 9
#define STRAIGHT_LINE 10

int state;
int prev_state;

float last_angle;
float dist_line;

// Class to contain generic PID algorithm.
class STATE_c {
  public:

    // Constructor, must exist.
    STATE_c() {

    }

    void initialise() {
      state = STRAIGHT_LINE;
      last_angle = 0;
      dist_line = 0;
    }

    void update(boolean online0, boolean online1, boolean online2, boolean online3, boolean online4, unsigned long start_time, unsigned long elapsed_ts) {

      
      if (global_X > 50) {

        last_angle = PI / 2;

      } else if (global_Y > 50) {

        last_angle = PI;

      } else if (global_X < 0) {

        last_angle = 3 * PI / 4;

      } else if (global_Y > 0) {

        motor.stop_robot();

      }

    }


    void action(unsigned long elapsed_ts) {

      if (state == DEBUG) {

        Serial.print(global_X);
        Serial.print(" ");
        Serial.println(global_Y);


      } else if (state = STRAIGHT_LINE) {

        motor.turn_to(last_angle, elapsed_ts, 25);

      }
    }
    };



#endif
