#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include "encoders.h"

int const COUNTS_PER_REV = 359; // Number of encoder updates per revolution
float const WHEEL_RADIUS_R = 32.5 / 2; // Wheel radius in mm right
float const WHEEL_RADIUS_L = 32.5 / 2; // Wheel radius in mm left
float distance_per_count_r = (2 * PI*WHEEL_RADIUS_R) / COUNTS_PER_REV; // Distance per encoder count in mm
float distance_per_count_l = (2 * PI*WHEEL_RADIUS_L) / COUNTS_PER_REV; // Distance per encoder count in mm

float x_i; // Global x position of the robot
float y_i; // Global y position of the robot
float theta_i; // Angle of the robot WRS to the x-axis

// Class to track robot position and angle
class Kinematics_c {

  private:
    float const L = 85.3 / 2; // Distance from wheel to center of robot in mm
    int last_e0 = 0; // Encoder value of the Right encoder at the last kinematic update
    int last_e1 = 0; // Encoder value of the Left encoder at the last kinematic update

  public:
    Kinematics_c() {
      x_i = 0;
      y_i = 0;
      theta_i = 0;
    }

    void initialise() {
      x_i = 0;
      y_i = 0;
      theta_i = 0;
    }

    // Function to update the position and angle of the robot based on the encoder counts
    void update() {

      // Calculate the change in encoder counts since the last kinematic update
      long delta_e0;
      long delta_e1;
      delta_e0 = count_e0 - last_e0;
      delta_e1 = count_e1 - last_e1;
      // Save the current encoder counts for the next update
      last_e0 = count_e0;
      last_e1 = count_e1;

      // calculate the distance travelled since the last update
      float distance_r = delta_e0 * distance_per_count_r;
      float distance_l = delta_e1 * distance_per_count_l;
      float delta_xr = (distance_l + distance_r) / 2;

      // update the global x and y position of the robot
      x_i = x_i + (delta_xr * cos(theta_i));
      y_i = y_i + (delta_xr * sin(theta_i));

      // calculate the change in angle since the last update
      float delta_theta = (distance_r - distance_l) / (2 * L);
      // update the global angle of the robot
      theta_i = theta_i + delta_theta;
    }

    // Function to calculate the radii of the left and right wheels
    float get_radius(int wheel, float dist_travelled, long start_count) {
      // this calculation is derived from travel along a straightline (and is only valid when travelling
      // in a straight line)

      float r;
      if (wheel == 0) {
        r = COUNTS_PER_REV * dist_travelled / (2*PI*(count_e0 - start_count));
      } else {
        r = COUNTS_PER_REV * dist_travelled / (2*PI*(count_e1 - start_count));
      }
      return abs(r);
    }

    // Function to calculate the width L
    float get_L(float angle, long start_count_r, long start_count_l, float new_r0, float new_r1) {
      // this function assumes that the radius of both wheels is known and correct
      float L;

      float delta_r = (count_e0 - start_count_r) * new_r0;
      float delta_l = (count_e1 - start_count_l) * new_r1;

      L = abs((PI * (delta_r - delta_l)) / (angle * COUNTS_PER_REV));

      return L;

    }

};



#endif
