// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define REV HIGH // Remember notation
# define FWD LOW

long e0_count_t;
long e1_count_t;

PID_c heading_pid;
PID_c left_PID;
PID_c right_PID;
float heading_feedback;

float left_feedback;
float right_feedback;


// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    }

    void initialise() {

      pinMode( L_PWM_PIN, OUTPUT);
      pinMode( R_PWM_PIN, OUTPUT);
      pinMode( R_DIR_PIN, OUTPUT);
      pinMode( L_DIR_PIN, OUTPUT);

      heading_pid.initialise(0.8, 0.3, 0);
      left_PID.initialise(1, 0, 0);
      right_PID.initialise(1, 0, 0);
      heading_feedback = 0;
      left_feedback = 0;
      right_feedback = 0;

    }

    void straight_line(float speed, unsigned long elapsed_ts) {

      // go in straight line at speed cm/s
      float rot_demand = (speed / 1.6) / 2; // as there are two wheel

      float measure_left = vel_rot(1, elapsed_ts);
      float measure_right = vel_rot(0, elapsed_ts);

      left_feedback = left_PID.update(rot_demand, measure_left, elapsed_ts);
      right_feedback = right_PID.update(rot_demand, measure_right, elapsed_ts);

      setMotorPower(left_feedback, right_feedback);
    }


    void stayOnLine(float dir, int speed, int con_speed) {
      // scale power to the wheels based the direction from linesensor
      // dir too far: = -1 = left, 0 = straight, 1 = right
      setMotorPower( con_speed + (dir * speed), con_speed - (dir * speed)); // speed is effectivel K_p gain
    }
    void turn_right() {
      setMotorPower(22, 0);
    }

    void turn_right_spot() {
      setMotorPower(22, -22);
    }

    void turn_left() {
      setMotorPower(-22, 22);
    }

    void turn_to(float heading_demand, unsigned long elapsed_ts, int speed) {

      heading_feedback = heading_pid.update(heading_demand, global_theta, elapsed_ts);
      stayOnLine(heading_feedback, speed, speed - 5);

    }

    float vel_rot(int wheel, unsigned long elapsed_ts) {

      //  float alpha = 0.1;
      float delta_e;
      float dt = elapsed_ts;

      if (wheel == 0) {
        delta_e = count_e0 - e0_count_t;
      } else if (wheel == 1) {
        delta_e = count_e1 - e1_count_t;
      } else {
        Serial.println("Invalid wheel index");
        return 0;
      }

      float velocity = delta_e / dt;

      return velocity;

    }

    void set_dir(boolean forwards) {
      if (forwards) {
        digitalWrite( L_DIR_PIN, FWD);
        digitalWrite( R_DIR_PIN, FWD);
      } else {
        digitalWrite( L_DIR_PIN, REV);
        digitalWrite( R_DIR_PIN, REV);
      }
    }

    void setMotorPower( float left_pwm, float right_pwm ) {
      // Sets the power up to 90 and direction of the motors.
      // Set the direction based on the sign of the input values
      if (left_pwm >= 0) {
        digitalWrite(L_DIR_PIN, FWD);
      } else {
        digitalWrite(L_DIR_PIN, REV);
      }

      if (right_pwm >= 0) {
        digitalWrite(R_DIR_PIN, FWD);
      } else {
        digitalWrite(R_DIR_PIN, REV);
      }

      // Map the absolute values of pwm to the range [0, 70]
      int left_pwm_mapped = constrain(abs(left_pwm), 0, 90);
      int right_pwm_mapped = constrain(abs(right_pwm), 0, 90);

      // Set the PWM values using analogWrite()
      analogWrite(L_PWM_PIN, left_pwm_mapped);
      analogWrite(R_PWM_PIN, right_pwm_mapped);

    }


    void circle(int size, int speed) {
      // go in a circle radius size
      analogWrite( L_PWM_PIN, speed * sin(size + 2));
      analogWrite( R_PWM_PIN, speed * sin(size ));

    }

    void stop_robot() {
      setMotorPower(0, 0);
    }




};



#endif
