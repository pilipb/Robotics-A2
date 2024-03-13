#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

// Global variables for the current PWM values

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

    // float ramp_motor(float target_speed, float step_size) {
        
    //     // If the current speed value is within the step size of the target value, return the target value
    //     if (abs(target_speed - current_speed) <= step_size) {
    //         current_speed = target_speed;
    //     }
    //     // If the current speed is less than the target value, increase it by the step size
    //     else if (current_speed < target_speed) {
    //         current_speed += step_size;
    //     }
    //     // If the current speed is greater than the target value, decrease it by the step size
    //     else
    //     {
    //         current_speed -= step_size;
    //     }

    //     return current_speed;
    // }

    };

#endif