#ifndef _PID_H
#define _PID_H

float rotation_speed_r_smoothed;
float rotation_speed_l_smoothed;

// Class to contain generic PID algorithm.
class PID_c {
    private:

        float Kp; // Proportional gain
        float Ki; // Integral gain
        float Kd; // Derivative gain
        unsigned long last_time; // ts of the last time the PID was updated
        float last_error; // Last error value

        float smoothed_measurement; // Smoothed measurement value

        unsigned long current_ts = millis();
        unsigned long previous_loop_ts = 0;
        unsigned long delta_ts = 0;
        long last_e0 = 0;
        long last_e1 = 0;

    public:
        float integral_sum; // Sum of error over time
        PID_c() :
        Kp(0), Ki(0), Kd(0), last_time(0), last_error(0), smoothed_measurement(0), integral_sum(0)
        {}

// Initialize the PID controller with a specific Kp value
    void initialise(float set_Kp, float set_Ki, float set_Kd) {
        Kp = set_Kp;
        Ki = set_Ki;
        Kd = set_Kd;

        last_time = millis(); // Set the last time to the current time
        integral_sum = 0; // Set the intial integral sum to 0
        last_error = 0; // Set the intial last error to 0
    }

    float LPF(float new_measurement, float filter_alpha) {
        // Pass readings through a low pass filter to smooth readings
        smoothed_measurement = (smoothed_measurement * (1 - filter_alpha)) + (new_measurement * filter_alpha);
        return smoothed_measurement;
    }

    void calculate_rotation_speed() {

        // Calculate the time since the last calculation
        current_ts = millis();
        delta_ts = current_ts - previous_loop_ts;
        previous_loop_ts = current_ts;

        // Calculate the change in encoder counts since the last update
        long delta_e0;
        long delta_e1;
        delta_e0 = count_e0 - last_e0; 
        delta_e1 = count_e1 - last_e1;
        last_e0 = count_e0; // Save the current encoder counts for the next update
        last_e1 = count_e1;

        // Calculate rotation speed in mm/s
        float rotation_speed_r = ((float)delta_e0 / (float)delta_ts) * 1000.0 * distance_per_count;
        float rotation_speed_l = ((float)delta_e1 / (float)delta_ts) * 1000.0 * distance_per_count;
        // Pass the readings through a low pass filter to smooth the readings
        rotation_speed_r_smoothed = LPF(rotation_speed_r, 0.17);
        rotation_speed_l_smoothed = LPF(rotation_speed_l, 0.17); 
    }

    float update(float demand, float measurement) {

        // Calculate the time since the last update
        unsigned long current_time = millis();
        float delta_t = (current_time - last_time) / 1000.0;
        // Store the current time
        last_time = current_time;

        // Calculate the error signal
        float error = demand - measurement;

        // Calculate the proportional term
        float P_out = Kp * error;

        // Calculate the integral sum
        integral_sum = integral_sum + (error * delta_t);
        // Calculate the integral term
        float I_out = Ki * integral_sum;

        // Calculate the derivative term
        float D_out = Kd * (error - last_error);

        // Store the error
        last_error = error;

        // Calculate the feedback signal
        float feedback = P_out + I_out + D_out;

        // Serial.print("demand:");
        // Serial.print(demand);
        // Serial.print(" ");
        // Serial.print("measurement:");
        // Serial.print(measurement);
        // Serial.print(" ");
        // Serial.print("error:");
        // Serial.print(error);
        // Serial.print(" ");
        // Serial.print("control:");
        // Serial.println(feedback);

        // Return the feedback
        return feedback;
    }

    
};
#endif