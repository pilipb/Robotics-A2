// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _PID_H
#define _PID_H

float feedback_signal;
float p_gain;
float i_gain;
float d_gain;

float i_prev;
float prev_error;

// Class to contain generic PID algorithm.
class PID_c {
  public:

    // Constructor, must exist.
    PID_c() {

    }

    void initialise(float init_p, float init_i, float init_d) {

      p_gain = init_p;
      i_gain = init_i;
      d_gain = init_d;

      i_prev = 0;
      prev_error = 0;

      feedback_signal = 0;

    }


    float update( float demand, float measure, unsigned long elapsed_ts) {

      // function to return the feedback value
      float dt = elapsed_ts;
      float error = demand - measure;

      // proportional component
      float p_term = error;

      // derivative
      float d_term = (error - prev_error) / dt;

      // integrator component (integral(error dt))
      float i_term = i_prev;
      if ( abs(error) > abs(0.95 * demand)) {
        dt = 0;
      } else {
        i_term =  i_term + (error * dt);
      }


      float feedback = p_gain * p_term + i_gain * i_term + d_gain * d_term;

      i_prev = i_term * i_gain;
      prev_error = error;

      return feedback;

    }

};



#endif
