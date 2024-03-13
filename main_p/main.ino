#include "state.h"
#include "linesensor.h"
#include "motors.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

STATE_c state_c;
PID_c pid1;
//Motors_c motor;
//LineSensor_c linesensor;
Kinematics_c matics;

// ls states
boolean online0;
boolean online1;
boolean online2;
boolean online3;
boolean online4;

// timings
unsigned long ts;
unsigned long start_time;

float straight_dist;
float heading;

void setup() {

  //  placement delay
  delay(1000);

  // initialise the classes
  setupEncoder0();
  setupEncoder1();
  linesensor.initialise();
  motor.initialise();
  matics.initialise();

  start_time = millis();
  e0_count_t = 0;
  e1_count_t = 0;

  // start time
  ts = millis();

  // Configure the Serial port
  Serial.begin(9600);

  // starting state
  state_c.initialise();

}

void loop() {

  //  get time
  unsigned long elapsed_ts = millis() - ts;

  if (elapsed_ts > 30) {

    //    update linesensors
//    online0 = linesensor.onLine(0);
//    online1 = linesensor.onLine(1);
//    online2 = linesensor.onLine(2);
//    online3 = linesensor.onLine(3);
//    online4 = linesensor.onLine(4);

    //    update matics
    long delta_e0 = count_e0 - e0_count_t;
    long delta_e1 = count_e1 - e1_count_t;
    matics.update(delta_e0, delta_e1);
    e0_count_t = count_e0;
    e1_count_t = count_e1;

    // update state
    state_c.update(online0, online1, online2, online3, online4, start_time, elapsed_ts);

    // state actions based on update
    state_c.action(elapsed_ts);

    ts = millis();
  }

}
