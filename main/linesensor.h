
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define EMIT_PIN 11

#define LS_PIN_1 A11 // line sensor 1
#define LS_PIN_2 A0
#define LS_PIN_3 A2
#define LS_PIN_4 A3
#define LS_PIN_5 A4

int ls_pins[5] = { LS_PIN_1, LS_PIN_2, LS_PIN_3, LS_PIN_4 , LS_PIN_5 };

// Class to operate the linesensor(s).
class LineSensor_c {
  public:

    // Constructor, must exist.
    LineSensor_c() {

    }

    void initialise() {
      // Set some initial pin modes and states
      pinMode( EMIT_PIN, OUTPUT ); // Set EMIT as an input (off)
      digitalWrite( EMIT_PIN, HIGH ); // turns on IR

      // set pins as resistor
      pinMode(ls_pins[0], INPUT_PULLUP);
      pinMode(ls_pins[1], INPUT_PULLUP);
      pinMode(ls_pins[2], INPUT_PULLUP);
      pinMode(ls_pins[3], INPUT_PULLUP);
      pinMode(ls_pins[4], INPUT_PULLUP);
    }


    int bangFollow() {
      // function to read the middle three sensors and as a result return whether the
      // robot should turn left -1, right 1, straight 0

      // scenario 1: all sensors on the line, return 0
      if (onLine(1) && onLine(2) && onLine(3)) {
        return 0; // going straight along line
      }
      if (not onLine(1) && onLine(2) && onLine(3)) {
        // too far left, should turn right
        return 1;
      }
      if (onLine(1) && onLine(2) && not onLine(3)) {
        // too far right, should turn left
        return -1;
      } else {
        // error state (line finished)
        return 0;
      }

    }

    float weightFollow() {
      // function to read two outer middle (2 and 4) sensors and return whether the robot should turn,
      // but as a contunuous value [-1 : 1 ]
      float ls1 = 1.4 * analogLineSensor(0);
      float ls5 = 1.4 * analogLineSensor(4);

      float ls2 = analogLineSensor(1);
      float ls4 = analogLineSensor(3);
      float sum = ls1 + ls2 + ls4 + ls5;
      //      normalise and double
      float W = (ls5 + ls4 - ls1 - ls2) * 2 / max(ls2, ls4);
     
      return W;
    }

    void weightReadings(float readings[5]) {
      // weight the readings
      float sum = 0;
      for (int i = 0; i < 5; i++) {
        sum = sum + readings[i];
      }
      for (int i = 0; i < 5; i++) {
        readings[i] = readings[i] / sum;
      }
    }

    void getReadings(float readings[5]) {
      for (int i = 0; i < 5; i++) {
        readings[i] = analogLineSensor(i);
      }
    }

    boolean onLine(int sensorNumber) {
      // returns whether the given sensor is on the line (true)
      //      or not (in daylight around 800)
      int val = analogLineSensor(sensorNumber);
      int threshold = 800;
      if (sensorNumber == 4) {
        threshold = 810;
      }
      if (val > threshold) {
        return true;
      } else {
        return false;
      }
    }

    int analogLineSensor(int sensorNumber) {
      int val;
      val = analogRead(ls_pins[sensorNumber]);
      return val;
    }


    float digiLineSensor(int sensorNumber) {
      //  function to read ls 1-5 and output float val index 0
      //  12, A0, A2, A3, A4
      pinMode( EMIT_PIN, OUTPUT );
      digitalWrite( EMIT_PIN, HIGH );

      if (sensorNumber < 0 || sensorNumber > 4) {
        // Invalid sensor number, handle error or return a specific value
        Serial.println("Invalid sensor number");
        return 0;
      }
      int sensorPin = ls_pins[sensorNumber];

      pinMode( sensorPin, OUTPUT );
      digitalWrite( sensorPin, HIGH ); // charges capacitor
      delayMicroseconds( 10 ); // delay to charge

      unsigned long start_time = micros();
      pinMode( sensorPin, INPUT );
      while (digitalRead( sensorPin) == HIGH) {
        // Do nothing here (waiting).
        //
        //    //    time out check:
        if (micros() - start_time >= 30000) {
          // time out happens because not enough IR reflecting
          // (not on surface = 2400 with IR)

          break;
        }

      }
      unsigned long end_time = micros();

      pinMode(EMIT_PIN, INPUT);

      unsigned long elapsed_time = end_time - start_time;
      return (float)elapsed_time;
    }



};



#endif
