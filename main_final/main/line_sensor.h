#ifndef _LINE_SENSOR_H
#define _LINE_SENSOR_H

#include <limits.h>

#define LINE_THRESHOLD   1100 // Threshold for line detection

#define EMIT_PIN    11   // Pin for IR emitter 
// OUTPUT HIGH = Activate line sensor LEDs
// OUTPUT LOW = Activate bump sensor LEDs 
// INPUT = turn off LEDs (high impedance)

// LED feedback pins:
#define LED_PIN 13 // Yellow LED on the Left (ACTIVE HIGH)
#define TX_PIN 30  // Green LED in the center (ACTIVE LOW)
#define RX_PIN 17  // Red LED on the Right (ACTIVE LOW)


// Line sensor analog pins:
#define DN1_ANALOG A11 // DN1 analog pin (left)
#define DN2_ANALOG A0  // DN2 analog pin
#define DN3_ANALOG A2  // DN3 analog pin (centre)
#define DN4_ANALOG A3  // DN4 analog pin
#define DN5_ANALOG A4  // DN5 analog pin (right)

// Intiger array of analog sensor pins:
int analog_sensor_pins[5]={DN1_ANALOG, 
                           DN2_ANALOG, 
                           DN3_ANALOG, 
                           DN4_ANALOG, 
                           DN5_ANALOG};

// Line sensor digital pins:
#define DN1_DIGITAL 12  // DN1 digital pin (left)
#define DN2_DIGITAL 18  // DN2 digital pin
#define DN3_DIGITAL 20  // DN3 digital pin (centre)
#define DN4_DIGITAL 21  // DN4 digital pin
#define DN5_DIGITAL 22  // DN5 digital pin (right)

// Intiger array of digital sensor pins:
int digital_sensor_pins[5]={DN1_DIGITAL, 
                            DN2_DIGITAL, 
                            DN3_DIGITAL, 
                            DN4_DIGITAL, 
                            DN5_DIGITAL};



class LineSensor_c {

    public:
        LineSensor_c() {}

void initialise_analog() {

    // Set the emmiter pin to output
    pinMode(EMIT_PIN, OUTPUT);
    pinMode(EMIT_PIN, HIGH);

    // Set the sensors pins to pullup input
    for(int i = 0; i < 5; i++) {
        pinMode(analog_sensor_pins[i], INPUT_PULLUP);
    }
}

void initialise_digital() {

    // Turn on the emitter
    pinMode(EMIT_PIN, OUTPUT);
    digitalWrite(EMIT_PIN, HIGH);

    // Set the LED feedback pins to output
    pinMode(LED_PIN, OUTPUT);
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, OUTPUT);

    // Set the sensor pins to input
    for(int i = 0; i < 5; i++) {
        pinMode(digital_sensor_pins[i], INPUT);
    }
}

// Read a single sensor:
unsigned long read_sensor_digital(int sensor_index) {

    // Return the maximum unasigned long value if the sensor_index 
    // is out of the valid range
    if(sensor_index < 0 || sensor_index > 4) {
        return ULONG_MAX;
    }

    // Set sensor pin to output and set it high to charge the capacitor
    pinMode(digital_sensor_pins[sensor_index], OUTPUT);
    digitalWrite(digital_sensor_pins[sensor_index], HIGH);

    // Wait for 10 microseconds
    delayMicroseconds(10);

    // Set sensor pin to input to start discharging the capacitor
    pinMode(digital_sensor_pins[sensor_index], INPUT);

    // Start the timer
    unsigned long start_time = micros();

    // Wait for the sensor to go low
    while (digitalRead(digital_sensor_pins[sensor_index]) == HIGH) {
            // (waiting).
    }

    // Calculate the time it took for the capacitor to discharge
    // Longer discharge time = darker surface
    unsigned long discharge_time = micros() - start_time;

    // Return the discharge time
    return discharge_time;

}

// Read all sensors in parallel:
void read_all_sensors_digital(unsigned long readings[5]) {
    
    const int no_sensors = 5; // Number of sensors
    int sensors_reading = no_sensors; // Initialise the number of sensors still reading
    unsigned long t_start, t_end; // Initialise the start and end times
    unsigned long timeout = 500000; // Timeout in microseconds (0.5s)
    bool timeout_reached = false; // Initialise a boolean variable that stores if the timeout has been reached

    // Create a boolean array to keep track of which sensors have finished reading
    bool finished[no_sensors];
    // Set all sensors to not finished initially
    for(int i = 0; i < no_sensors; i++) {
        finished[i] = false;
    }
    
    // Set sensor pins to output and set them all high to charge the capacitors
    for(int i = 0; i < no_sensors; i++) {
        pinMode(digital_sensor_pins[i], OUTPUT);
        digitalWrite(digital_sensor_pins[i], HIGH);
    }

    delayMicroseconds(10);

    // Start reading
    for(int i = 0; i < no_sensors; i++) {
        pinMode(digital_sensor_pins[i], INPUT);
    }
    t_start = micros();

    // Wait for all sensors to go low or for the timeout to be reached
    while(sensors_reading > 0 && !timeout_reached) {
        // Check if any sensors have finished reading
        for(int i = 0; i < no_sensors; i++) {
            // Check if the timeout has been exceeded
            if(micros() - t_start > timeout) {
                timeout_reached = true;
                break; // Break out of the for loop
            }
            // If the sensor has not finished reading and is low, record the time and set it to finished
            if(!finished[i] && digitalRead(digital_sensor_pins[i]) == LOW) {
                t_end = micros();
                readings[i] = t_end - t_start; // Record the time to the global readings array
                finished[i] = true;
                // Subtract 1 from the list of sensors still reading
                sensors_reading--;
            }
        }
    }

    // Set all unfinished sensors to 0
    if(micros() - t_start > timeout) {
        for(int i = 0; i < no_sensors; i++) {
            if(!finished[i]) {
                readings[i] = 0;
            }
        }
    }
}

void read_all_sensors_adc(unsigned long readings[5]) {

    // Iterate through each sensor and read the analog value
    // and store it in the sensor_readings array
    for(int i = 0; i < 5; i++) {
        readings[i] = analogRead(analog_sensor_pins[i]);
    }
}

void line_threshold(unsigned long readings[5], bool line[5]) {

    // Iterate through each sensor and compare the reading to the threshold
    for (int i = 0; i < 5; i++) {
        line[i] = readings[i] > LINE_THRESHOLD; // Set line to TRUE if the reading is above the threshold
    }

    // Activate the correct LED feedback if a line is detected
    digitalWrite(LED_PIN, line[1] ? HIGH : LOW);
    digitalWrite(TX_PIN, line[2] ? LOW : HIGH);
    digitalWrite(RX_PIN, line[3] ? LOW : HIGH);
}

float normalise_sensor_readings(unsigned long readings[5]) {

    // Calculate the difference between the mid-left and mid-right sensor readings
    long difference = readings[1] - readings[3];

    // Find the maximum sensor value
    long max_sensor_value = max(max(readings[1], readings[2]), readings[3]);

    // Normalize the difference to a value between -1.0 and 1.0
    float normalised = (float)difference / (float)max_sensor_value;

    return normalised;
}

};

#endif