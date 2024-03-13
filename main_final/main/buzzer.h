#ifndef _BUZZER_H
#define _BUZZER_H

#define BUZZER_PIN 6

// Class to operate the motors:
class Buzzer_c {
  public:
    Buzzer_c() {}

    void initialise() {
    pinMode(BUZZER_PIN, OUTPUT);
    }

    void beep(int frequency, int duration) {
    tone(BUZZER_PIN, frequency, duration);
    }
};

#endif