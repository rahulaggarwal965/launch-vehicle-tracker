#ifndef GPIO_H
#define GPIO_H

#include <pigpio.h>

// This is really annoying. I have to basically push out to pin 12 (and hope the channel is still connected to GPIO12). Then I will likely have to have some sort of jumper between GPIO 12/13. This is because some GPIO pins are not connected for some reason.............. - Rahul
#define PWM_OUT 12
#define PWM_CHANNEL 0
#define PWM_RANGE 1024

#define SERVO1_PIN 12
#define SERVO2_PIN 13

#define STEP_PIN 11
#define DIR_PIN 12
#define MS1_PIN 24
#define MS2_PIN 23
#define MS3_PIN 22
#define ENABLE_PIN 25
#define SLEEP_PIN 7
#define RESET_PIN 27

int setup_pins();
void cleanup_pins();

#endif
