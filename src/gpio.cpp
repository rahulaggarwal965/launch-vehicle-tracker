#include "gpio.h"

int setup_pins() {
    if (gpioInitialise() < 0) {
        return 1;
    }

    gpioSetMode(STEP_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);
    gpioSetMode(MS1_PIN, PI_OUTPUT);
    gpioSetMode(MS2_PIN, PI_OUTPUT);
    gpioSetMode(MS3_PIN, PI_OUTPUT);
    gpioSetMode(ENABLE_PIN, PI_OUTPUT);
    gpioSetMode(SLEEP_PIN, PI_OUTPUT);
    gpioSetMode(RESET_PIN, PI_OUTPUT);

    // Full step microstepping
    gpioWrite(MS1_PIN, 0);
    gpioWrite(MS2_PIN, 0);
    gpioWrite(MS3_PIN, 0);

    // TODO(rahul): should set these at a startup phase so that the a4988 doesn't move when not working
    gpioSetPullUpDown(RESET_PIN, PI_PUD_UP); 
    gpioSetPullUpDown(ENABLE_PIN, PI_PUD_UP);
    gpioSetPullUpDown(SLEEP_PIN, PI_PUD_UP); 

    return 0;
}

void cleanup_pins() {
    gpioServo(SERVO1_PIN, 0);
    gpioServo(SERVO2_PIN, 0);
    gpioTerminate();
}
