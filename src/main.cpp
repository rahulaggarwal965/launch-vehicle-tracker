// opencv libraries
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

// linux system libraries
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

// usr includes
#include "tracker.h"
#include "gpio.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? high : (((x) < (low)) ? (low) : (x)))

struct CurrentAngle {
    float x;
    float y;
};  

// angle in degrees
// -90 -> 1000
// 90 -> 2000
int map_to_pulse_width(float angle) {
    angle = CLAMP(angle, -90, 90);
    return (int) (angle * 5 / 9 + 1500);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }

    // prevent process swapping from the linux kernel while on the raspberry pi
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    setup_pins();

    const char * vName = argv[1];
    int x = (argc > 2) ? atoi(argv[2]) : 640;
    int y = (argc > 3) ? atoi(argv[3]) : 220;

    cv::VideoCapture cap(vName);
    const float half_w = cap.get(cv::CAP_PROP_FRAME_WIDTH) / 2;
    const float half_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT) / 2;

    const float h_fov = 62;
    const float v_fov = 49;

    cv::Mat frame;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;

    CurrentAngle curr_angle = {};

    cap >> frame;
    if(frame.empty()) {
        fprintf(stderr, "Could not get frame of video\n");
        return -1;
    }
    int frames = 0;
    time_t start, end;
    time(&start);
    gpioServo(SERVO1_PIN, PULSE_CENTER);
    gpioServo(SERVO2_PIN, PULSE_CENTER);
    tracker.initialize(frame, x, y);

    while (cap.isOpened()) {
        cap >> frame;
        if(frame.empty()) {
            fprintf(stderr, "Could not get frame of video\n");
            time(&end);
            printf("FPS: ~%f\n", frames / difftime(end, start));
            return -1;
        }

        tracker.update(frame);
        /* printf("Difference: {%d, %d}\n", tracker.diff_x, tracker.diff_y); */
        //float angle_diff_x = (tracker.diff_x / half_w) * h_fov;
        //float angle_diff_y = -(tracker.diff_y / half_h) * v_fov;

        // x is a stepper
        // y is a servo
        float diff_angle_x = tracker.diff_x / half_w + h_fov;
        curr_angle.x += diff_angle_x;
        curr_angle.y -= (tracker.diff_y / half_h) * v_fov;
        int pw_y = map_to_pulse_width(curr_angle.y);

        gpioServo(SERVO1_PIN, pw_y);
        gpioServo(SERVO2_PIN, pw_y);

        if (diff_angle_x > STEP_ANGLE_THRESH) {
            int dir = (diff_angle_x < 0) ? 0 : 1;
            gpioWrite(DIR_PIN, dir);
            gpioTrigger(STEP_PIN, 5, 1);
        }
        /* printf("Current Angle: {x: %f, y: %f}, PWM: {x: %d, y: %d}\n", curr_angle.x, curr_angle.y, pwm_x, pwm_y); */


        if (draw_window) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            tracker.draw(gray);

            cv::imshow("tracker", gray);

            if (cv::waitKey(-1) == 113) {
                break;
            }
        }
    frames++;
    }
    time(&end);
    printf("FPS: ~%f\n", frames / difftime(end, start));

    cleanup_pins();

    return 0;
}
