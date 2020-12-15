#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "tracker.h"
#include <time.h>
#include <wiringPi.h>

#define CLAMP(x, low, high)  (((x) > (high)) ? high : (((x) < (low)) ? (low) : (x)))

struct CurrentAngle {
    float x;
    float y;
};  

int map_to_pwm(float angle) {
    angle = CLAMP(angle, -90, 90);
    return (int) (((angle + 90) / 9) * 10 + 50);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }
    const char * vName = argv[1];
    int x = (argc > 2) ? atoi(argv[2]) : 640;
    int y = (argc > 3) ? atoi(argv[3]) : 220;

    cv::VideoCapture cap(vName);
    const float half_w = cap.get(cv::CAP_PROP_FRAME_WIDTH) / 2;
    const float half_h = cap.get(cv::CAP_PROP_FRAME_HEIGHT) / 2;

    const float h_fov = 62;
    const float v_fov = 49;

    const int PWM_CENTER = 150;

    cv::Mat frame;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;


    CurrentAngle curr_angle = {};

    // TODO(rahul): probably move this into another file
    // WiringPi
    wiringPiSetupGpio();

    // TODO(rahul): both channels
    int pwm_pin_x = 18, pwm_pin_y = 13;
    pinMode(pwm_pin_x, PWM_OUTPUT);
    pinMode(pwm_pin_y, PWM_OUTPUT);

    // 50hz (19200000 / 192 / 2000)
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(192);
    pwmSetRange(2000);

    cap >> frame;
    if(frame.empty()) {
        fprintf(stderr, "Could not get frame of video\n");
        return -1;
    }
    int frames = 0;
    time_t start, end;
    time(&start);
    pwmWrite(pwm_pin_x, PWM_CENTER);
    pwmWrite(pwm_pin_y, PWM_CENTER - 50);
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
        curr_angle.x += (tracker.diff_x / half_w) + h_fov;
        curr_angle.y -= (tracker.diff_y / half_h) * v_fov;
        int pwm_x = map_to_pwm(curr_angle.x);
        int pwm_y = map_to_pwm(curr_angle.y);

        printf("Current Angle: {x: %f, y: %f}, PWM: {x: %d, y: %d}\n", curr_angle.x, curr_angle.y, pwm_x, pwm_y);

        pwmWrite(pwm_pin_x, pwm_x);
        pwmWrite(pwm_pin_y, pwm_y);

        // TODO: actuate servo motors based on these difference values, I have
        // to find correct values for speed using pwm.

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
    return 0;
}
