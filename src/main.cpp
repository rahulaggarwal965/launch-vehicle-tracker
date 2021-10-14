// opencv libraries
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// linux system libraries
#include <time.h>
#include <sched.h>
#include <sys/mman.h>

// usr includes
#include "tracker.h"
#include "gpio.h"

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
    cv::Mat frame;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;

    cap >> frame;
    if(frame.empty()) {
        fprintf(stderr, "Could not get frame of video");
        return -1;
    }
    int frames = 0;
    time_t start, end;
    time(&start);
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
        printf("Difference: {%d, %d}\n", tracker.diff_x, tracker.diff_y);
        // TODO: actuate servo motors based on these difference values, I have
        // to find correct values for speed using pwm.

        if (draw_window) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            tracker.draw(gray);

            cv::imshow("tracker", gray);

            if (cv::waitKey(33) == 113) {
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
