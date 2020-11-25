#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "tracker.h"
#include <time.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }
    const char * vName = argv[1];
    int x = (argc > 2) ? atoi(argv[2]) : 640;
    int y = (argc > 3) ? atoi(argv[3]) : 220;

    cv::VideoCapture cap(vName);
    cv::Mat frame;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;

    const float pwm_mod = 0;

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
    return 0;
}
