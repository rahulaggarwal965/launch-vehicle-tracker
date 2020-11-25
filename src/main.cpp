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
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        tracker.draw(gray);

        //cv::imshow("tracker", gray);

        frames++;

        //if (cv::waitKey(-1) == 113) {
        //    break;
        //}
    }
    time(&end);
    printf("FPS: ~%f\n", frames / difftime(end, start));
    return 0;
}
