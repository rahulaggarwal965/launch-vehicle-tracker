#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdlib.h>
#include <queue>
#include <thread>
#include <unistd.h>
#include "tracker.h"

//NOTE: matrix copies headers only
void capture_frames(cv::VideoCapture *cap, std::queue<cv::Mat> *q) {
    cv::Mat frame;
    while ((*cap).isOpened()) {
        *cap >> frame;
        if(frame.empty()) {
            fprintf(stderr, "Frame was empty\n");
            break;
        }
        (*q).push(frame);
    }
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
    cv::Mat frame;
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);

    cap >> frame;
    if(frame.empty()) {
        fprintf(stderr, "Could not get frame of video");
        return -1;
    }

    tracker.initialize(frame, x, y);

    std::queue<cv::Mat> q;
    std::thread capture(capture_frames, &cap, &q);

    /* while (cap.isOpened()) { */
    while (true) {
        printf("queue length %lu\n", q.size());
        sleep(1);
        while (!q.empty()) {
            frame = q.front();
            q.pop();

            tracker.update(frame);
            if (draw_window) {
              cv::Mat gray;
              cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
              tracker.draw(gray);

              cv::imshow("tracker", gray);
              if (cv::waitKey(33) == 113) {
                break;
              }
            }
        }
    }
    capture.join();
    return 0;
}
