#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "tracker.h"

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

    cv::Mat frame;

    //TODO: EPSILON has a HUGE difference on max_val
    Tracker tracker(cv::Size(64, 64), 0.125);
    const bool draw_window = (std::getenv("DRAW_WINDOW") == NULL) ? 0 : 1;

    cap >> frame;
    if(frame.empty()) {
        fprintf(stderr, "Could not get frame of video");
        return -1;
    }

    tracker.initialize(frame, x, y);

    while (true) {
        cap >> frame;
        if(frame.empty()) {
            fprintf(stderr, "Could not get frame of video\n");
            return -1;
        }

        tracker.update(frame);
        /* printf("Difference: {%d, %d}\n", tracker.diff_x, tracker.diff_y); */
        float angle_diff_x = (tracker.diff_x / half_w) * h_fov;
        float angle_diff_y = -(tracker.diff_y / half_h) * v_fov;

        printf("Angle Difference: {x: %f, y: %f}\n", angle_diff_x, angle_diff_y);
        /* printf("Current Angle: {x: %f, y: %f}\n", curr_angle_x, curr_angle_y); */

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
    }
    return 0;
}
