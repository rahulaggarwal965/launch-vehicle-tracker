#ifndef tracker_h
#define tracker_h

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "fourier_tools.h"

class Tracker {
public:
    cv::Mat N, D;
    cv::Size tracking_window_size;
    int prev_x, prev_y;
    double learning_rate, epsilon;
    Tracker(const cv::Size& tracking_window_size, double learning_rate = 0.125, double epsilon = 0.001);
    void initialize(const cv::Mat& frame, int x, int y);
    void update(const cv::Mat& frame);
    void draw(cv::Mat& frame);

private:
    cv::Mat H, response;
    void transform_fourier_space(const cv::Mat& frame, cv::Mat& dft, bool preprocess = true);
    void generate_perturbations(const cv::Mat& tracking_window, const cv::Mat& gaussian, cv::Mat perturbations[8], cv::Mat target_aff[8]);
    void preprocess(const cv::Mat &frame, cv::Mat &dst);
};

#endif
