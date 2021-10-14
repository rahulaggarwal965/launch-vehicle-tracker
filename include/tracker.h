#ifndef tracker_h
#define tracker_h

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "fourier_tools.h"
#include <iostream>
#include <fstream>

class Tracker {
    public:
        cv::Mat N, D;
        cv::Size tracking_window_size;
        // TODO(rahul): should this be a float or an int
        float prev_x, prev_y, diff_x, diff_y;
        double learning_rate, epsilon;

        Tracker(const cv::Size &tracking_window_size, double learning_rate = 0.125, double epsilon = 0.001);
        void initialize(const cv::Mat &frame, int x, int y);
        void update(const cv::Mat &frame);
        void draw(cv::Mat &frame);

    private:
        cv::Mat H, response, hanning_window;

        double compute_psr(const cv::Mat &correlation, cv::Point *loc);
        void transform_fourier_space(const cv::Mat &frame, cv::Mat &dft, bool preprocess = false);
        void generate_perturbations(const cv::Mat &tracking_window, const cv::Mat &gaussian, cv::Mat perturbations[8], cv::Mat target_aff[8]);
        void preprocess(const cv::Mat &frame, cv::Mat &dst);
        void seek(const cv::Mat &frame, const cv::Mat &filter, cv::Point *loc, cv::Mat &peak_dft, int w = 3);
};

#endif
