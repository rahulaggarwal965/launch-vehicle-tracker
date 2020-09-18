#include "opencv2/core.hpp"
#include "opencv2/core/base.hpp"
#include <tracker.h>

Tracker::Tracker(const cv::Size& tracking_window_size, double learning_rate, double epsilon) {
    this->tracking_window_size = tracking_window_size;
    this->learning_rate = learning_rate;
    this->epsilon = epsilon;
}

void Tracker::initialize(const cv::Mat& frame, int x, int y) {
    prev_x = x; prev_y = y;
    cv::Mat tracking_window = frame(cv::Rect(x, y, tracking_window_size.width, tracking_window_size.height));
    imhold(tracking_window, "Bounding box");
    cv::Mat preprocessed_tracking_window;
    preprocess_tracking_window(tracking_window, preprocessed_tracking_window);
    cv::Mat synth_target;
    generate_gaussian(synth_target, tracking_window_size.height, tracking_window_size.width, tracking_window_size.width/2, tracking_window_size.height/2, 2, 2);
    transform_fourier_space(synth_target, synth_target);

    //TODO: perturbations
    cv::mulSpectrums(synth_target, preprocessed_tracking_window, N, 0, true);
    cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);
    D += epsilon; //So we don't divide by 0 accidently.

}

void Tracker::update(const cv::Mat &frame) {
    cv::Mat tracking_window = frame(cv::Rect(prev_x, prev_y, tracking_window_size.width, tracking_window_size.height));
    cv::Mat preprocessed_tracking_window;
    preprocess_tracking_window(tracking_window, preprocessed_tracking_window);
    cv::Mat filter;
    divide_spectrums(N, D, filter);

    //cv::Mat temp;
    //cv::Mat ones  = cv::Mat::ones(filter.rows, filter.cols, CV_32FC2);
    //cv::mulSpectrums(ones, filter, temp, true);
    //cv::dft(temp, temp, cv::DFT_INVERSE + cv::DFT_SCALE);

    //cv::Mat channels[2];
    //cv::split(temp, channels);
    //imhold(channels[0], "response");

    cv::Mat peak;
    cv::mulSpectrums(filter, preprocessed_tracking_window, peak, 0);
    cv::dft(peak, peak, cv::DFT_INVERSE + cv::DFT_SCALE);
    cv::Mat channels[2];
    cv::split(peak, channels);
    imhold(channels[0], "response");
}

void Tracker::preprocess_tracking_window(const cv::Mat& tracking_window, cv::Mat& dst) {
    cv::cvtColor(tracking_window, dst, cv::COLOR_BGR2GRAY);
    dst.convertTo(dst, CV_32FC1);
    cv::log(dst + 1, dst);
    cv::normalize(dst, dst, cv::NORM_MINMAX);
    transform_fourier_space(dst, dst);
}
