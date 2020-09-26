#include "opencv2/core.hpp"
#include <tracker.h>

Tracker::Tracker(const cv::Size& tracking_window_size, double learning_rate, double epsilon) {
    this->tracking_window_size = tracking_window_size;
    this->learning_rate = learning_rate;
    this->epsilon = epsilon;
}

void Tracker::initialize(const cv::Mat& frame, int x, int y) {
    prev_x = x; prev_y = y;
    cv::Mat tracking_window = frame(cv::Rect(x - tracking_window_size.width / 2, y - tracking_window_size.height / 2, tracking_window_size.width, tracking_window_size.height));
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

void Tracker::update(const cv::Mat& frame) {
    cv::Mat tracking_window = frame(cv::Rect(prev_x - tracking_window_size.width / 2, prev_y - tracking_window_size.height / 2, tracking_window_size.width, tracking_window_size.height));
    cv::Mat preprocessed_tracking_window;
    preprocess_tracking_window(tracking_window, preprocessed_tracking_window);
    cv::Mat filter;
    divide_spectrums(N, D, filter);

    //for drawing
    cv::Mat temp;
    cv::Mat ones  = cv::Mat::ones(filter.rows, filter.cols, CV_32FC2);
    cv::mulSpectrums(ones, filter, temp, true);
    cv::dft(temp, temp, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);
    cv::normalize(temp, temp, 0, 1, cv::NORM_MINMAX);
    H = temp;

    cv::Mat peak;
    cv::mulSpectrums(filter, preprocessed_tracking_window, peak, 0);
    cv::dft(peak, peak, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    cv::Point gaussian_peak;

    cv::minMaxLoc(peak, NULL, NULL, NULL, &gaussian_peak);
    this->prev_x += gaussian_peak.x - tracking_window_size.width / 2;
    this->prev_y += gaussian_peak.y - tracking_window_size.height / 2;

    //For drawing
    response = peak.clone();
}

void Tracker::draw(cv::Mat& frame) {
    cv::Mat H_preview, response_preview;
    H.convertTo(H_preview, CV_8UC1, 255.0f);
    //cv::resize(H_preview, H_preview, cv::Size(), 2);
    response.convertTo(response_preview, CV_8UC1, 255.0f);
    //cv::resize(response_preview, response_preview, cv::Size(), 2);

    //cv::resize(H, H_preview, cv::Size(), 2);
    //cv::resize(response, response_preview, cv::Size(), 2);

    int padding = 32;
    H_preview.copyTo(frame(cv::Rect(padding, frame.rows - padding - H_preview.rows, H_preview.cols, H_preview.rows)));
    response_preview.copyTo(frame(cv::Rect(2 * padding + H_preview.cols, frame.rows - padding - response_preview.rows, response_preview.cols, response_preview.rows)));
    cv::rectangle(frame, cv::Rect(this->prev_x - this->tracking_window_size.width / 2, this->prev_y - this->tracking_window_size.height / 2, this->tracking_window_size.width, this->tracking_window_size.height), cv::Scalar(0, 0, 0));
    cv::circle(frame, cv::Point(prev_x, prev_y), 3, cv::Scalar(0, 0, 255));
}

void Tracker::preprocess_tracking_window(const cv::Mat& tracking_window, cv::Mat& dst) {
    cv::cvtColor(tracking_window, dst, cv::COLOR_BGR2GRAY);
    //TODO: experiment with this
    dst.convertTo(dst, CV_32FC1);
    cv::log(dst + 1, dst);
    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
    transform_fourier_space(dst, dst);
}
