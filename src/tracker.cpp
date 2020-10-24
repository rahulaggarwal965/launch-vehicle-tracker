#include "fourier_tools.h"
#include <tracker.h>

Tracker::Tracker(const cv::Size& tracking_window_size, double learning_rate, double epsilon) {
    this->tracking_window_size = tracking_window_size;
    this->learning_rate = learning_rate;
    this->epsilon = epsilon;
}

void Tracker::initialize(const cv::Mat& frame, int x, int y) {

    //Choose initial window.
    prev_x = x; prev_y = y;
    cv::Mat tracking_window = frame(cv::Rect(x - tracking_window_size.width / 2, y - tracking_window_size.height / 2, tracking_window_size.width, tracking_window_size.height));

    //Generate synthetic target
    cv::Mat synth_target, fourier_synth_target;
    generate_gaussian(synth_target, tracking_window_size.height,
            tracking_window_size.width,
            tracking_window_size.width / 2,
            tracking_window_size.height / 2, 2, 2);
    transform_fourier_space(synth_target, fourier_synth_target, false);

    // Get perturbations
    cv::Mat perturbations[8];
    cv::Mat target_aff[8];
    generate_perturbations(tracking_window, synth_target, perturbations, target_aff);

    //Initialize N and D with normal image.
    cv::Mat preprocessed_tracking_window;
    transform_fourier_space(tracking_window, preprocessed_tracking_window);
    /* preprocess(tracking_window, preprocessed_tracking_window); */

    cv::mulSpectrums(fourier_synth_target, preprocessed_tracking_window, N, 0,
            true);
    cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);
    D += epsilon; //So we don't divide by 0 accidently.

    //Update with perturbed image.
    for (int i = 0; i < 8; i++) {
        transform_fourier_space(perturbations[i], preprocessed_tracking_window);
        /* preprocess(perturbations[i], preprocessed_tracking_window); */

        cv::Mat N, D;
        cv::mulSpectrums(fourier_synth_target, preprocessed_tracking_window, N, 0,
                true);
        cv::mulSpectrums(preprocessed_tracking_window,
                preprocessed_tracking_window, D, 0, true);
        D += epsilon; // So we don't divide by 0 accidently.

        this->N = this->learning_rate * N + (1 - this->learning_rate) * this->N;
        this->D = this->learning_rate * (D + epsilon) +
            (1 - this->learning_rate) * this->D;
    }
}

void Tracker::update(const cv::Mat& frame) {
    cv::Mat tracking_window = frame(cv::Rect(prev_x - tracking_window_size.width / 2, prev_y - tracking_window_size.height / 2, tracking_window_size.width, tracking_window_size.height));
    cv::Mat preprocessed_tracking_window;
    preprocess(tracking_window, preprocessed_tracking_window);
    transform_fourier_space(tracking_window, preprocessed_tracking_window);
    cv::Mat filter;
    // NOTE: gives conjugate of filter
    divide_spectrums(N, D, filter);

    // Getting conjugate of filter and storing it for DRAWING
    /* cv::Mat temp; */
    /* cv::Mat ones  = cv::Mat::ones(filter.rows, filter.cols, CV_32FC2); */
    /* cv::mulSpectrums(ones, filter, temp, true); */
    cv::Mat filter_real;
    cv::dft(filter, filter_real, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    shift_quadrants(filter_real);
    cv::normalize(filter_real, filter_real, 0, 255.0, cv::NORM_MINMAX);
    H = filter_real;

    // Getting new peak
    cv::Mat peak;
    cv::mulSpectrums(filter, preprocessed_tracking_window, peak, 0);

    // Adapting the filter based on given learning rate
    cv::Mat N, D;
    cv::mulSpectrums(peak, preprocessed_tracking_window, N, 0, true);
    cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);

    this->N = this->learning_rate * N + (1 - this->learning_rate) * this->N;
    this->D = this->learning_rate * (D + epsilon) + (1 - this->learning_rate) * this->D;

    // Getting the gaussian peak out of fourier space
    cv::dft(peak, peak, cv::DFT_INVERSE | cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    // Finding strongest correlation point and storing x and y values
    cv::Point gaussian_peak;
    cv::minMaxLoc(peak, NULL, NULL, NULL, &gaussian_peak);
    this->prev_x += gaussian_peak.x - tracking_window_size.width / 2;
    this->prev_y += gaussian_peak.y - tracking_window_size.height / 2;

    //Only for drawing the peak
    response = peak.clone();
}

void Tracker::draw(cv::Mat& frame) {
    cv::Mat H_preview, response_preview;
    H.convertTo(H_preview, CV_8UC1);
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

void Tracker::transform_fourier_space(const cv::Mat &frame, cv::Mat &dst, bool preprocess) {

    //Pad with zeroes around optimal matrix
    int w = cv::getOptimalDFTSize(frame.cols);
    int h = cv::getOptimalDFTSize(frame.rows);

    cv::Mat padded;
    cv::copyMakeBorder(frame, padded, 0, h - frame.rows, 0, w - frame.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    //preprocess
    if (preprocess) {
        this->preprocess(padded, padded);
    }

    cv::dft(padded, dst, cv::DFT_COMPLEX_OUTPUT);
    //Note, we bitwise and with 2 in order to get rid of odd rows
    dst = dst(cv::Rect(0, 0, dst.cols & -2, dst.rows & -2));
}

void Tracker::preprocess(const cv::Mat &frame, cv::Mat &dst) {
    cv::cvtColor(frame, dst, cv::COLOR_BGR2GRAY);
    // TODO: experiment with this (scale by 255 or no?)
    dst.convertTo(dst, CV_32FC1);
    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
    dst += cv::Scalar::all(1);
    cv::log(dst, dst);

    cv::Scalar mean, stddev;
    cv::meanStdDev(dst, mean, stddev);
    dst -= mean.val[0];

    dst /= cv::sum(dst.mul(dst))[0];
}

void Tracker::generate_perturbations(const cv::Mat& tracking_window, const cv::Mat& gaussian, cv::Mat perturbations[8], cv::Mat target_aff[8]) {
    cv::Point center(tracking_window.cols / 2, tracking_window.rows / 2);
    perturbations[0] = cv::getRotationMatrix2D(center,  -2.8, 1);
    perturbations[1] = cv::getRotationMatrix2D(center,  2.8, 1);
    perturbations[2] = cv::getRotationMatrix2D(center, -1.2, 1);
    perturbations[3] = cv::getRotationMatrix2D(center,  1.2, 1);
    perturbations[4] = cv::getRotationMatrix2D(center,  0, 0.95);
    perturbations[5] = cv::getRotationMatrix2D(center,  0, 0.97);
    perturbations[6] = cv::getRotationMatrix2D(center,  0, 1.03);
    perturbations[7] = cv::getRotationMatrix2D(center,  0, 1.05);

    for (int i = 0; i < 8; i++) {
        cv::warpAffine(gaussian, target_aff[i], perturbations[i], cv::Size(0,0));
        cv::warpAffine(tracking_window, perturbations[i], perturbations[i], cv::Size(0, 0));
    }
}
