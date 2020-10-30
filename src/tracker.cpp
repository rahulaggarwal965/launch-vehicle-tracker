#include "opencv2/core.hpp"
#include <ios>
#include <tracker.h>

//TODO: (BREAKING) might change to double precision to avoid inverse dft issues

Tracker::Tracker(const cv::Size &tracking_window_size, double learning_rate, double epsilon) :
    tracking_window_size(tracking_window_size),
    learning_rate(learning_rate),
    epsilon(epsilon) {}

void Tracker::initialize(const cv::Mat &frame, int x, int y) {

    //Choose initial window.
    prev_x = x; prev_y = y;
    cv::Mat tracking_window = frame(cv::Rect(
                x - tracking_window_size.width / 2,
                y - tracking_window_size.height / 2,
                tracking_window_size.width,
                tracking_window_size.height
                ));

    //Generate synthetic target
    cv::Mat synth_target, fourier_synth_target;
    generate_gaussian(synth_target,
            tracking_window_size.height,
            tracking_window_size.width,
            tracking_window_size.width / 2,
            tracking_window_size.height / 2
            );
    transform_fourier_space(synth_target, fourier_synth_target);

    // Get perturbations
    cv::Mat perturbations[8];
    cv::Mat target_aff[8];
    generate_perturbations(tracking_window, synth_target, perturbations, target_aff);

    //Initialize N and D with normal image.
    cv::Mat preprocessed_tracking_window;
    transform_fourier_space(tracking_window, preprocessed_tracking_window, true);

    cv::mulSpectrums(fourier_synth_target, preprocessed_tracking_window, N, 0,
            true);
    cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);
    //TODO: better way of regularization
    D += epsilon; //So we don't divide by 0 accidently.

    //Update with perturbed image.
    for (int i = 0; i < 8; i++) {
        transform_fourier_space(perturbations[i], preprocessed_tracking_window, true);

        cv::Mat N, D;
        cv::mulSpectrums(fourier_synth_target, preprocessed_tracking_window, N, 0, true);
        cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);
        D += epsilon; // So we don't divide by 0 accidently.

        this->N = this->learning_rate * N + (1 - this->learning_rate) * this->N;
        this->D = this->learning_rate * (D + epsilon) + (1 - this->learning_rate) * this->D;
    }
}

void Tracker::update(const cv::Mat &frame) {
    cv::Mat tracking_window = frame(cv::Rect(prev_x - tracking_window_size.width / 2, prev_y - tracking_window_size.height / 2, tracking_window_size.width, tracking_window_size.height));
    cv::Mat preprocessed_tracking_window;
    /* preprocess(tracking_window, preprocessed_tracking_window); */
    transform_fourier_space(tracking_window, preprocessed_tracking_window, true);
    cv::Mat filter;
    // NOTE: gives conjugate of filter
    divide_spectrums(N, D, filter);

    //NOTE: gonna say this is fine for now
    cv::Mat filter_real;
    cv::dft(filter, filter_real, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
    shift_quadrants(filter_real);
    cv::normalize(filter_real, filter_real, 0, 255.0, cv::NORM_MINMAX);
    cv::flip(filter_real, filter_real, 0);
    H = filter_real;

    // Getting new peak
    cv::Mat peak, peak_real;
    cv::mulSpectrums(filter, preprocessed_tracking_window, peak, 0);

    // Getting the gaussian peak out of fourier space
    cv::idft(peak, peak_real, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    // Finding strongest correlation point and storing x and y values
    cv::Point gaussian_peak;
    /* double max_val; */
    /* cv::minMaxLoc(peak_real, NULL, &max_val, NULL, &gaussian_peak); */
    /* printf("Found peak: %f\n", max_val); */

    double psr = compute_psr(peak_real, &gaussian_peak);
    std::ofstream psr_dump;
    psr_dump.open("psr_dump.txt", std::ios_base::app);
    psr_dump << psr << '\n';
    psr_dump.close();
    printf("PSR: %f\n", psr);

    //TODO: do peak to sidelobe test before doing gaussian again
    cv::Mat new_peak, new_peak_F;
    generate_gaussian(new_peak, peak.rows, peak.cols, gaussian_peak.x, gaussian_peak.y);
    transform_fourier_space(new_peak, new_peak);

    //Seek with error
    //TODO: tune threshold (consider finding median of previous max_val and basing threshold off of that
    /* if (max_val < 0.3) { */
    /*     this->seek(frame, filter, &gaussian_peak, peak); */
    /* } */

    // Adapting the filter based on given learning rate
    cv::Mat N, D;
    cv::mulSpectrums(new_peak, preprocessed_tracking_window, N, 0, true);
    cv::mulSpectrums(preprocessed_tracking_window, preprocessed_tracking_window, D, 0, true);

    this->N = this->learning_rate * N + (1 - this->learning_rate) * this->N;
    //TODO: scalar regularization parameter? Fix
    this->D = this->learning_rate * (D + cv::Scalar(epsilon, 0)) + (1 - this->learning_rate) * this->D;

    this->prev_x += gaussian_peak.x - tracking_window_size.width / 2;
    this->prev_y += gaussian_peak.y - tracking_window_size.height / 2;

    //Only for drawing the peak
    response = peak_real.clone();
}

void Tracker::seek(const cv::Mat &frame, const cv::Mat &filter, cv::Point *loc, cv::Mat &peak_dft, int w) {
    //TODO: numeric limit?
    double best_peak = -10000;

    //TODO: integer or fractional offsets?
    for (int i = w; i >= -w; i--) {
        for (int j = w; j >= -w; j--) {
            int lx = prev_x - (2 * w + 1) * (tracking_window_size.width / 2);
            int ly = prev_y - (2 * w + 1) * (tracking_window_size.height / 2);
            cv::Mat tracking_window = frame(cv::Rect(lx, ly,
                        tracking_window_size.width,
                        tracking_window_size.height
                        ));
            cv::Mat preprocessed_tracking_window;
            transform_fourier_space(tracking_window, preprocessed_tracking_window, true);

            cv::Mat peak, peak_real;
            cv::mulSpectrums(filter, preprocessed_tracking_window, peak, 0);

            cv::idft(peak, peak_real, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

            // Finding strongest correlation point and storing x and y values
            cv::Point gaussian_peak;
            double max_val;
            cv::minMaxLoc(peak_real, NULL, &max_val, NULL, &gaussian_peak);
            if (max_val > best_peak) {
                best_peak = max_val;
                peak_dft = peak;
                loc->x = lx + gaussian_peak.x;
                loc->y = gaussian_peak.y;
            }
        }
    }
}

/*

To compute the PSR the correlation output g is split into the
peak which is the maximum value and the sidelobe which is the
rest of the pixels excluding an 11 × 11 window around the peak.
The PSR is then defined as gmax−µsl/σsl where gmax is the peak
values and µsl and σsl are the mean and standard deviation of
the sidelobe.

*/
double Tracker::compute_psr(const cv::Mat &correlation, cv::Point *loc) {
    double max_val;
    cv::minMaxLoc(correlation, NULL, &max_val, NULL, loc);
    //threshold
    int lx = fmax(0, loc->x - 5);
    int ly = fmax(0, loc->y - 5);
    int rx = fmin(correlation.cols, loc->x + 6);
    int ry = fmin(correlation.rows,  loc->y + 6);

    cv::Mat mask = cv::Mat::ones(correlation.rows, correlation.cols, CV_8UC1);
    mask(cv::Range(ly, ry), cv::Range(lx, rx)) = 0;

    cv::Scalar mean, stddev;
    cv::meanStdDev(correlation, mean, stddev, mask);

    return (max_val - mean[0]) / stddev[0];
}

void Tracker::draw(cv::Mat &frame) {
    cv::Mat H_preview, response_preview;
    H.convertTo(H_preview, CV_8UC1);
    response.convertTo(response_preview, CV_8UC1, 255.0f);

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
    dst.convertTo(dst, CV_32FC1);
    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
    dst += cv::Scalar::all(1);
    cv::log(dst, dst);

    //TODO: normalize further to mean?
    cv::Scalar mean, stddev;
    cv::meanStdDev(dst, mean, stddev);
    dst -= mean.val[0];

    dst /= cv::sum(dst.mul(dst))[0];
}

/* cv::Mat createRegularization(const cv::Mat &m) { */

/* } */

void Tracker::generate_perturbations(const cv::Mat &tracking_window, const cv::Mat &gaussian, cv::Mat perturbations[8], cv::Mat target_aff[8]) {
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
