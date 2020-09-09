#include "opencv2/core/base.hpp"
#include <cstdio>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void transform_fourier_space(const cv::Mat& gray, cv::Mat& dst) {
    //Get optimal FFT size (usually powers of 2)
    int w = cv::getOptimalDFTSize(gray.cols);
    int h = cv::getOptimalDFTSize(gray.rows);

    //Pad with zeroes around optimal matrix
    cv::Mat padded;
    cv::copyMakeBorder(gray, padded, 0, h - gray.rows, 0, w - gray.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    //Make Matrix with 2 channels for real and complex parts
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32FC1)};
    cv::merge(planes, 2, dst);

    //perform fft
    cv::dft(dst, dst);
}

void shift_fourier_space(const cv::Mat& dft, cv::Mat& dst) {

    //This is just rearranging so DC term is in the middle of image (not really necessary, just for visually seeing)
    //Note, we bitwise and with 2 in order to get rid of odd rows
    dst = dft(cv::Rect(0, 0, dft.cols & -2, dft.rows & -2));

    int cx = dst.cols/2;
    int cy = dst.rows/2;

    cv::Mat q0(dst, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(dst, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(dst, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(dst, cv::Rect(cx, cy, cx, cy));

    cv::Mat temp;
    q0.copyTo(temp);
    q3.copyTo(q0);
    temp.copyTo(q3);

    q1.copyTo(temp);
    q2.copyTo(q1);
    temp.copyTo(q2);
}


void generate_magnitude_spectrum(const cv::Mat& dft, cv::Mat& dst) {

    //Split and get the magnitude of the complex fourier output
    cv::Mat planes[2];
    cv::split(dft, planes);
    cv::magnitude(planes[0], planes[1], dst);

    //Don't do log(0) and convert to logarithmic scale
    dst += cv::Scalar::all(1);
    cv::log(dst, dst);

    //Normalize to the min and max bounds so we can see (again not necessary)
    cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
}

int main(int argc, char ** argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }
    const char * vName = argv[1];

    cv::VideoCapture cap(vName);
    cv::Mat frame;

    while(true) {
        cap >> frame;
        if(frame.empty()) break;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat dft, dft_shift, m_s;
        transform_fourier_space(gray, dft);
        shift_fourier_space(dft, dft_shift);

        int cx = dft_shift.cols/2;
        int cy = dft_shift.rows/2;
        int radius = 100;
        cv::circle(dft_shift, cv::Point(cx, cy), radius/2, cv::Scalar(0), -1, cv::LINE_AA);
        cv::Mat idft_shift, idft;
        shift_fourier_space(dft_shift, idft_shift);
        cv::dft(idft_shift, idft, cv::DFT_INVERSE + cv::DFT_SCALE);
        cv::Mat planes[2];
        cv::split(idft, planes);
        cv::magnitude(planes[0], planes[1], idft);
        cv::normalize(idft, idft, 0, 1, cv::NORM_MINMAX);
        //generate_magnitude_spectrum(dft_shift, m_s);

        cv::imshow("rocket", gray);
        cv::imshow("magnitude_spectrum", idft);
        if(cv::waitKey(33) == 113) {
            break;
        }
    }
    return 0;
}
