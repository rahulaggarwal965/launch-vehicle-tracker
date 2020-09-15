#ifndef fourier_tools_h
#define fourier_tools_h

#include "opencv2/core/base.hpp"
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

void generate_gaussian(cv::Mat& dst, int rows, int cols, int uX, int uY, float sigmaX, float sigmaY, float amplitude = 1) {
    cv::Mat temp(rows, cols, CV_32FC1); //guaranteed continuous;
    for (int j = 0; j < rows; j++) {
        float * elementPtr = temp.ptr<float>(j);
        for (int i = 0; i < cols; i++) {
            float x = (float) (i - uX) * (i - uX) / (2.0f * sigmaX * sigmaX);
            float y = (float) (j - uY) * (j - uY) / (2.0f * sigmaY * sigmaY);
            elementPtr[i] = amplitude*exp(-x - y);//A
        }
    }
    normalize(temp, temp, 0.0f, 1.0f, cv::NORM_MINMAX);
    dst = temp; //copies header only;
}



#endif
