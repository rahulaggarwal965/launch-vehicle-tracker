#ifndef fourier_tools_h
#define fourier_tools_h

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void shift_quadrants(cv::Mat &dst);

void generate_magnitude_spectrum(const cv::Mat& dft, cv::Mat& dst);

void generate_gaussian(cv::Mat& dst, int rows, int cols, int uX, int uY, float sigmaX, float sigmaY, float amplitude = 1);

void divide_spectrums(const cv::Mat& comp, const cv::Mat& re, cv::Mat& dst);

//temp
void imhold(const cv::Mat& image, const char * windowName);

#endif
