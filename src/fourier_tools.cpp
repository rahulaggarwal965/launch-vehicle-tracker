#include "fourier_tools.h"

void shift_quadrants(cv::Mat& dst) {

  // This is just rearranging so DC term is in the middle of image (not really
  // necessary, just for visually seeing

  int cx = dst.cols / 2;
  int cy = dst.rows / 2;

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

void generate_gaussian(cv::Mat& dst, int rows, int cols, int uX, int uY, float sigmaX, float sigmaY, float amplitude) {
    cv::Mat temp(rows, cols, CV_32FC1); //guaranteed continuous; use i, and j for ease of reading code
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

void divide_spectrums(const cv::Mat& comp, const cv::Mat& re, cv::Mat& dst) {
    //TODO: assert that their size is equal
    int rows = comp.rows, cols = comp.cols;
    cv::Mat temp(rows, cols, CV_32FC2); //guaranteed continuous
    if (comp.isContinuous() && re.isContinuous()) {
        cols *= rows;
        rows = 1;
    }
    for (int j = 0; j < rows; j++) {
        const cv::Vec2f * compRowPtr = comp.ptr<cv::Vec2f>(j);
        const cv::Vec2f * reRowPtr = re.ptr<cv::Vec2f>(j);
        cv::Vec2f * tempRowPtr = temp.ptr<cv::Vec2f>(j);
        for (int i = 0; i < cols; i++) {
            tempRowPtr[i][0] = compRowPtr[i][0]/reRowPtr[i][0];
            tempRowPtr[i][1] = compRowPtr[i][1]/reRowPtr[i][0];
        }
    }
    dst = temp; //copy header
}

//temp
void imhold(const cv::Mat& image, const char * windowName) {
    while(true) {
        cv::imshow(windowName, image);
        if (cv::waitKey(33) == 113) {
            break;
        }
    }
}
