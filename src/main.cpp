#include "opencv2/core.hpp"
#include "opencv2/core/base.hpp"
#include "opencv2/core/hal/interface.h"
#include <cstdio>
#include "fourier_tools.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

void imhold(const cv::Mat& image, const char * windowName) {
    while(true) {
        cv::imshow(windowName, image);
        if (cv::waitKey(33) == 113) {
            break;
        }
    }
}

int main(int argc, char ** argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument\n");
        return -1;
    }
    const char * vName = argv[1];

    cv::VideoCapture cap(vName);
    cv::Mat frame;

    // Algo Outline
    // Generate initial response, gausian distro about center of mass
    // Get Function to predict response
    // Train from there

    int fc = 0;
    cv::Mat known_response, h;
    int r0x = 630, r0y = 190, r1x = 652, r1y = 310;

    cv::namedWindow("magnitude_spectrum");
    cv::createTrackbar("r0x", "magnitude_spectrum", &r0x, 1279);
    cv::createTrackbar("r0y", "magnitude_spectrum", &r0y, 519);
    cv::createTrackbar("r1x", "magnitude_spectrum", &r1x, 1279);
    cv::createTrackbar("r1y", "magnitude_spectrum", &r1y, 519);

    while(true) {
        cap >> frame;
        if(frame.empty()) break;
        fc++;

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        gray = gray(cv::Rect(0, 0, gray.cols, gray.rows - 180));

        if(fc == 1) {
            //Just for choosing the rectangle, will get abstracted out later
            while (true) {
                cv::Mat tGray = gray.clone();
                cv::rectangle(tGray, cv::Rect(r0x, r0y, r1x - r0x, r1y - r0y), cv::Scalar(255, 0, 0));
                cv::imshow("magnitude_spectrum", tGray);
                int key = cv::waitKey(33);
                if (key == 113) {
                    break;
                }
            }

            //Generate gaussian around our point with sigma 2
            generate_gaussian(known_response, gray.rows, gray.cols, (r0x + r1x) / 2, (r0y + r1y) / 2, 2.0f, 2.0f);
            //Make it g + 0i (complex)
            cv::Mat planes[] = {known_response, cv::Mat::zeros(known_response.rows, known_response.cols, CV_32FC1)};
            cv::merge(planes, 2, known_response);

            //get the dft of the gray image
            cv::Mat dft;
            transform_fourier_space(gray, dft);
            cv::Mat temp_num, temp_dom;
            //G has conj?

            cv::mulSpectrums(dft, known_response, temp_num, 0, true);
            cv::mulSpectrums(dft, dft, temp_dom, 0, true);
            h = temp_num/temp_dom;
            cv::dft(h, h, cv::DFT_INVERSE + cv::DFT_SCALE);
            cv::Mat p[2];
            cv::split(h, p);
            cv::magnitude(p[0], p[1], h);
            cv::normalize(h, h, 0, 1, cv::NORM_MINMAX);
        }


        //shift_fourier_space(dft, dft_shift);

        //int cx = dft_shift.cols/2;
        //int cy = dft_shift.rows/2;
        //int radius = 100;
        //cv::Mat mask = cv::Mat::zeros(dft_shift.rows, dft_shift.cols, CV_32FC2);
        //cv::circle(mask, cv::Point(cx, cy), radius/2, cv::Scalar(1), -1, cv::LINE_AA);
        //cv::mulSpectrums(dft_shift, mask, dft_shift, 0);
        //cv::Mat idft_shift, idft;
        //shift_fourier_space(dft_shift, idft_shift);
        //cv::dft(idft_shift, idft, cv::DFT_INVERSE + cv::DFT_SCALE);
        //cv::Mat planes[2];
        //cv::split(idft, planes);
        //cv::magnitude(planes[0], planes[1], idft);
        //cv::normalize(idft, idft, 0, 1, cv::NORM_MINMAX);
        //generate_magnitude_spectrum(dft_shift, m_s);
        cv::imshow("magnitude_spectrum", h);
        //cv::imshow("known_response", known_response);
        if(cv::waitKey(33) == 113) {
            break;
        }
    }
    return 0;
}
