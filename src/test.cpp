#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

void generate_magnitude_spectrum(const cv::Mat& gray, cv::Mat& dst) {

        //Get optimal FFT size (usually powers of 2)
        int w = cv::getOptimalDFTSize(gray.cols);
        int h = cv::getOptimalDFTSize(gray.rows);

        //Pad with zeroes around optimal matrix
        cv::Mat padded;
        cv::copyMakeBorder(gray, padded, 0, h - gray.rows, 0, w - gray.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

        //Make Matrix with 2 channels for real and complex parts
        cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32FC1)};
        cv::Mat complexImage;
        cv::merge(planes, 2, complexImage);

        //perform fft
        cv::dft(complexImage, complexImage);

        //Split and get the magnitude of the complex fourier output
        cv::split(complexImage, planes);
        cv::magnitude(planes[0], planes[1], planes[0]);
        dst = planes[0];

        //Don't do log(0) and convert to logarithmic scale
        dst += cv::Scalar::all(1);
        cv::log(dst, dst);

        //This is just rearranging so DC term is in the middle of image (not really necessary, just for visually seeing)
        dst = dst(cv::Rect(0, 0, dst.cols & -2, dst.rows & -2));

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

        //Normalize to the min and max bounds so we can see (again not necessary)
        cv::normalize(dst, dst, 0, 1, cv::NORM_MINMAX);
}

int main(int argc, char ** argv) {
    if (argc < 2) {
        fprintf(stderr, "Expected video file name as a command line argument");
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

        cv::Mat m_s;
        generate_magnitude_spectrum(gray, m_s);

        cv::imshow("rocket", gray);
        cv::imshow("magnitude_spectrum", m_s);
        if(cv::waitKey(33) == 113) {
            break;
        }
    }
    return 0;
}
