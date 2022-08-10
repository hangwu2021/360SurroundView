#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./main image.png" << std::endl;
        return -1;
    }
    
    const int64 start = cv::getTickCount();
    
    cv::Mat image, gray;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 1.5);
    cv::Canny(gray, gray, 0, 50);
    
    double duration = (cv::getTickCount() - start) / cv::getTickFrequency();
    
    std::cout << "duration = " << duration << " seconds." << std::endl;
    
    return 0;
}
