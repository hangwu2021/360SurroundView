#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include "myStitch/common_include.hpp"

namespace myStitch
{

class Homography 
{
public:
    Homography();
    Homography(cv::Mat img1, cv::Mat img2);
    
    std::vector<cv::KeyPoint> getKeyPoints1();
    std::vector<cv::KeyPoint> getKeyPoints2();
    
    cv::Mat getDescriptors1();
    cv::Mat getDescriptors2();
    
    std::vector<cv::DMatch> getMatches();
    
    void drawMatches();
    
    cv::Mat getHomography();
    
    ~Homography();

private:
    void detectKeyPoints();
    void computeDescriptors();
    void match();
    void matches2SelfPoints();
    void findHomography();
    void matchesFilter();
    
private:
    cv::Mat img1, img2;
    
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;
    cv::Ptr<cv::DescriptorMatcher> matcher;
    
    std::vector<cv::KeyPoint> keyPoints1;
    std::vector<cv::KeyPoint> keyPoints2;
    
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    
    std::vector<cv::DMatch> firstMatches;
    std::vector<cv::DMatch> matches;
    
    std::vector<cv::Point2f> selfPoints1;
    std::vector<cv::Point2f> selfPoints2;
    
    std::vector<uchar> inliers;
    
    cv::Mat homography;
};

} // namespace myStitch

#endif // HOMOGRAPHY_H_
