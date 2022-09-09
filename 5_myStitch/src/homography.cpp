#include "myStitch/homography.hpp"

namespace myStitch
{

Homography::Homography()
{
    this->detector = cv::SIFT::create(800);
    this->extractor = detector;
    this->matcher = cv::DescriptorMatcher::create("BruteForce");
}

Homography::Homography(cv::Mat img1, cv::Mat img2)
{
    new(this) Homography();
    this->img1 = img1;
    this->img2 = img2;
}

std::vector<cv::KeyPoint> Homography::getKeyPoints1()
{
    if (keyPoints1.size() == 0)
    {
        detectKeyPoints();
    }
    
    return keyPoints1;
}

std::vector<cv::KeyPoint> Homography::getKeyPoints2()
{
    if (keyPoints2.size() == 0)
    {
        detectKeyPoints();
    }
    
    return keyPoints2;
}

cv::Mat Homography::getDescriptors1()
{
    if (descriptors1.data == nullptr)
    {
        computeDescriptors();
    }
    
    return descriptors1;
}

cv::Mat Homography::getDescriptors2()
{
    if (descriptors2.data == nullptr)
    {
        computeDescriptors();
    }
    
    return descriptors2;
}

std::vector<cv::DMatch> Homography::getMatches()
{
    if (matches.size() == 0)
    {
        matchesFilter();
    }
    
    return matches;
}

cv::Mat Homography::getHomography()
{
    if (homography.data == nullptr)
    {
        findHomography();
    }
    
    return homography;
}

void Homography::drawMatches()
{
    cv::Mat matchImage;
    if (matches.size() == 0)
    {
        matchesFilter();
    }
    
    cv::drawMatches(img1, keyPoints1, img2, keyPoints2, matches, matchImage);
    cv::imshow("drawMatches", matchImage);
}

void Homography::detectKeyPoints()
{
    detector->detect(img1, keyPoints1, cv::Mat());
    detector->detect(img2, keyPoints2, cv::Mat());
}

void Homography::computeDescriptors()
{
    if (keyPoints1.size() == 0 || keyPoints2.size() == 0)
    {
        detectKeyPoints();
    }
    
    extractor->compute(img1, keyPoints1, descriptors1);
    extractor->compute(img2, keyPoints2, descriptors2);
}

void Homography::match()
{
    if (descriptors1.data == nullptr || descriptors2.data == nullptr)
    {
        computeDescriptors();
    }
    
    matcher->match(descriptors1, descriptors2, firstMatches, cv::Mat());
}

void Homography::matches2SelfPoints()
{
    for (std::vector<cv::DMatch>::const_iterator it = firstMatches.begin(); it != firstMatches.end(); ++it)
    {
        selfPoints1.push_back(keyPoints1[it->queryIdx].pt);
        selfPoints2.push_back(keyPoints2[it->trainIdx].pt);
    }
}

void Homography::findHomography()
{
    if (firstMatches.size() == 0)
    {
        match();
    }
    
    if (selfPoints1.size() == 0 || selfPoints2.size() == 0)
    {
        matches2SelfPoints();
    }
    
    inliers = std::vector<uchar>(selfPoints1.size(), 0);
    homography = cv::findHomography(selfPoints1, selfPoints2, inliers, cv::FM_RANSAC, 1.0);
}

void Homography::matchesFilter()
{
    if (firstMatches.size() == 0)
    {
        findHomography();
    }
    
    std::vector<cv::DMatch>::const_iterator itM = firstMatches.begin();
    std::vector<uchar>::const_iterator itIn = inliers.begin();
    
    for (; itIn != inliers.end(); ++itIn, ++itM)
    {
        if (*itIn)
        {
            matches.push_back(*itM);
        }
    }
}

Homography::~Homography()
{
    
}

} // namespace myStitch
