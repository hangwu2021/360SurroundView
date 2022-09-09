#ifndef SEAM_CARVING_H_
#define SEAM_CARVING_H_

#include "myStitch/common_include.hpp"

#include <opencv2/stitching/detail/seam_finders.hpp>

namespace myStitch
{

class SeamCarving 
{
public:
    typedef std::shared_ptr<SeamCarving> Ptr;
    SeamCarving();
    
    void setSources(const cv::Mat& img1, const cv::Mat& img2);
    
    void setCorners(const cv::Point& corner1, const cv::Point& corner2);
    
    std::vector<cv::UMat> getMasks();
    
    cv::Mat getCanvasSeamCropped();
    
    // Way2: Optimize Seam 
    void OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst, const int start);

private:
    // Way1: SeamFinder
    cv::Ptr<cv::detail::SeamFinder> seam_finder;
    
    std::vector<cv::UMat> sources;
    std::vector<cv::Point> corners;
    std::vector<cv::UMat> masks;
};

}

#endif // SEAM_CARVING_H_
