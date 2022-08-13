#ifndef UTILSVIEW_H_
#define UTILSVIEW_H_

#include "CommonInclude.h"

class UtilsView 
{  
public:
    
    cv::Mat merge(const cv::Mat &imA, const cv::Mat &imB, const cv::Mat &G);
    cv::Mat make_white_blance(const cv::Mat &image);
    cv::Mat convert_binary_to_bool(const cv::Mat &mask);
    cv::Mat adjust_luminance(const cv::Mat &gray, const double factor);
    
    cv::Mat get_weight_mask_matrix(const cv::Mat &imA, const cv::Mat &imB, cv::Mat &overlap_mask, int dist_threshold=5);
    cv::Mat get_mask(const cv::Mat &image);
    cv::Mat get_overlap_region_mask(const cv::Mat &imA, const cv::Mat &imB);
    std::vector<cv::Point> get_outmost_polygon_boundary(const cv::Mat &img);
    double get_mean_statistisc(const cv::Mat &gray, const cv::Mat &mask_bool);
    double mean_luminance_ratio(const cv::Mat &grayA, const cv::Mat &grayB, const cv::Mat &mask_bool);
    void make_luminace_balance(cv::Mat &image);
    
private:
    double tune(double x);
};

class ConstPointComp
{
public:
    bool operator()(const std::vector<cv::Point> &l, const std::vector<cv::Point> &r)
    {
        return cv::contourArea(l) > cv::contourArea(r);
    }
};

#endif  // UTILSVIEW_H_
