#ifndef BIRDVIEW_H_
#define BIRDVIEW_H_

#include "ParamSettings.h"
#include "UtilsView.h"

class BirdView 
{
public:
    BirdView(cv::Mat &car_model);
    
public:
    void add_4frames(cv::Mat &front_image, cv::Mat &back_image, cv::Mat &left_image, cv::Mat &right_image);
    
public:
    cv::Mat FI();
    cv::Mat FM();
    cv::Mat FII();
    
    cv::Mat BIII();
    cv::Mat BM();
    cv::Mat BIV();
    
    cv::Mat LI();
    cv::Mat LM();
    cv::Mat LIII();
    
    cv::Mat RII();
    cv::Mat RM();
    cv::Mat RIV();
    
    cv::Mat stitch_all_parts();
    
private:
    ParamSettings           params;
    UtilsView               uv;
    
    cv::Mat                 front_image;
    cv::Mat                 back_image;
    cv::Mat                 left_image;
    cv::Mat                 right_image;
    cv::Mat                 car_model;
    cv::Mat                 final_image;    // final bird view image
    
};

#endif  // BIRDVIEW_H_
