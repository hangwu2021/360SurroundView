#ifndef PARAMSETTINGS_H_
#define PARAMSETTINGS_H_

#include "CommonInclude.h"

class ParamSettings 
{
public:
    ParamSettings();
    
public:
    cv::Mat getCarImage(const std::string &carModelFileName);
    
public:
    std::vector<std::string> camera_names;
    
public:
    int shift_w;
    int shift_h;
    
    int inn_shift_w;
    int inn_shift_h;
    
    int total_w;
    int total_h;
    
    // Four Points of Rectangle
    int xl;
    int yt;
    int xr;
    int yb;
    
public:
    std::map<std::string, cv::Vec2f> project_shapes;
    std::map<std::string, std::vector<cv::Point2f>> project_keypoints;
    
private:
    std::vector<cv::Point2f> tempPointsVec;
};

#endif  // PARAMSETTINGS_H_
