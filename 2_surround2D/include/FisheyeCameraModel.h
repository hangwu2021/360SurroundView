#ifndef FISHEYECAMERAMODEL_H_
#define FISHEYECAMERAMODEL_H_

#include "ParamSettings.h"

class FisheyeCameraModel 
{
public:
    FisheyeCameraModel(std::string camera_param_file, std::string camera_name);
    
public:
    void load_camera_params();
    void update_undistort_maps();
    void set_scale_and_shift(cv::Vec2f scale_xy=cv::Vec2f(1.0, 1.0), cv::Vec2f shift_xy=cv::Vec2f(0, 0));
    void undistort(cv::Mat image);
    
    void save_data();
    
private:
    std::string     camera_file;
    std::string     camera_name;
    cv::Vec2f       scale_xy;
    cv::Vec2f       shift_xy;
    cv::Mat         undistort_maps;
    cv::Mat         project_matrix;
    cv::Vec2f       project_shape;
    
private:
    cv::Mat         camera_matrix;
    cv::Mat         dist_coeffs;
    cv::Vec2i       resolution;
    cv::Mat         map1;
    cv::Mat         map2;

private:
    ParamSettings   settings;
};

#endif  // FISHEYECAMERAMODEL_H_
