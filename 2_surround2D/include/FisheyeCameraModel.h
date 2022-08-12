#ifndef FISHEYECAMERAMODEL_H_
#define FISHEYECAMERAMODEL_H_

#include "ParamSettings.h"

class FisheyeCameraModel 
{
public:
    FisheyeCameraModel(const std::string &camera_name);
    
public:
    void load_camera_params();
    void update_undistort_maps();
    void set_scale_and_shift(cv::Vec2f scale_xy=cv::Vec2f(1.0, 1.0), cv::Vec2f shift_xy=cv::Vec2f(0, 0));
    void undistort_remap(const cv::Mat &image, cv::Mat &result);
    void project_warp_perspective(const cv::Mat &image, cv::Mat &result);
    void save_data();
    
private:
    std::string     camera_file;
    std::string     camera_name;
    cv::Mat         scale_xy;
    cv::Mat         shift_xy;
    cv::Mat         undistort_maps;
    cv::Mat         project_matrix;
    cv::Vec2f       project_shape;
    
private:
    cv::Mat         camera_matrix;
    cv::Mat         dist_coeffs;
    cv::Mat         resolution;
    cv::Mat         map1;
    cv::Mat         map2;

private:
    ParamSettings   params;
};

#endif  // FISHEYECAMERAMODEL_H_
