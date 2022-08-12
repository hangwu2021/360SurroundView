#include "FisheyeCameraModel.h"

FisheyeCameraModel::FisheyeCameraModel(const std::string &camera_name)
{
    this->camera_name = camera_name;
    
    this->scale_xy = cv::Vec2f(1.0, 1.0);
    this->shift_xy = cv::Vec2f(0.0, 0.0);
    
    this->project_shape = params.project_shapes[camera_name];
}

void FisheyeCameraModel::load_camera_params()
{
    camera_file = "config/fisheye/" + camera_name + ".yaml";
    
    cv::FileStorage fs;
    fs.open(camera_file, cv::FileStorage::READ);
    
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs["resolution"] >> resolution;
    fs["project_matrix"] >> project_matrix;
    fs["scale_xy"] >> scale_xy;
    fs["shift_xy"] >> shift_xy;
    
    fs.release();
}

void FisheyeCameraModel::update_undistort_maps()
{
    // camera inner params
    cv::Mat new_matrix = camera_matrix.clone();
    
    new_matrix.at<double>(0, 0) *= scale_xy.at<float>(0, 0);
    new_matrix.at<double>(1, 1) *= scale_xy.at<float>(1, 0);
    new_matrix.at<double>(0, 2) += shift_xy.at<float>(0, 0);
    new_matrix.at<double>(1, 2) += shift_xy.at<float>(1, 0);
    
    // Resolution
    int width = resolution.at<int>(0, 0), height = resolution.at<int>(1, 0);
    
    cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat::eye(cv::Size(3, 3), CV_32F), new_matrix, cv::Size(width, height), CV_32FC1, map1, map2);
}

void FisheyeCameraModel::set_scale_and_shift(cv::Vec2f scale_xy, cv::Vec2f shift_xy)
{
    this->scale_xy = scale_xy;
    this->shift_xy = shift_xy;
    
    update_undistort_maps();
}

void FisheyeCameraModel::undistort_remap(const cv::Mat &image, cv::Mat &result)
{
    if (!map1.data || !map2.data)
    {
        std::cerr << "map1 or map2 is empty!" << std::endl;
        return ;
    }
    
    cv::remap(image, result, map1, map2, cv::INTER_LINEAR);
}

void FisheyeCameraModel::project_warp_perspective(const cv::Mat &image, cv::Mat &result)
{
    cv::warpPerspective(image, result, project_matrix, cv::Size(project_shape[0], project_shape[1]));
    
    // Rotation Result
    if ("back" == camera_name)
    {
        cv::flip(result, result, -1);
    }
    else if ("left" == camera_name)
    {
        cv::transpose(result, result);
        cv::flip(result, result, 0);
    }
    else if ("right" == camera_name)
    {
        cv::transpose(result, result);
        cv::flip(result, result, 1);
    }
    else 
    {
        ;   // front camera: do nothing.
    }
    
}

void FisheyeCameraModel::save_data()
{
    cv::FileStorage fs;
    fs.open(this->camera_file, cv::FileStorage::WRITE);
    
    fs << "camera_matrix" << this->camera_matrix;
    fs << "dist_coeffs" << this->dist_coeffs;
    fs << "resolution" << this->resolution;
    fs << "project_matrix" << this->project_matrix;
    fs << "scale_xy" << this->scale_xy;
    fs << "shift_xy" << this->shift_xy;
    
    fs.release();
}
