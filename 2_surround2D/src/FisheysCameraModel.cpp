#include "FisheyeCameraModel.h"

FisheyeCameraModel::FisheyeCameraModel(std::string camera_param_file, std::string camera_name)
{
    this->camera_file = camera_param_file;
    this->camera_name = camera_name;
    this->scale_xy = cv::Vec2f(1.0, 1.0);
    this->shift_xy = cv::Vec2f(0.0, 0.0);
    
    this->project_shape = this->settings.project_shapes[this->camera_name];
}

void FisheyeCameraModel::load_camera_params()
{
    cv::FileStorage fs;
    fs.open(this->camera_file, cv::FileStorage::READ);
    
    fs["camera_matrix"] >> this->camera_matrix;
    fs["dist_coeffs"] >> this->dist_coeffs;
    fs["resolution"] >> this->resolution;
    fs["scale_xy"] >> this->scale_xy;
    fs["shift_xy"] >> this->shift_xy;
    
    cv::Mat tempProjectMatrix;
    fs["project_matrix"] >> tempProjectMatrix;
    if (tempProjectMatrix.data != nullptr)
    {
        this->project_matrix = tempProjectMatrix;
    }
    
    fs.release();
}

void FisheyeCameraModel::update_undistort_maps()
{
    // camera inner params
    cv::Mat new_matrix = camera_matrix.clone();
    
    new_matrix.at<double>(0, 0) *= scale_xy[0];
    new_matrix.at<double>(1, 1) *= scale_xy[1];
    new_matrix.at<double>(0, 2) += shift_xy[0];
    new_matrix.at<double>(1, 2) += shift_xy[1];
    
    // Resolution
    int width = resolution[0], height = resolution[1];
    
    cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat::eye(cv::Size(3, 3), CV_32F), new_matrix, cv::Size(width, height), CV_32FC1, map1, map2);
}

void FisheyeCameraModel::set_scale_and_shift(cv::Vec2f scale_xy, cv::Vec2f shift_xy)
{
    this->scale_xy = scale_xy;
    this->shift_xy = shift_xy;
    
    update_undistort_maps();
}

void FisheyeCameraModel::undistort(cv::Mat &image, cv::Mat &result)
{
    cv::remap(image, result, map1, map2, cv::INTER_LINEAR);
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

