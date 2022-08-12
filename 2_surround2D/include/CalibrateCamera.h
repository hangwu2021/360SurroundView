#ifndef CALIBRATECAMERA_H_
#define CALIBRATECAMERA_H_

#include "ParamSettings.h"

class CalibrateCamera
{
public:
    CalibrateCamera(const std::string &camera_name, const cv::Size &board_size, const cv::Size &square_size);
    ~CalibrateCamera();
    
    void extract_corners();
    void calibrate_process();
    void calc_project_matrix();
    void save_calibrated_yaml();

private:
    // on Mouse Event
    static void onMouse(int event, int x, int y, int flags, void *param);
    
private:
    //static int                              bird_point_x;
    //static int                              bird_point_y;
    static std::vector<cv::Point2f>         bird_4points;
    ParamSettings                           params;
    
private:
    std::string                             camera_name;
    cv::Size                                board_size;
    cv::Size                                square_size;        // Real Block SIze
    cv::Mat                                 image_input;
    std::vector<std::vector<cv::Point2f>>   image_corners;
    std::vector<std::vector<cv::Point3f>>   object_points;
    cv::Mat                                 project_matrix;     // Bird View Matrix
    cv::Mat                                 scale_xy;
    cv::Mat                                 shift_xy;
    
    bool                                    isFishEye;
    
    // fisheye 
    cv::Matx33d fisheye_intrinsic_matrix;
    cv::Vec4d fisheye_distortion_coeffs;
    std::vector<cv::Vec3d> fisheye_rotation_vectors;
    std::vector<cv::Vec3d> fisheye_translation_vectors;
    
    // nonfisheye
    cv::Mat nonfisheye_intrinsic_matrix;
    cv::Mat nonfisheye_distortion_coeffs;
    std::vector<cv::Mat> nonfisheye_rotation_vectors;
    std::vector<cv::Mat> nonfisheye_translation_vectors;
};

#endif  // CALIBRATECAMERA_H_
