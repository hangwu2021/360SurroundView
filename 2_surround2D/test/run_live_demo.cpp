#include "FisheyeCameraModel.h"
#include "CalibrateCamera.h"
#include "BirdView.h"

#include "UtilsView.h"

int main(int argc, char *argv[])
{
    // Test Instance CalibrateCamera
    /*CalibrateCamera camera0(argv[1], cv::Size(9, 6), cv::Size(10, 10));
    
    camera0.extract_corners();
    camera0.calibrate_process();
    camera0.calc_project_matrix();
    camera0.save_calibrated_yaml();
    */
    // Test Instance FisheyeCameraModel
    FisheyeCameraModel camera_front("front");
    cv::Mat image_front, undistorted_result_front, perspective_result_front;
    
    image_front = cv::imread("data/images/fisheye/front/front.png");
    camera_front.load_camera_params();
    camera_front.update_undistort_maps();    // Must mapx and mapy for remap function
    camera_front.undistort_remap(image_front, undistorted_result_front);
    camera_front.project_warp_perspective(undistorted_result_front, perspective_result_front);
    
    // camera back
    FisheyeCameraModel camera_back("back");
    cv::Mat image_back, undistorted_result_back, perspective_result_back;
    
    image_back = cv::imread("data/images/fisheye/back/back.png");
    camera_back.load_camera_params();
    camera_back.update_undistort_maps();
    camera_back.undistort_remap(image_back, undistorted_result_back);
    camera_back.project_warp_perspective(undistorted_result_back, perspective_result_back);
    
    // camera left
    FisheyeCameraModel camera_left("left");
    cv::Mat image_left, undistorted_result_left, perspective_result_left;
    
    image_left = cv::imread("data/images/fisheye/left/left.png");
    camera_left.load_camera_params();
    camera_left.update_undistort_maps();
    camera_left.undistort_remap(image_left, undistorted_result_left);
    camera_left.project_warp_perspective(undistorted_result_left, perspective_result_left);
    
    // camera right
    FisheyeCameraModel camera_right("right");
    cv::Mat image_right, undistorted_result_right, perspective_result_right;
    
    image_right = cv::imread("data/images/fisheye/right/right.png");
    camera_right.load_camera_params();
    camera_right.update_undistort_maps();
    camera_right.undistort_remap(image_right, undistorted_result_right);
    camera_right.project_warp_perspective(undistorted_result_right, perspective_result_right);
    
    cv::Mat car_model;
    car_model = cv::imread("data/images/fisheye/car.png");
    
    // Generate Bird View Final Image
    BirdView bv(car_model);
    bv.add_4frames(perspective_result_front, perspective_result_back, perspective_result_left, perspective_result_right);
    
    UtilsView uv;
    /*cv::imshow("balance", uv.make_white_blance(bv.FM()));
    cv::Mat result = bv.stitch_all_parts();
    cv::imshow("fusion_result", result);
    cv::waitKey(0);*/
    
    cv::Mat stitced_img = bv.stitch_all_parts();
    uv.make_luminace_balance(stitced_img);
    
    cv::imwrite("data/images/fisheye/stitch_result.png", stitced_img);
    
    return 0;
}
