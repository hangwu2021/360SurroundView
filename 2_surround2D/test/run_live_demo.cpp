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
    // camera front
    FisheyeCameraModel camera_front("front");
    camera_front.load_camera_params();
    camera_front.update_undistort_maps();    // Must mapx and mapy for remap function
    
    cv::VideoCapture video_front("data/videos/front0810_1280.avi");
    if (!video_front.isOpened())
    {
        std::cerr << "video front is opened failed!" << std::endl;
        return -1;
    }
    
    // camera back
    FisheyeCameraModel camera_back("back");
    camera_back.load_camera_params();
    camera_back.update_undistort_maps();
    
    cv::VideoCapture video_back("data/videos/back0810_1280.avi");
    if (!video_back.isOpened())
    {
        std::cerr << "video back is opened failed!" << std::endl;
        return -1;
    }
    
    // camera left
    FisheyeCameraModel camera_left("left");
    camera_left.load_camera_params();
    camera_left.update_undistort_maps();
    
    cv::VideoCapture video_left("data/videos/left0810_1280.avi");
    if (!video_left.isOpened())
    {
        std::cerr << "video left is opened failed!" << std::endl;
        return -1;
    }
    
    // camera right
    FisheyeCameraModel camera_right("right");
    camera_right.load_camera_params();
    camera_right.update_undistort_maps();
    
    cv::VideoCapture video_right("data/videos/right0810_1280.avi");
    if (!video_right.isOpened())
    {
        std::cerr << "video right is opened failed!" << std::endl;
        return -1;
    }
    
    // car model 
    cv::Mat car_model;
    car_model = cv::imread("data/images/fisheye/car.png");
    BirdView bv(car_model);
    UtilsView uv;
    
    // stitch 4 frames
    ParamSettings params;
    cv::VideoWriter stitched_output("data/videos/stitched_output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, cv::Size(params.total_w, params.total_h));
    
    cv::Mat frame_front, undistorted_result_front, perspective_result_front;
    cv::Mat frame_back, undistorted_result_back, perspective_result_back;
    cv::Mat frame_left, undistorted_result_left, perspective_result_left;
    cv::Mat frame_right, undistorted_result_right, perspective_result_right;
    cv::Mat frame_stitched;
    
    std::cout << "number of video: " << video_front.get(CV_CAP_PROP_FRAME_COUNT) << ", " << video_front.get(CV_CAP_PROP_FRAME_WIDTH) << ", " << video_front.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "number of video: " << video_back.get(CV_CAP_PROP_FRAME_COUNT) << ", " << video_back.get(CV_CAP_PROP_FRAME_WIDTH) << ", " << video_back.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "number of video: " << video_left.get(CV_CAP_PROP_FRAME_COUNT) << ", " << video_left.get(CV_CAP_PROP_FRAME_WIDTH) << ", " << video_left.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    std::cout << "number of video: " << video_right.get(CV_CAP_PROP_FRAME_COUNT) << ", " << video_right.get(CV_CAP_PROP_FRAME_WIDTH) << ", " << video_right.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    
    bool stop = false; long cnt = 0;
    stop = (!video_front.read(frame_front)) || (!video_back.read(frame_back)) || (!video_front.read(frame_left)) || (!video_right.read(frame_right));
    while (!stop)
    {
        /*cv::resize(frame_front, frame_front, cv::Size(960, 640));
        cv::resize(frame_back, frame_back, cv::Size(960, 640));
        cv::resize(frame_left, frame_left, cv::Size(960, 640));
        cv::resize(frame_right, frame_right, cv::Size(960, 640));
        */
        camera_front.undistort_remap(frame_front, undistorted_result_front);
        camera_front.project_warp_perspective(undistorted_result_front, perspective_result_front);
        
        camera_back.undistort_remap(frame_back, undistorted_result_back);
        camera_back.project_warp_perspective(undistorted_result_back, perspective_result_back);
        
        camera_left.undistort_remap(frame_left, undistorted_result_left);
        camera_left.project_warp_perspective(undistorted_result_left, perspective_result_left);
        
        camera_right.undistort_remap(frame_right, undistorted_result_right);
        camera_right.project_warp_perspective(undistorted_result_right, perspective_result_right);
        
        bv.add_4frames(perspective_result_front, perspective_result_back, perspective_result_left, perspective_result_right);
        frame_stitched = bv.stitch_all_parts();
        /*
        cv::Mat frame_whited;
        frame_whited = uv.make_white_blance(frame_stitched);// Step5: White Balance
        
        uv.make_luminace_balance_yuv(frame_whited);
        */
        stitched_output.write(frame_stitched);
        
        cnt++; std::cout << "frame num = " << cnt << std::endl;
        stop = (!video_front.read(frame_front)) || (!video_back.read(frame_back)) || (!video_front.read(frame_left)) || (!video_right.read(frame_right));
    }
    
    video_front.release();
    video_back.release();
    video_left.release();
    video_right.release();
    stitched_output.release();
    
    //cv::imwrite("data/images/fisheye/stitch_result.png", stitced_img);
    
    return 0;
}
