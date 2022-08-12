#include "CalibrateCamera.h"

CalibrateCamera::CalibrateCamera(const std::string &camera_name, const cv::Size &board_size, const cv::Size &square_size)
{
    this->camera_name = camera_name;
    this->board_size = board_size;
    this->square_size = square_size;
    this->scale_xy = cv::Mat(2, 1, CV_32F, {1.0, 1.0});
    this->shift_xy = cv::Mat(2, 1, CV_32F, {0.0, 0.0});
    
    this->isFishEye = false;    // default value
}

CalibrateCamera::~CalibrateCamera()
{
    
}

void CalibrateCamera::extract_corners()
{
    std::cout << __func__ << std::endl;
    
    std::string image_input_path;
    
    // is fisheye or not
    if (camera_name == "front" || camera_name == "back" || camera_name == "left" || camera_name == "right")
    {
        isFishEye = true;
        image_input_path = "data/images/fisheye/" + camera_name + "/";
    }
    else
    {
        isFishEye = false;
        image_input_path = "data/images/nonfisheye/";
    }
    
    // Extract 2D Corners
    std::ifstream filelist;
    filelist.open(image_input_path + "filelist.txt", std::fstream::in);
    
    std::vector<cv::Point2f> corners;
    bool isFindChessBoardCorners;
    
    std::string filename;
    while (std::getline(filelist, filename))
    {
        std::cout << image_input_path+filename << std::endl;
        
        image_input = cv::imread(image_input_path+filename);
        
        if (image_input.data != nullptr)
        {
            isFindChessBoardCorners = cv::findChessboardCorners(image_input, board_size, corners);
            if (isFindChessBoardCorners)
            {
                cv::Mat gray;
                cv::cvtColor(image_input, gray, cv::COLOR_BGR2GRAY);
                cv::find4QuadCornerSubpix(gray, corners, cv::Size(5, 5));
                cv::drawChessboardCorners(gray, board_size, corners, false);
                
                if (!corners.empty())
                {
                    image_corners.push_back(corners);
                }
            }
            std::cout << "corners.size() = " << corners.size() << std::endl;
        }
    }
    
    filelist.close();
}

void CalibrateCamera::calibrate_process()
{
    std::cout << __func__ << std::endl;
    
    // Generate Real 3D points of Calibrate 
    std::vector<cv::Point3f> objects;
    
    if (image_corners.empty())
    {
        std::cerr << "image corners is empty." << std::endl;
        return ;
    }
    
    for (int i = 0; i < image_corners[0].size(); i++)
    {
        objects.push_back(cv::Point3f((i % board_size.width)*square_size.width, (i / board_size.width)*square_size.height, 0));
    }
    
    for (int i = 0; i < image_corners.size(); i++)
    {
        object_points.push_back(objects);
        //objects.clear();    // Must be cleared, nor overflow
    }
    
    std::cout << "image_corners.size() = " << image_corners.size() << ", object_points.size() = " << object_points.size() << std::endl;
    
    // Camera Calibrate
    if (isFishEye)
    {
        std::cout << "fisheye" << std::endl;
        
        int flag = 0;
        
        flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        //flag |= cv::fisheye::CALIB_CHECK_COND;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        
        cv::fisheye::calibrate(object_points, image_corners, image_input.size(), fisheye_intrinsic_matrix, fisheye_distortion_coeffs, fisheye_rotation_vectors, fisheye_translation_vectors, flag, cv::TermCriteria(3, 20, 1e-6));
        
        std::cout << "fisheye_intrinsic_matrix = \n" << fisheye_intrinsic_matrix << "\nfisheye_distortion_coeffs = \n" << fisheye_distortion_coeffs << std::endl;
    }
    else 
    {
        std::cout << "nonfisheye" << std::endl;
        
        nonfisheye_intrinsic_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));  // Camera Inner Parameters
        nonfisheye_distortion_coeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); // Camera distorted Parameters
        
        cv::calibrateCamera(object_points, image_corners, image_input.size(), nonfisheye_intrinsic_matrix, nonfisheye_distortion_coeffs, nonfisheye_rotation_vectors, nonfisheye_translation_vectors, 0);
        
        std::cout << "nonfisheye_intrinsic_matrix: \n" << nonfisheye_intrinsic_matrix << "\n" << "nonfisheye_distortion_coeffs: \n" << nonfisheye_distortion_coeffs << std::endl;
    }
}

// Initialize Static Member Variable
std::vector<cv::Point2f> CalibrateCamera::bird_4points = {cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0), cv::Point2f(0, 0)};

// undistored Image to Bird View Image
void CalibrateCamera::calc_project_matrix()
{
    if (!image_input.data)
    {
        std::cerr << "Input Image is Invalid." << std::endl;
        return ;
    }
    
    cv::Mat undistorted_result;
    cv::Size result_size = cv::Size(image_input.cols*1.2, image_input.rows*1.2);
    cv::Mat mapx, mapy;
    
    if (isFishEye)
    {
        std::cout << "fisheye remap..." << std::endl;
        
        cv::fisheye::initUndistortRectifyMap(fisheye_intrinsic_matrix, fisheye_distortion_coeffs, cv::Matx33d::eye(), fisheye_intrinsic_matrix, result_size, CV_16SC2, mapx, mapy);
    }
    else 
    {
        std::cout << "nonfisheye remap..." << std::endl;
        
        cv::initUndistortRectifyMap(nonfisheye_intrinsic_matrix, nonfisheye_distortion_coeffs, cv::Mat::eye(3, 3, CV_32F), nonfisheye_intrinsic_matrix, result_size, CV_32F, mapx, mapy);
    }
    
    cv::remap(image_input, undistorted_result, mapx, mapy, cv::INTER_LINEAR);//, cv::BORDER_TRANSPARENT);
    
    cv::namedWindow("undistorted_result", CV_WINDOW_AUTOSIZE);
    cv::imshow("undistorted_result", undistorted_result);
    cv::waitKey(0);
    
    // Set 4 Point in undistorted_result Image, from left_top to right_bottom
    std::cout << "Get Points Sequnece: LT(Left-Top) -> RT(Right-Top) -> LB(Left-Bottom) -> RB(Right-Bottom)" << std::endl;
    
    // Catch Mouse Event
    cv::setMouseCallback("undistorted_result", onMouse, reinterpret_cast<void*>(&undistorted_result));
    
    // Calculate Perspective Transform Matrix 
    cv::Point2f tmp_src4points[] = {bird_4points[0], bird_4points[1], bird_4points[2], bird_4points[3]};
    cv::Point2f tmp_dst4points[] = {params.project_keypoints[camera_name][0], params.project_keypoints[camera_name][1], params.project_keypoints[camera_name][2], params.project_keypoints[camera_name][3]};
    project_matrix = cv::getPerspectiveTransform(tmp_src4points, tmp_dst4points);
    
    std::cout << "project_matrix = \n" << project_matrix << std::endl;
}

void CalibrateCamera::onMouse(int event, int x, int y, int flags, void* param)
{
    cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
    static int i = 0;
    switch(event)
    {
        case cv::EVENT_LBUTTONDOWN:
            bird_4points[(i++)%4] = cv::Point2f(x, y);
            std::cout << "(" << x << ", " << y << ")" << std::endl;
            break;
    }
}

void CalibrateCamera::save_calibrated_yaml()
{
    if (isFishEye)
    {
        cv::FileStorage fisheyefs("config/fisheye/"+camera_name+".yaml", cv::FileStorage::WRITE);
        
        // Transfer Format, more easy to access.
        cv::Mat fisheye_intrinsic_matrix_yaml = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                fisheye_intrinsic_matrix_yaml.at<float>(i, j) = fisheye_intrinsic_matrix(i, j);
            }
        }
        
        cv::Mat fisheye_distortion_coeffs_yaml = cv::Mat(4, 1, CV_64FC1, cv::Scalar::all(0));
        for (int i = 0; i < 4; i++)
        {
            fisheye_distortion_coeffs_yaml.at<double>(i, 0) = fisheye_distortion_coeffs(i);
        }
        
        cv::Mat resolution = cv::Mat(2, 1, CV_32SC1);
        resolution.at<int>(0, 0) = int(image_input.cols);
        resolution.at<int>(1, 0) = int(image_input.rows);
        
        fisheyefs << "camera_matrix" << fisheye_intrinsic_matrix_yaml;
        fisheyefs << "dist_coeffs" << fisheye_distortion_coeffs_yaml;
        fisheyefs << "resolution" << resolution;
        fisheyefs << "project_matrix" << project_matrix;
        fisheyefs << "scale_xy" << scale_xy;
        fisheyefs << "shift_xy" << shift_xy;
        
        fisheyefs.release();
    }
    else
    {
        cv::FileStorage nonfisheyefs;
        nonfisheyefs.open("config/nonfisheye/"+camera_name+".yaml", cv::FileStorage::WRITE);
        
        nonfisheyefs << "nonfisheye_intrinsic_matrix" << nonfisheye_intrinsic_matrix;
        nonfisheyefs << "nonfisheye_distortion_coeffs" << nonfisheye_distortion_coeffs;
        
        nonfisheyefs.release();
    }
}
