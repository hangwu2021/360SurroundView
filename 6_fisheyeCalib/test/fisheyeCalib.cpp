#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

/*
 * CheesBoard Corners: 11*8 30mmx30mm
 */

int main()
{
    // Step1: Load Source Fisheye Images
    cv::String dirname = "data/calibration_img/front";
    std::vector<cv::String> files;
    
    cv::glob(dirname, files);
    
    std::cout << "Source Fisheye Images:" << std::endl;
    for (auto & file : files)
    {
        std::cout << file << std::endl;
    }
    
    // Step2: Start Extracting Corners
    std::cout << "\nStart Extracting Corners" << std::endl;
    cv::Size image_size;
    cv::Size board_size = cv::Size(11, 8);
    std::vector<cv::Point2f> image_points_buf; // Every image corners(11x8)
    std::vector<std::vector<cv::Point2f>> image_points_seq; // All corners
    
    bool ret = false;
    int image_count = 0;
    for (cv::String &file : files)
    {
        image_count++;
        std::cout << "image_count = " << image_count << std::endl;
        
        cv::Mat imageInput = cv::imread(file);
        if (1 == image_count)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            std::cout << "image_size.width = " << image_size.width << ", image_size.height = " << image_size.height << std::endl;
        }
        
        ret = cv::findChessboardCorners(imageInput, board_size, image_points_buf);
        if (false == ret)
        {
            std::cout << "Error: Can not find chessboard corners!" << std::endl;
            exit(EXIT_FAILURE);
        }
        else
        {
            cv::Mat view_gray;
            cv::cvtColor(imageInput, view_gray, cv::COLOR_BGR2GRAY);
            cv::find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5));
            image_points_seq.push_back(image_points_buf);
            
            cv::drawChessboardCorners(view_gray, board_size, image_points_buf, false);
            cv::imshow("Camera Calibration", view_gray);
            cv::waitKey(500);
        }
    }
    
    int total = image_points_seq.size();
    std::cout << "total corners = " << total << std::endl;
    std::cout << "Corners Extracted Over!" << std::endl;
    
    // Step3: Start Calibration
    std::cout << "Start Calibration!" << std::endl;
    cv::Size square_size = cv::Size(30, 30);    // chessboard scale
    std::vector<std::vector<cv::Point3f>> object_points;
    
    // Camera Intrinsics
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat distCoeffs = cv::Mat(1, 4, CV_32FC1, cv::Scalar::all(0));
    
    std::vector<cv::Mat> tvecsMat;
    std::vector<cv::Mat> rvecsMat;
    
    int i, j, t;
    for (t=0; t <image_count; ++t)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (i = 0; i < board_size.height; ++i)
        {
            for (j = 0; j < board_size.width; ++j)
            {
                cv::Point3f realPoint;
                realPoint.x = i * square_size.width;
                realPoint.y = j * square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        object_points.push_back(tempPointSet);
    }
    
    int flags = 0;
    flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flags |= cv::fisheye::CALIB_CHECK_COND;
    flags |= cv::fisheye::CALIB_FIX_SKEW;
    cv::fisheye::calibrate(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, flags, cv::TermCriteria(3, 20, 1e-6));
    
    
    std::cout << "K = \n" << cameraMatrix << "\nD = \n" << distCoeffs << std::endl;
    std::cout << "rvecs.size() = " << rvecsMat.size() << ", tvecs.size() = " << tvecsMat.size() << std::endl;
    std::cout << "Calibration Finished!" << std::endl;
    
    // Step4: reprojection error
    double total_err = 0.0;
    double per_err = 0.0;
    std::vector<cv::Point2f> image_points2;
    for (i = 0; i < image_count; ++i)
    {
        std::vector<cv::Point3f> tempPointSet = object_points[i];
        cv::fisheye::projectPoints(tempPointSet, image_points2, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs);
        
        std::vector<cv::Point2f> image_points = image_points_seq[i];
        cv::Mat image_points_uv = cv::Mat(1, image_points.size(), CV_32FC2);
        cv::Mat image_points_uv2 = cv::Mat(1, image_points2.size(), CV_32FC2);
        
        for (j = 0; j < image_points.size(); ++j)
        {
            image_points_uv.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points[j].x, image_points[j].y);
            image_points_uv2.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
        }
        
        per_err = cv::norm(image_points_uv, image_points_uv2, cv::NORM_L2);
        std::cout << "per_err = " << per_err << std::endl;
    }
    
    // Step5: Show Calibration Result
    cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
    cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32FC1);
//     cv::Mat pCameraMatrix;
//     
//     cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, image_size, R, pCameraMatrix, 1.0);
//     std::cout << pCameraMatrix << std::endl;
    
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
    
//     for (cv::String &file : files)
//     {
//         cv::Mat img = cv::imread(file);
//         cv::Mat undistorted_img;
//         
//         cv::remap(img, undistorted_img, mapx, mapy, cv::INTER_LINEAR);
//         
//         cv::imshow("source", img);
//         cv::imshow("undistorted", undistorted_img);
//         cv::waitKey(1000);
//     }
    
    cv::Mat test_img = cv::imread("data/calibration_img/lf_l2_f1/1663243310_1.jpg");
    cv::Mat undistorted_img;
    
    cv::imshow("src", test_img);
    cv::remap(test_img, undistorted_img, mapx, mapy, cv::INTER_LINEAR);
    
    cv::imshow("undistorted", undistorted_img);
    cv::waitKey(0);
    
    cv::destroyAllWindows();
    
    
    return 0;
}




