#include <iostream>

#include "opencv2/opencv.hpp"

// frame_front -> frame_left -> frame_rear -> frame_right -> frame_front

void CamParamInit(const std::string& cam, cv::Mat& intrisics, cv::Mat& distCoeffs, cv::Mat& resolution);

void UndistortedPerImage();

int main(int argc, char* argv[])
{
    std::string video_path = "data/ArUco/";

    cv::Mat frame_front, frame_left, frame_rear, frame_right;
    cv::VideoCapture cap_front, cap_left, cap_rear, cap_right;
    
    cap_front.open(video_path+"5/front.avi");
    cap_left.open(video_path+"5/left.avi");
    if (!cap_front.isOpened())
    {
        std::cerr << "ERROR! Unable to open Front VideoCapture" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // Front Cmaera
    cv::Mat intrisics_front, distCoeffs_front;
    cv::Mat resolution_front;
    cv::Mat mapx_front, mapy_front;
    cv::Mat undistorted_frame_front;
    CamParamInit("front", intrisics_front, distCoeffs_front, resolution_front);
    cv::fisheye::initUndistortRectifyMap(intrisics_front, distCoeffs_front, cv::Mat::eye(3, 3, CV_32F), intrisics_front, cv::Size(resolution_front.at<int>(0, 0), resolution_front.at<int>(1, 0)), CV_32FC1, mapx_front, mapy_front);
    frame_front = cv::imread("/home/hangwu/Documents/test_slambook2/ch7/front.jpg", CV_LOAD_IMAGE_COLOR);
    cv::remap(frame_front, undistorted_frame_front, mapx_front, mapy_front, CV_INTER_LINEAR);
    cv::imwrite("undistorted_frame_front.png", undistorted_frame_front);
    
    // Left Camera 
    cv::Mat intrisics_left, distCoeffs_left, resolution_left;
    cv::Mat mapx_left, mapy_left;
    cv::Mat undistorted_frame_left;
    CamParamInit("left", intrisics_left, distCoeffs_left, resolution_left);
    cv::fisheye::initUndistortRectifyMap(intrisics_left, distCoeffs_left, cv::Mat::eye(3, 3, CV_32F), intrisics_left, cv::Size(resolution_left.at<int>(0, 0), resolution_left.at<int>(1, 0)), CV_32FC1, mapx_left, mapy_left);
   
    // Back Camera
    
    // Right Camera
    cv::Mat intrisics_right, distCoeffs_right, resolution_right;
    cv::Mat mapx_right, mapy_right;
    cv::Mat undistorted_frame_right;
    CamParamInit("right", intrisics_right, distCoeffs_right, resolution_right);
    cv::fisheye::initUndistortRectifyMap(intrisics_right, distCoeffs_right, cv::Mat::eye(3, 3, CV_32F), intrisics_right, cv::Size(resolution_right.at<int>(0, 0), resolution_right.at<int>(1, 0)), CV_32FC1, mapx_right, mapy_right);
    // test function
    //cv::Mat frame_right;
    frame_right = cv::imread("/home/hangwu/Documents/test_slambook2/ch7/right.jpg", CV_LOAD_IMAGE_COLOR);
    cv::remap(frame_right, undistorted_frame_right, mapx_right, mapy_right, CV_INTER_LINEAR);
    cv::imwrite("undistorted_frame_right.png", undistorted_frame_right);
    
    
//     for (; ;)
//     {
//         cap_front.read(frame_front);
//         if (frame_front.empty())
//         {
//             std::cerr << "WARNING! Blank frame_front grabbed!" << std::endl;
//             break;
//         }
//         
//         cap_left.read(frame_left);
//         if (frame_left.empty())
//         {
//             std::cerr << "WARNING! Blank frame_left grabbed!" << std::endl;
//             break;
//         }
//         
//         // Undistort Frame
//         cv::remap(frame_front, undistorted_frame_front, mapx, mapy, CV_INTER_LINEAR);
//         cv::remap(frame_left, undistorted_frame_left, mapx_left, mapy_left, CV_INTER_LINEAR);
//         
// //         // Crop Input Frame 
// //         cv::Mat frame_front_1, frame_left_2;
// //         frame_front.copyTo(frame_front_1);
// //         frame_left.copyTo(frame_left_2);
//         
// //         cv::imshow("div_front", frame_front_1);
// //         cv::imshow("div_left", frame_left_2);
// //         
// //         cv::waitKey(1000);
// //         continue;
//         
//         // Step2: Feature Matching
//         // <1> Initialize
//         std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
//         cv::Mat descriptors_1, descriptors_2;
//         cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
//         cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
//         cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//         
//         // <2> Detect Corners
//         detector->detect(undistorted_frame_front, keypoints_1);
//         detector->detect(undistorted_frame_left, keypoints_2);
//         
//         // <3> Compute Descriptors 
//         descriptor->compute(undistorted_frame_front, keypoints_1, descriptors_1);
//         descriptor->compute(undistorted_frame_left, keypoints_2, descriptors_2);
//         
// //         cv::Mat outImg1;
// //         cv::drawKeypoints(frame_front, keypoints_1, outImg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
// //         cv::imshow("outImg1", outImg1);
//         
//         // <4> Featrue Match 
//         std::vector<cv::DMatch> matches;
//         matcher->match(descriptors_1, descriptors_2, matches);
//         
//         // <5> Compute Dist
//         auto min_max = std::minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2){ return m1.distance < m2.distance; });
//         double min_dist = min_max.first->distance;
//         double max_dist = min_max.second->distance;
//         
//         std::cout << "min_dist = " << min_dist << std::endl;
//         std::cout << "max_dist = " << max_dist << std::endl;
//         
//         // <6> Good Match 
//         std::vector<cv::DMatch> good_matches;
//         for (int i = 0; i < descriptors_1.rows; ++i)
//         {
//             if (matches[i].distance <= std::max(2*min_dist, 30.0))
//             {
//                 good_matches.push_back(matches[i]);
//             }
//         }
//         
//         cv::Mat img_goodmatch;
//         cv::drawMatches(undistorted_frame_front, keypoints_1, undistorted_frame_left, keypoints_2, good_matches, img_goodmatch);
//         cv::imshow("goodMatch", img_goodmatch);
//         
//         
// //         cv::imshow("front", frame_front);
// //         cv::imshow("left", frame_left);
//         if (cv::waitKey(50) >= 0)
//         {
//             break;
//         }
//     }
    
    return 0;
}

void CamParamInit(const std::string& cam, cv::Mat& intrisics, cv::Mat& distCoeffs, cv::Mat& resolution)
{
    std::string params_file = "intrisics/"+cam+".yaml";
    cv::FileStorage params;
    params.open(params_file, cv::FileStorage::READ);
    if (!params.isOpened())
    {
        std::cerr << "Error: Failed to open file." << std::endl;
        return ;
    }
    
    params["camera_matrix"] >> intrisics;
    params["dist_coeffs"] >> distCoeffs;
    params["resolution"] >> resolution;
    
    params.release();
}







