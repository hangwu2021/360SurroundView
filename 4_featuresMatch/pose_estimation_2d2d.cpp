#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

void find_feature_matches_by_ORB(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1, 
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);

void find_feature_matches_by_chessboard(
    const cv::Mat &img_1, const cv::Mat &img_2,
    const cv::Mat &K_1, const cv::Mat &K_2,
    const cv::Mat &D_1, const cv::Mat &D_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
);

cv::Mat pose_estimation_2d2d(
    std::vector<cv::KeyPoint> keypoints_1,
    std::vector<cv::KeyPoint> keypoints_2,
    std::vector<cv::DMatch> matches,
    cv::Mat &R, cv::Mat &t
);

cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

void CamParamInit(const std::string& cam, cv::Mat& intrisics, cv::Mat& distCoeffs, cv::Mat& resolution);

/*
 *  MAIN FUNCTION
 */
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: ./bin 1.png 2.png" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // Step1: Load Source Images, Front and Right Image
    cv::Mat img_1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    assert(img_1.data != nullptr && img_2.data != nullptr);
    
    // Step2: Load Camera Intrisics, Front and Right 
    cv::Mat K_1, D_1, K_2, D_2, reso_1, reso_2;
    CamParamInit("front", K_1, D_1, reso_1);
    CamParamInit("right", K_2, D_2, reso_2);
    
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    std::vector<cv::DMatch> matches;
    find_feature_matches_by_chessboard(img_1, img_2, K_1, K_2, D_1, D_2, keypoints_1, keypoints_2, matches);
    std::cout << "matches.size() = " << matches.size() << std::endl;
    
    // Step3: Solve Homography Matrix 
    cv::Mat homo_matrix;
    cv::Mat R, t;
    homo_matrix = pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
    
    std::cout << "R = \n" << R << "\nt = \n" << t << std::endl;
    
    std::cout << "homo_matrix = \n" << homo_matrix << std::endl;
    
    cv::Mat h21;
    cv::invert(homo_matrix, h21, cv::DECOMP_LU);
    
    std::cout << "h21 = " << h21 << std::endl;
    cv::Mat canvas;
    cv::warpPerspective(img_2, canvas, h21, cv::Size(img_1.cols*2, img_1.rows));
    img_1.copyTo(canvas(cv::Range::all(), cv::Range(0, img_1.cols)));
    
    cv::imshow("canvas", canvas);
    cv::imwrite("canvas.png", canvas);
    cv::waitKey(0);
    
    return 0;
}

// Define Functions
void find_feature_matches_by_ORB(
    const cv::Mat& img_1, const cv::Mat& img_2, 
    std::vector<cv::KeyPoint>& keypoints_1, 
    std::vector<cv::KeyPoint>& keypoints_2, 
    std::vector<cv::DMatch>& matches
)
{
    
}

// Fisheye Image 
void find_feature_matches_by_chessboard(
    const cv::Mat &img_1, const cv::Mat &img_2,
    const cv::Mat &K_1, const cv::Mat &K_2,
    const cv::Mat &D_1, const cv::Mat &D_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches
)
{
    // board size of chessboard 
    const cv::Size board_size = cv::Size(11, 8);
    
    // Step1: Extract Corners for Image1
    std::vector<cv::Point2f> corners_1;
    cv::findChessboardCorners(img_1, board_size, corners_1);
    
    cv::Mat img_1_gray;
    cv::cvtColor(img_1, img_1_gray, cv::COLOR_BGR2GRAY);
    cv::find4QuadCornerSubpix(img_1_gray, corners_1, cv::Size(5, 5));
    
    cv::drawChessboardCorners(img_1, board_size, corners_1, false);
//     cv::imshow("corners_1", img_1);
    
    // Step2: Extract Corners for Image2
    std::vector<cv::Point2f> corners_2;
    cv::findChessboardCorners(img_2, board_size, corners_2);
    
    cv::Mat img_2_gray;
    cv::cvtColor(img_2, img_2_gray, cv::COLOR_BGR2GRAY);
    cv::find4QuadCornerSubpix(img_2_gray, corners_2, cv::Size(5, 5));
    
    cv::drawChessboardCorners(img_2, board_size, corners_2, false);
//     cv::imshow("corners_2", img_2);
    
    // Step3: Corners Matching 
    for (int i = 0; i < corners_1.size(); ++i)
    {
        cv::DMatch m(i, i, 256);
        matches.push_back(m);
        
        cv::KeyPoint kp_1;
        kp_1.pt.x = corners_1[i].x;
        kp_1.pt.y = corners_1[i].y;
        keypoints_1.push_back(kp_1);
        
        cv::KeyPoint kp_2;
        kp_2.pt.x = corners_2[i].x;
        kp_2.pt.y = corners_2[i].y;
        keypoints_2.push_back(kp_2);
    }
    
    // Step4: Show Result 
    cv::Mat match_result;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, match_result);
    
    cv::imshow("match_result", match_result);
    cv::imwrite("match_result.png", match_result);
}



cv::Mat pose_estimation_2d2d(
    std::vector<cv::KeyPoint> keypoints_1, 
    std::vector<cv::KeyPoint> keypoints_2, 
    std::vector<cv::DMatch> matches, 
    cv::Mat& R, cv::Mat& t
)
{
    // cv::KeyPoint -> cv::Point2f
    std::vector<cv::Point2f> corners_1, corners_2;
    for (int i = 0; i < keypoints_1.size(); ++i)
    {
        corners_1.push_back(cv::Point2f(keypoints_1[i].pt.x, keypoints_1[i].pt.y));
        corners_2.push_back(cv::Point2f(keypoints_2[i].pt.x, keypoints_2[i].pt.y));
    }
    
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(corners_1, corners_2, cv::RANSAC, 3);
    
//     std::cout << "homography_matrix = \n" << homography_matrix << std::endl;
    
    return homography_matrix;
}

cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
{
    return cv::Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
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
