#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>


typedef struct
{
    cv::Point2f left_top;
    cv::Point2f left_bottom;
    cv::Point2f right_top;
    cv::Point2f right_bottom;
}four_corners_t;

four_corners_t corners;

void CalcCorners(const cv::Mat& H, const cv::Mat& src)
{
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    cv::Mat V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    cv::Mat V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    std::cout << "V2: " << V2 << std::endl;
    std::cout << "V1: " << V1 << std::endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = cv::Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = cv::Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];

}

int main()
{
    cv::Mat image01 = cv::imread("data/left.jpg", 1);
    cv::Mat image02 = cv::imread("data/right.jpg", 1);
    
    // Gray Convert
    cv::Mat image1, image2;
    cv::cvtColor(image01, image1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(image02, image2, cv::COLOR_BGR2GRAY);
    
    // Feature Match
    cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SURF::create(800);
    std::vector<cv::KeyPoint> keyPoint1, keyPoint2;
    detector->detect(image1, keyPoint1);
    detector->detect(image2, keyPoint2);
    
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::xfeatures2d::SURF::create();
    cv::Mat desc1, desc2;
    descriptor->compute(image1, keyPoint1, desc1);
    descriptor->compute(image2, keyPoint2, desc2);
    
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matchePoints;
    std::vector<cv::DMatch> good_matchePoints;
    
    std::vector<cv::Mat> train_desc(1, desc1);
    matcher.add(train_desc);
    matcher.train();
    matcher.knnMatch(desc2, matchePoints, 2);
   
    std::cout << "total matched points: " << matchePoints.size() << std::endl;
    
    // Lowe's algorithm
    for (auto& m : matchePoints)
    {
        if (m[0].distance < 0.6 * m[1].distance)
        {
            good_matchePoints.push_back(m[0]);
        }
    }
    
    std::cout << "good_matchePoints.size() = " << good_matchePoints.size() << std::endl;
    
    cv::Mat first_match;
    cv::drawMatches(image02, keyPoint2, image01, keyPoint1, good_matchePoints, first_match);
    cv::imshow("matched", first_match);
    
    std::vector<cv::Point2f> imagePoints1, imagePoints2;
    for (int i = 0; i < good_matchePoints.size(); i++)
    {
        imagePoints2.push_back(keyPoint2[good_matchePoints[i].queryIdx].pt);
        imagePoints1.push_back(keyPoint1[good_matchePoints[i].trainIdx].pt);
    }
    
    // Get Homography Matrix
    cv::Mat homo = cv::findHomography(imagePoints1, imagePoints2, cv::RANSAC);
    std::cout << "homo.matrix() = " << homo << std::endl;
    
    // Compute Vertex
    CalcCorners(homo, image01);
    std::cout << "left_top:" << corners.left_top << std::endl;
    std::cout << "left_bottom:" << corners.left_bottom << std::endl;
    std::cout << "right_top:" << corners.right_top << std::endl;
    std::cout << "right_bottom:" << corners.right_bottom << std::endl;
    
    cv::Mat imageTransform1, imageTransform2;
    cv::warpPerspective(image01, imageTransform1, homo, cv::Size(corners.right_top.x, corners.right_bottom.x), image02.rows);
    cv::imshow("trans1", imageTransform1);
    
    
    cv::waitKey(0);
    
    return 0;
}
