#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "usage: ./main image_left.png image_right.png" << std::endl;
        return -1;
    }
    
    cv::Mat img_left, img_right;
    img_left = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE), img_right = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
    assert(img_left.data != nullptr && img_right.data != nullptr);
    
    // Match Feature Points
    std::vector<cv::KeyPoint>           keypoints_left, keypoints_right;
    cv::Mat                             descriptors_left, descriptors_right;
    cv::Ptr<cv::FeatureDetector>        detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor>    descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher>      matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch>             matches;
    std::vector<cv::DMatch>             good_matches;
    
    detector->detect(img_left, keypoints_left);
    detector->detect(img_right, keypoints_right);
    
    descriptor->compute(img_left, keypoints_left, descriptors_left);
    descriptor->compute(img_right, keypoints_right, descriptors_right);
    
    matcher->match(descriptors_left, descriptors_right, matches);
    
    double dist_max = 0.0, dist_min = 10000.0;
    for (auto &match : matches)
    {
        if (match.distance > dist_max)
        {
            dist_max = match.distance;
        }
        
        if (match.distance < dist_min)
        {
            dist_min = match.distance;
        }
    }
    
    std::cout << "dist_max = " << dist_max << std::endl;
    std::cout << "dist_min = " << dist_min << std::endl;
    
    for (auto &match : matches)
    {
        if (match.distance < std::max<float>(dist_min * 2, 30))
        {
            good_matches.push_back(match);
        }
    }
    
    std::cout << good_matches.size() << std::endl;
    
    // Step3: Homography Matrix
    std::vector<cv::Point2f> points_left, points_right;
    for (auto &good_match : good_matches)
    {
        points_left.push_back(keypoints_left[good_match.queryIdx].pt);
        points_right.push_back(keypoints_right[good_match.trainIdx].pt);
    }
    
    cv::Mat homography_matrix;
    homography_matrix = cv::findHomography(points_left, points_right, cv::RANSAC);
    
    cv::Mat homography_matrix_inv;
    cv::invert(homography_matrix, homography_matrix_inv, cv::DECOMP_LU);
    
    cv::Mat canvas;
    cv::Mat imgSrc_left = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR), imgSrc_right = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);
    
    cv::warpPerspective(imgSrc_right, canvas, homography_matrix_inv, cv::Size(imgSrc_left.cols*2, img_left.rows));
    imgSrc_left.copyTo(canvas(cv::Range::all(), cv::Range(0, imgSrc_left.cols)));
    
    cv::imshow("canvas", canvas);
    
    // Display Image
    cv::Mat outImg1;
    cv::drawMatches(img_left, keypoints_left, img_right, keypoints_right, good_matches, outImg1);
    cv::imshow("Result Image", outImg1);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}
