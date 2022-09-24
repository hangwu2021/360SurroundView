#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[])
{
    cv::Mat img1 = cv::imread("1.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat img2 = cv::imread("2.png", CV_LOAD_IMAGE_COLOR);
    assert(img1.data != nullptr && img2.data != nullptr);
    
    std::vector<cv::KeyPoint> points1, points2;
    cv::Mat descriptors1, descriptors2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img1, points1);
    detector->detect(img2, points2);
    
    descriptor->compute(img1, points1, descriptors1);
    descriptor->compute(img2, points2, descriptors2);
    
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "extract ORB cost: " << time_used.count() << " seconds." << std::endl;
    
    cv::Mat outimg1;
    cv::drawKeypoints(img1, points1, outimg1, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("ORB features", outimg1);
    
    // Step3: Matching
    std::vector<cv::DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = std::chrono::steady_clock::now();
    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "match ORB cost: " << time_used.count() << " seconds." << std::endl;
    
    // Step4: Dist
    auto min_max = std::minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2){ return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    std::cout << "-- Max dist: " << max_dist << std::endl;
    std::cout << "-- Min dist: " << min_dist << std::endl;
    
    std::vector<cv::DMatch> good_matches;
    for (auto &m : matches)
    {
        if (m.distance < std::max(2*min_dist, 25.0))
        {
            good_matches.push_back(m);
        }
    }
    std::cout << "good_matches.size() = " << good_matches.size() << std::endl;
    
    // Step5: Show
    cv::Mat img_match;
    cv::Mat img_good_match;
    cv::drawMatches(img1, points1, img2, points2, matches, img_match);
    cv::drawMatches(img2, points2, img2, points2, good_matches, img_good_match);
    cv::imshow("img_match", img_match);
    cv::imshow("img_good_match", img_good_match);
    
    cv::waitKey(0);
    
    return 0;
}
