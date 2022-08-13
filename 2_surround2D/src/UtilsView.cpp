#include "UtilsView.h"

double UtilsView::tune(double x)
{
    if (x > 1)
    {
        return x * std::exp((1 - x) * 0.5);
    }
    else 
    {
        return x * std::exp((1 - x) * 0.8);
    }
}

cv::Mat UtilsView::get_mask(const cv::Mat &image)
{
    cv::Mat mask;
    
    cv::cvtColor(image, mask, cv::COLOR_BGR2GRAY);
    
    cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);
    
    return mask;
}

cv::Mat UtilsView::get_overlap_region_mask ( const cv::Mat& imA, const cv::Mat& imB )
{
    cv::Mat overlap, mask;
    
    cv::bitwise_and(imA, imB, overlap);
    
    mask = get_mask(overlap);
    
    cv::dilate(mask, mask, cv::Mat::ones(2, 2, CV_8UC1));
    
    return mask;
}

std::vector<cv::Point> UtilsView::get_outmost_polygon_boundary ( const cv::Mat& img )
{
    cv::Mat mask;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour_poly;
    
    mask = get_mask(img);
    cv::dilate(mask, mask, cv::Mat::ones(2, 2, CV_8UC1));
    cv::findContours(mask, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    // Find Largest Area Contour 
    std::sort(contours.begin(), contours.end(), ConstPointComp());
    
    cv::approxPolyDP(contours[0], contour_poly, 0.009*cv::arcLength(contours[0], true), true);
    
    return contour_poly;
}

// Get the weight matrix G that combines two images imA, imB smoothly.
cv::Mat UtilsView::get_weight_mask_matrix ( const cv::Mat& imA, const cv::Mat& imB, cv::Mat& overlap_mask, int dist_threshold )
{
    overlap_mask = get_overlap_region_mask(imA, imB);
    
    cv::Mat overlap_mask_inv;
    cv::bitwise_not(overlap_mask, overlap_mask_inv);
    
    cv::Mat imA_diff, imB_diff;
    cv::bitwise_and(imA, imA, imA_diff, overlap_mask_inv);
    cv::bitwise_and(imB, imB, imB_diff, overlap_mask_inv);
    
    cv::Mat G, G_f;
    get_mask(imA).convertTo(G_f, CV_64F);   // Must 64bit, nor cannot respective
    G = G_f / 255.0;
    
    std::vector<cv::Point> polyA, polyB;
    polyA = get_outmost_polygon_boundary(imA_diff);
    polyB = get_outmost_polygon_boundary(imB_diff);
    
    std::vector<cv::Point> idx;
    cv::findNonZero(overlap_mask, idx);
    for (auto &id : idx)
    {
        double distToB = cv::pointPolygonTest(polyB, cv::Point(id.x, id.y), true);
        if (distToB < dist_threshold)
        {
            double distToA = cv::pointPolygonTest(polyA, cv::Point(id.x, id.y), true);
            distToB *= distToB;
            distToA *= distToA;
            G.at<double>(id.y, id.x) = distToB / (distToA + distToB);
        }
    }
    
    return G;
}

cv::Mat UtilsView::merge ( const cv::Mat& imA, const cv::Mat& imB, const cv::Mat& G )
{
    // Step1: split channels
    std::vector<cv::Mat> channelsA, channelsB;
    
    cv::split(imA, channelsA);
    cv::split(imB, channelsB);
    
    // Step2: add weight G matrix
    std::vector<cv::Mat> fusion_channels;
    
    for (int i = 0; i < 3; i++)
    {
        channelsA[i].convertTo(channelsA[i], G.type());
        //channelsA[i] /= 255.0;        //多多实验，熟能生巧！！！
        
        channelsB[i].convertTo(channelsB[i], G.type());
        //channelsB[i] /= 255.0;        //多多实验，熟能生巧！！！
        
        fusion_channels.push_back((channelsA[i].mul(G) + channelsB[i].mul(cv::Mat::ones(G.rows, G.cols, CV_64FC1) - G)));
    }
    
    // Step3: merge 3 channels image
    cv::Mat merge_result;
    cv::merge(fusion_channels, merge_result);
    
    return merge_result;
}

double UtilsView::get_mean_statistisc ( const cv::Mat& gray, const cv::Mat& mask_bool )
{
    // Ensure the matrix mask value either 0 or 1.
    
    return gray.dot(mask_bool);
}

cv::Mat UtilsView::convert_binary_to_bool(const cv::Mat& mask)
{
    cv::Mat mask_bool;
    
    mask.convertTo(mask_bool, CV_64FC1);
    mask_bool /= 255.0;
    mask_bool.convertTo(mask_bool, CV_8UC1);
    
    return mask_bool;
}

double UtilsView::mean_luminance_ratio(const cv::Mat& grayA, const cv::Mat& grayB, const cv::Mat& mask_bool)
{
    return get_mean_statistisc(grayA, mask_bool) / get_mean_statistisc(grayB, mask_bool);
}

cv::Mat UtilsView::adjust_luminance(const cv::Mat& gray, const double factor)
{
    cv::Mat gray_factor = gray.clone();
    
    for (int i = 0; i < gray_factor.rows; i++)
    {
        for (int j = 0; j < gray_factor.cols; j++)
        {
            gray_factor.at<uchar>(i, j) = std::min(gray.at<uchar>(i, j) * factor, 255.0);
        }
    }
    
    return gray_factor;
}

void UtilsView::make_luminace_balance(cv::Mat& image)
{
    // step1: BGR -> YUV
    cv::Mat yuv_img;
    cv::cvtColor(image, yuv_img, CV_BGR2YUV);
    
    // step2: split
    std::vector<cv::Mat> yuv_channels;
    cv::split(yuv_img, yuv_channels);
    
    // step3: equalize
    cv::equalizeHist(yuv_channels[0], yuv_channels[0]);
    
    // step4: merge
    cv::merge(yuv_channels, image);
    
    // step5: YUV -> BGR 
    cv::cvtColor(image, image, CV_YUV2BGR);
}

cv::Mat UtilsView::make_white_blance(const cv::Mat& image)
{
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    double m1, m2, m3, k;
    cv::Scalar s = cv::mean(image);
    m1 = s[0], m2 = s[1], m3 = s[2];
    k = (m1 + m2 + m3) / 3;
    
    std::vector<double> factors;
    factors.push_back(k / m1);
    factors.push_back(k / m2);
    factors.push_back(k / m3);
    
    std::vector<cv::Mat> balanced_channels;
    for (int i = 0; i < 3; i++)
    {
        balanced_channels.push_back(adjust_luminance(channels[i], factors[i]));
    }
    
    cv::Mat balanced_image;
    cv::merge(balanced_channels, balanced_image);
    
    return balanced_image;
}

