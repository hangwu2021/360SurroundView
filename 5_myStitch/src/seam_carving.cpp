#include "myStitch/seam_carving.hpp"

namespace myStitch
{

SeamCarving::SeamCarving()
{
    seam_finder = new cv::detail::GraphCutSeamFinder(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
}

void SeamCarving::setSources(const cv::Mat& img1, const cv::Mat& img2)
{
    cv::UMat uimg1, uimg2;
    
    img1.convertTo(uimg1, CV_32FC3);
    img1 /= 255.0;
    sources.push_back(uimg1);
    
    img2.convertTo(uimg2, CV_32FC3);
    img2 /= 255.0;
    sources.push_back(uimg2);
}

void SeamCarving::setCorners(const cv::Point& corner1, const cv::Point& corner2)
{
    corners.push_back(corner1);
    corners.push_back(corner2);
}

std::vector<cv::UMat> SeamCarving::getMasks()
{
    if (sources.empty())
    {
        std::cout << "Error: sources image is empty!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    for (size_t i = 0; i < sources.size(); ++i)
    {
        cv::UMat temp = cv::UMat(sources[i].size(), CV_8U);
        temp = cv::Scalar::all(255);
        
        masks.push_back(temp);
    }
    
    return masks;
}


cv::Mat SeamCarving::getCanvasSeamCropped()
{
    (void)getMasks();
    
    seam_finder->find(sources, corners, masks);
    
    cv::Mat canvas(sources[1].rows, sources[1].cols*3/2, CV_32FC3);
    sources[0].copyTo(canvas(cv::Range::all(), cv::Range(0, canvas.cols*2/3)), masks[0]);
    sources[1].copyTo(canvas(cv::Range::all(), cv::Range(canvas.cols/3, canvas.cols)), masks[1]);

    canvas /= 255.0;
    
    return canvas;
}

// Way2: Optimize Seam (trans == img2)
void SeamCarving::OptimizeSeam(cv::Mat& img1, cv::Mat& trans, cv::Mat& dst, const int start)
{
    if (start > img1.cols)
    {
        std::cerr << "Error: Overlap Region is too Small!" << std::endl;
        exit(EXIT_FAILURE);
    }
    
    double processWidth = img1.cols - start;
    int rows = dst.rows;
    int cols = img1.cols;
    double alpha = 1.0;
    
    for (int i = 0; i < rows; ++i)
    {
        uchar* p = img1.ptr<uchar>(i);
        uchar* t = trans.ptr<uchar>(i);
        uchar* d = dst.ptr<uchar>(i);
        for (int j = start; j < cols; ++j)
        {
            if (t[j*3] == 0 && t[j*3+1] == 0 && t[j*3+2]==0)
            {
                alpha = 1.0f;
            }
            else
            {
                alpha = (processWidth - (j - start)) / processWidth;
            }
            
            d[j*3] = p[j*3] * alpha + t[j*3] * (1.0 - alpha);
            d[j*3+1] = p[j*3+1] * alpha + t[j*3+1] * (1.0 - alpha);
            d[j*3+2] = p[j*3+2] * alpha + t[j*3+2] * (1.0 - alpha);
            //std::cout << alpha;
        }
    }
}

} // namespace myStitch
