#include "myStitch/homography.hpp"
#include "myStitch/seam_carving.hpp"

int main()
{
    std::string filepath1 = "data/images/0.jpg";
    std::string filepath2 = "data/images/1.jpg";
    
    cv::Mat img1 = cv::imread(filepath1, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(filepath2, cv::IMREAD_GRAYSCALE);
    
    myStitch::Homography homo12(img1, img2);
    
    cv::Mat h12 = homo12.getHomography();
    std::cout << h12 << std::endl;
    
    cv::Mat h21;
    cv::invert(h12, h21, cv::DECOMP_LU);
    
    cv::Mat canvas;
    cv::Mat img1_color = cv::imread(filepath1, cv::IMREAD_COLOR);
    cv::Mat img2_color = cv::imread(filepath2, cv::IMREAD_COLOR);
    img1_color = img1_color * 0.9;
    
    cv::warpPerspective(img2_color, canvas, h21, cv::Size(img1.cols*2, img1.rows));
//     cv::imshow("canvas_00", canvas);
    cv::Mat trans_right = canvas.clone(); // way2!!!
    
    cv::Mat temp_right;
    temp_right = canvas(cv::Range::all(), cv::Range(canvas.cols/4, canvas.cols*3/4)).clone();
    
    img1_color.copyTo(canvas(cv::Range::all(), cv::Range(0, img1.cols)));
    
    cv::imshow("canvas_seam", canvas);
    cv::imwrite("data/output/canvas_seam.png", canvas);
    
    // Step2: Seam Carving
    myStitch::SeamCarving::Ptr seam_cropper(new myStitch::SeamCarving);
    
    cv::Mat img1_seam = img1_color.clone();
    cv::Mat img2_seam = temp_right.clone();
    seam_cropper->setSources(img1_seam, img2_seam);
    
    cv::Point corner1, corner2;
    corner1.x = 0;
    corner1.y = 0;
    corner2.x = img2_seam.cols/2;
    corner2.y = 0;
    seam_cropper->setCorners(corner1, corner2);
    
    cv::Mat canvas_seamcropped;
    canvas_seamcropped = seam_cropper->getCanvasSeamCropped();
    cv::imshow("canvas_cropped", canvas_seamcropped);
    cv::imwrite("data/output/canvas_seam_cropped.png", canvas_seamcropped*255.0);
    
//     std::vector<cv::UMat> masks;
//     masks = seam_cropper->getMasks();
//     std::string filepath = "data/output/mask";
//     for (size_t i = 0; i < masks.size(); ++i)
//     {
//         cv::imwrite(filepath + std::to_string(i) + ".png", masks[i]);
//     }
    
    // Step2: Seam Crop --- OptimizeSeam
    //cv::Mat canvas_optSeam(cv::Size(img1_color.cols+temp_right.cols/2, img1_color.rows), CV_8UC3);
    cv::Mat canvas_optSeam(trans_right.size(), CV_8UC3);
    canvas_optSeam.setTo(0);
    std::cout << canvas_optSeam.size() << std::endl;
    
    cv::Mat img1_left;
    img1_left = img1_color.clone();
    
    cv::imshow("img1", img1_color);
    cv::imshow("trans", trans_right);
    
    trans_right.copyTo(canvas_optSeam(cv::Range::all(), cv::Range::all()));
    img1_left.copyTo(canvas_optSeam(cv::Range::all(), cv::Range(0, img1.cols)));
    
    seam_cropper->OptimizeSeam(img1_left, trans_right, canvas_optSeam, img1_left.cols*0.9);
    
    cv::imshow("canvas_optSeam", canvas_optSeam);
    cv::imwrite("data/output/canvas_optSeam.png", canvas_optSeam);
    
    // Step3: Luminance Balance
    
    
    
    
    cv::waitKey(0);
    
    return 0;
}
