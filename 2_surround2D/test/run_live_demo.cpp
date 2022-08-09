#include "FisheyeCameraModel.h"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: ./run_live_demo carImage.png" << std::endl;
        
        return -1;
    }
    
    ParamSettings param;
    
    cv::Mat carImage;
    carImage = param.getCarImage(argv[1]);
    
    
    FisheyeCameraModel fish("../config/front.yaml", "front");
    
    fish.load_camera_params();
    
    fish.update_undistort_maps();
    
    
    
    
    /*
    cv::namedWindow("Car Model", CV_WINDOW_AUTOSIZE);
    cv::imshow("Car Model", carImage);
    
    cv::waitKey(0);
    cv::destroyAllWindows();
    */
    
    return 0;
}
