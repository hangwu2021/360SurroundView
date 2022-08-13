#include "ParamSettings.h"

ParamSettings::ParamSettings()
{
    camera_names = {"front", "back", "left", "right"};
    
    shift_w = 240;
    shift_h = 240;
    
    inn_shift_w = 40;
    inn_shift_h = 40;
    
    total_w = 600 + 2*shift_w;
    total_h = 600 + 2*shift_h;
    
    // Four Corners of the Rectangle occupied by the Car Model
    xl = shift_w + 120 + inn_shift_w;
    yt = shift_h + 120 + inn_shift_h;
    xr = total_w - xl;
    yb = total_h - yt;
    
    project_shapes.insert(std::map<std::string, cv::Vec2f>::value_type("front", cv::Vec2f(total_w, yt)));
    project_shapes.insert(std::map<std::string, cv::Vec2f>::value_type("back", cv::Vec2f(total_w, yt)));
    project_shapes.insert(std::map<std::string, cv::Vec2f>::value_type("left", cv::Vec2f(total_h, xl)));
    project_shapes.insert(std::map<std::string, cv::Vec2f>::value_type("right", cv::Vec2f(total_h, xl)));
    
    tempPointsVec.clear();
    tempPointsVec = {cv::Point2f(shift_w+120, shift_h), cv::Point2f(shift_w+480, shift_h), cv::Point2f(shift_w+120, shift_h+160), cv::Point2f(shift_w+480, shift_h+160)};
    project_keypoints.insert(std::map<std::string, std::vector<cv::Point2f>>::value_type("front", tempPointsVec));
    
    tempPointsVec.clear();
    tempPointsVec = {cv::Point2f(shift_w+120, shift_h), cv::Point2f(shift_w+480, shift_h), cv::Point2f(shift_w+120, shift_h+160), cv::Point2f(shift_w+480, shift_h+160)};
    project_keypoints.insert(std::map<std::string, std::vector<cv::Point2f>>::value_type("back", tempPointsVec));
    
    tempPointsVec.clear();
    tempPointsVec = {cv::Point2f(shift_h+280, shift_w), cv::Point2f(shift_h+840, shift_w), cv::Point2f(shift_h+280, shift_w+160), cv::Point2f(shift_h+840, shift_w+160)};
    project_keypoints.insert(std::map<std::string, std::vector<cv::Point2f>>::value_type("left", tempPointsVec));
    
    tempPointsVec.clear();
    tempPointsVec = {cv::Point2f(shift_h+160, shift_w), cv::Point2f(shift_h+720, shift_w), cv::Point2f(shift_h+160, shift_w+160), cv::Point2f(shift_h+720, shift_w+160)};
    project_keypoints.insert(std::map<std::string, std::vector<cv::Point2f>>::value_type("right", tempPointsVec));
    
    tempPointsVec.clear();
}
