#pragma once
#include <opencv2/opencv.hpp>

class Cone
{
private:
    cv::Rect box;
    cv::Point2f pos;
public:
    cv::Rect getBox();
    cv::Point2f getPos2f();
    cv::Point getPos();
    Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar , cv::Point2f pos);
    ~Cone();
};