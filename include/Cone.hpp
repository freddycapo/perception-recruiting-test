#pragma once
#include <opencv2/opencv.hpp>

typedef enum {RED, BLUE, YELLOW} _Color;

class Cone
{
private:
    cv::Rect box;
    cv::Point2f pos;
    _Color color;

public:
    cv::Rect getBox();
    cv::Point2f getPos2f();
    cv::Point getPos();
    Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar , cv::Point2f pos);
    ~Cone();
};