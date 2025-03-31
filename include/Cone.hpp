#pragma once
#include <opencv2/opencv.hpp>

typedef enum {RED, BLUE, YELLOW} _Color;

class Cone
{
private:
    cv::Rect box;
    cv::Point pos;
    _Color color;

public:
    cv::Rect getBox();
    cv::Point getPos();
    Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar , cv::Point pos);
    ~Cone();
};