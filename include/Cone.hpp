#pragma once
#include <opencv2/opencv.hpp>

typedef enum {RED, BLUE, YELLOW} _Color;

class Cone
{
private:
    cv::Rect box;
    _Color color;

public:
    Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar);
    ~Cone();
};