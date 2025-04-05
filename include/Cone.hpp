#pragma once
#include <opencv2/opencv.hpp>

typedef enum {ORANGE , BLUE , YELLOW} Color_t;

class Cone
{
private:
    cv::Rect box;
    cv::Point2f pos;
    Color_t color;
public:
    cv::Rect getBox();
    cv::Point2f getPos2f();
    cv::Point getPos();
    Cone(cv::Rect box, cv::Point2f pos, Color_t color);
    ~Cone();
};