#include "Cone.hpp"

Cone::Cone(cv::Rect box , cv::Point2f pos , Color_t color)
{
    this->box = box;
    this->pos = pos;
    this->color = color;
}

Cone::~Cone()
{
}

cv::Rect Cone::getBox(){
    return this->box;
}

cv::Point2f Cone::getPos2f(){
    return this->pos;
}

cv::Point Cone::getPos(){
    return (cv::Point) this->pos;
}
