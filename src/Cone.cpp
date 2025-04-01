#include "Cone.hpp"

Cone::Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar , cv::Point2f pos)
{
    this->box = box;
    this->pos = pos;
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
