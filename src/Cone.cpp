#include "Cone.hpp"

Cone::Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar , cv::Point pos)
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

cv::Point Cone::getPos(){
    return this->pos;
}
