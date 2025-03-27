#include "Cone.hpp"

Cone::Cone(cv::Rect box, cv::Scalar lower_color_bound, cv::Scalar upper_color_scalar)
{
    this->box = box;
}

Cone::~Cone()
{
}
