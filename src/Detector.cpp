#include "Detector.hpp"


Detector::Detector(Mat frame)
{
    this->frame = frame;
}

Detector::~Detector()
{
}

std::vector<Cone> Detector::get_cones_boxes(Scalar lower_bound , Scalar upper_bound){
    Mat hsv , mask;
    cvtColor(this->frame , hsv , COLOR_BGR2HSV);
    inRange(hsv , lower_bound, upper_bound, mask);

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    return std::vector<Cone>();
}
