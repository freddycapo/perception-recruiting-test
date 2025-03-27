#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

#include "Cone.hpp"

using namespace cv;

class Detector{
    private:
        Mat frame;
    public:
        std::vector<Cone> get_cones_boxes(Scalar lower_bound , Scalar upper_bound);
        Detector(Mat frame);
        ~Detector();
};
    