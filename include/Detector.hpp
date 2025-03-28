#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "Cone.hpp"

using namespace cv;

std::vector<Cone> get_cones_boxes(Mat& Img , Scalar lower_bound, Scalar upper_bound, const float WH_TRESHOLD);

    