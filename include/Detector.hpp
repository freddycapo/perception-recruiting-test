#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include <Cone.hpp>

using namespace cv;

typedef enum {RED, BLUE, YELLOW} _Color;

class Detector{
    private:
        Mat img;
        std::vector<Cone> detect_cones(Scalar& lower_bound , Scalar& upper_bound, const float AREA_TRESHOLD , const float WH_RATEO_TRESHOLD);
        public:
        Detector(Mat& img);
        ~Detector();

        std::vector<Cone> get_cones();
        std::pair<std::vector<Point2f> , std::vector<Point2f>> extract_track_edges(std::vector<Cone>& cones);
};