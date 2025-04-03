#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include <Cone.hpp>

using namespace cv;

typedef enum {RED, BLUE, YELLOW} _Color;

Scalar lower_orange(114, 0, 145);  // Limite inferiore (HSV)
Scalar upper_orange(180, 255, 255); // Limite superiore (HSV)
Scalar lower_blue(40 , 70 , 70);
Scalar upper_blue(125 , 255 , 255);
Scalar lower_yellow(14 , 100 , 184);
Scalar upper_yellow(23 , 255 , 255);


class Detector{
    private:
        Mat img;
        std::vector<Cone> detect_cones(Scalar& lower_bound , Scalar& upper_bound, const float AREA_TRESHOLD , const float WH_RATEO_TRESHOLD);
    public:
        Detector(Mat& img);
        ~Detector();

        std::vector<Cone> get_cones();
        pair<std::vector<Point> , std::vector<Point>> extract_track_edges(std::vector<Cone>& cones);
};