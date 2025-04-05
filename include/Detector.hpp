#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>

#include <Cone.hpp>

using namespace cv;

class Detector{
    private:
        Mat img;
        std::vector<Cone> detect_cones(Color_t color, const float AREA_TRESHOLD , const float WH_RATEO_TRESHOLD);
        const Scalar color_masks[3][2] = {{ Scalar(114, 0, 145) , Scalar(180, 255, 255)},
                                    { Scalar(40 , 70 , 70) , Scalar(125 , 255 , 255)},
                                    { Scalar(14 , 100 , 184) , Scalar(23 , 255 , 255)}}; 
        public:
        Detector(Mat& img);
        ~Detector();

        std::vector<Cone> get_cones();
        std::pair<std::vector<Point2f> , std::vector<Point2f>> extract_track_edges(std::vector<Cone>& cones);
        std::vector<KeyPoint> find_keypoints();
};