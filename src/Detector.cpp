#include "Detector.hpp"

using namespace std;

std::vector<Cone> get_cones_boxes(Mat& Img  , Scalar lower_bound , Scalar upper_bound , const float WH_TRESHOLD){
    Mat hsv , mask;
    float wh_rateo;
    std::vector<Cone> ret_vec;

    cvtColor(Img , hsv , COLOR_BGR2HSV);
    inRange(hsv , lower_bound, upper_bound, mask);

    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours){
        std::vector<Point> approx;
        approxPolyDP(contour, approx, 10, true);

        double area = cv::contourArea(contour);
        cv::Rect bounding_box = cv::boundingRect(contour);

        wh_rateo = bounding_box.width / bounding_box.height;

        if (area >500 && wh_rateo > WH_TRESHOLD){
            ret_vec.push_back(Cone(bounding_box , lower_bound , upper_bound));
            cv::rectangle(Img, bounding_box, cv::Scalar(0, 255, 0), 2);
        }
    }

    return ret_vec;
}
