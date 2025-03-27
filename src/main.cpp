#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>

#include "Cone.hpp"

using namespace std;

int main() {
    cv::Mat image = cv::imread("../src/img/frame_1.png");
    if (image.empty()) {
        cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }

    //cv::Rect Rect_ROI(0 , 200 , image.cols, 200);
    //cv::rectangle(image, Rect_ROI, cv::Scalar(255), 1, 8, 0);

    //cv::Mat ROI = image(Rect_ROI);

    cv::Mat hsv;
    cv::cvtColor(image , hsv , cv::COLOR_BGR2HSV);
    //cv::imwrite("../src/img/hsv_frame.png", hsv);

    cv::Scalar lower_orange(114, 0, 145);  // Limite inferiore (HSV)
    cv::Scalar upper_orange(180, 255, 255); // Limite superiore (HSV)

    cv::Scalar lower_blue(40 , 70 , 70);
    cv::Scalar upper_blue(125 , 255 , 255);

    cv::Scalar lower_yellow(0 , 114 , 197);
    cv::Scalar upper_yellow(60 , 255 , 255);

    cv::Mat mask;
    cv::inRange(hsv, lower_orange, upper_orange, mask);
    //cv::imwrite("../src/img/hsv_mask_frame.png", mask);

    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const float WH_RATEO_TRESHOLD = 0.66; // rapporto width / height ( 2 / 3 )
    float wh_rateo;

    
    
    for (const auto& contour : contours) {
        // Approssima la forma trovata
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 10, true);

        // Filtra i contorni in base all'area e alla forma
        double area = cv::contourArea(contour);
        cv::Rect bounding_box = cv::boundingRect(contour);

        Cone cono(bounding_box , lower_orange , upper_orange);

        wh_rateo = bounding_box.width / bounding_box.height;

        if (area > 500 && wh_rateo < WH_RATEO_TRESHOLD) { // Controlla che sia abbastanza grande per essere un cono
            cv::rectangle(image, bounding_box, cv::Scalar(0, 255, 0), 2);
        }
    }

    cv::imshow("Image" , image);
    cv::waitKey(0);
    return 0;
}