#include "Detector.hpp"

using namespace std;

Detector::Detector(Mat& img){
    this->img = img;
}

Detector::~Detector(){
    //std::cout << "Detector distrutto" << std::endl;
}

std::vector<Cone> Detector::detect_cones(Scalar& lower_bound , Scalar& upper_bound , const float AREA_TRESHOLD , const float WH_RATEO_TRESHOLD)
{
    Mat hsv , mask;
    float wh_rateo;
    std::vector<Cone> cones;

    cvtColor(this->img , hsv , COLOR_BGR2HSV);
    inRange(hsv , lower_bound, upper_bound, mask);

    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours){
        std::vector<Point> approx;
        approxPolyDP(contour, approx, 10, true);
        
        double area = cv::contourArea(contour);
        cv::Rect bounding_box = cv::boundingRect(contour);
        
        wh_rateo = bounding_box.width / bounding_box.height;

        if (area > AREA_TRESHOLD && wh_rateo < WH_RATEO_TRESHOLD){
            cones.push_back(Cone(bounding_box , lower_bound , upper_bound, Point2f(bounding_box.x , bounding_box.y)));
        }
    }

    return cones;
}

std::vector<Cone> Detector::get_cones(){
    cv::Scalar lower_orange(114, 0, 145);   // Limite inferiore (HSV)
    cv::Scalar upper_orange(180, 255, 255); // Limite superiore (HSV)
    cv::Scalar lower_blue(40 , 70 , 70);   
    cv::Scalar upper_blue(125 , 255 , 255);
    cv::Scalar lower_yellow(14 , 100 , 184);
    cv::Scalar upper_yellow(23 , 255 , 255);
    
    std::vector<Cone> orange_cones = this->detect_cones(lower_orange , upper_orange, 500 , 0.66);
    std::vector<Cone> blue_cones   = this->detect_cones(lower_blue , upper_blue, 30 , 2);
    std::vector<Cone> yellow_cones = this->detect_cones(lower_yellow , upper_yellow, 30 , 2);
    std::vector<Cone> ret;
    std::vector<std::vector<Cone>> tmp = {orange_cones  , blue_cones , yellow_cones};

    for(std::vector<Cone> row : tmp){
        for (Cone c: row){
            ret.push_back(c);
        }
    }

    return ret;

}

std::pair<std::vector<Point2f> , std::vector<Point2f>> Detector::extract_track_edges(std::vector<Cone>& cones){
    std::vector<Point2f> Rbound , Lbound;
    for (Cone c : cones){
        if (c.getPos2f().x  > this->img.size().width / 2){
            Rbound.push_back(c.getPos2f());
        } else{
            Lbound.push_back(c.getPos2f());
        }
    }

    approxPolyDP(Lbound, Lbound, 10, true);
    approxPolyDP(Rbound, Rbound, 10, true);
    //cout << "R:: " << Rbound << endl;
    //cout << "L:: " << Lbound << endl;
    return std::pair<std::vector<Point2f> , std::vector<Point2f>>(Lbound, Rbound);

}

std::vector<KeyPoint> Detector::find_keypoints(){
    std::vector<KeyPoint> punti;
    Ptr<FeatureDetector> feature_extracter = ORB::create();
    feature_extracter->detect ( this->img , punti );
    punti.erase(std::remove_if(punti.begin() , punti.end() , [](KeyPoint p){return p.pt.y < 200 || p.pt.y > 330; }) , punti.end());
    return punti;
}
