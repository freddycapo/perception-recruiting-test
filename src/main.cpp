#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "Cone.hpp"

using namespace cv;

std::vector<Cone> get_cones_boxes(Mat& Img  , Scalar lower_bound , Scalar upper_bound , const float WH_TRESHOLD ,  const float AREA_TRESHOLD){
    Mat hsv , mask;
    float wh_rateo;
    std::vector<Cone> ret_vec;

    cvtColor(Img , hsv , COLOR_BGR2HSV);
    inRange(hsv , lower_bound, upper_bound, mask);
    
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours){
        std::vector<Point> approx;
        approxPolyDP(contour, approx, 10, true);
        
        double area = cv::contourArea(contour);
        cv::Rect bounding_box = cv::boundingRect(contour);
        
        wh_rateo = bounding_box.width / bounding_box.height;

        if (area > AREA_TRESHOLD && wh_rateo < WH_TRESHOLD){
            ret_vec.push_back(Cone(bounding_box , lower_bound , upper_bound, Point(bounding_box.x , bounding_box.y)));
        }
    }
    

    return ret_vec;
}

Point get_mid_point(const Point& p1 ,const Point& p2){
    return Point((p1.x + p2.x)/2 , (p1.y + p2.y)/2);
}

int main() {
    Mat image = imread("../src/img/frame_1.png");
    if (image.empty()) {
        std::cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }

    Scalar lower_orange(114, 0, 145);  // Limite inferiore (HSV)
    Scalar upper_orange(180, 255, 255); // Limite superiore (HSV)

    Scalar lower_blue(40 , 70 , 70);
    Scalar upper_blue(125 , 255 , 255);

    Scalar lower_yellow(14 , 100 , 184);
    Scalar upper_yellow(23 , 255 , 255);

    std::vector<Cone> orange_cones = get_cones_boxes(image,lower_orange , upper_orange, 0.66 , 500);
    std::vector<Cone> blue_cones = get_cones_boxes(image,lower_blue , upper_blue, 2 , 30);
    std::vector<Cone> yellow_cones = get_cones_boxes(image,lower_yellow , upper_yellow, 2 , 30);

    //ordinare i coni in base all'area (per via della prospettiva)

    std::vector<std::vector<Cone>> cones = {orange_cones , blue_cones , yellow_cones};
    std::vector<Point> Rcones , Lcones;
    
    for (std::vector<Cone> row: cones){
        for (Cone cono : row){
            if (cono.getBox().x < (image.size().width / 2)){
                Lcones.push_back(cono.getPos());
            } else{
                Rcones.push_back(cono.getPos());
            }
            
        }
    }

    std::vector<Point> track;
    for(int i = 0; i < Rcones.size(); ++i){
        track.push_back(get_mid_point(Rcones[i] , Lcones[i]));
    }

    polylines(image, Rcones ,false ,  Scalar(0,255,0),3);
    polylines(image, Lcones ,false ,  Scalar(0,255,0),3);
    polylines(image, track, false , Scalar(255,0,0) , 3);

    int temp = 0;
    for (std::vector<Cone> row: cones){
        temp += 1;
        for (Cone cono : row){
            rectangle(image , cono.getBox() , Scalar(0,255,0) );
            std::cout <<"Area " << temp << ": " << cono.getBox().area()<< std::endl;
        }
    }
/*
    for(int i = 0; i < orange_cones.size(); i += 2){
        line(image , 
            Point(orange_cones[i].getBox().x + (orange_cones[i].getBox().width / 2) , orange_cones[i].getBox().y ),
            Point(orange_cones[i+1].getBox().x + (orange_cones[i+1].getBox().width / 2) , orange_cones[i+1].getBox().y ),
            Scalar(0,0,255));
            //putText(image ,std::to_string(i) ,Point(orange_cones[i].getBox().x + (orange_cones[i].getBox().width / 2) , orange_cones[i].getBox().y ) , cv::FONT_HERSHEY_SIMPLEX , 0.5,Scalar(255,0,255));

    }

    
    if (blue_cones.size() <= yellow_cones.size()){
        for(int i = 0; i < blue_cones.size(); ++i){
            line(image ,
                Point(blue_cones[i].getBox().x + (blue_cones[i].getBox().width / 2) , blue_cones[i].getBox().y ),
                Point(yellow_cones[i].getBox().x + (yellow_cones[i].getBox().width / 2) , yellow_cones[i].getBox().y),
                Scalar(0,255,0));
                //putText(image ,std::to_string(i) ,Point(blue_cones[i].getBox().x + (blue_cones[i].getBox().width / 2) , blue_cones[i].getBox().y ) , cv::FONT_HERSHEY_SIMPLEX , 0.5,Scalar(255,0,255));

        }
    } else{
        for(int i = 0; i < yellow_cones.size(); ++i){
            line(image ,
                Point(blue_cones[i].getBox().x + (blue_cones[i].getBox().width / 2) , blue_cones[i].getBox().y ),
                Point(yellow_cones[i].getBox().x + (yellow_cones[i].getBox().width / 2) , yellow_cones[i].getBox().y ),
                Scalar(0,255,0));
                //putText(image ,std::to_string(i) ,Point(blue_cones[i].getBox().x + (blue_cones[i].getBox().width / 2) , blue_cones[i].getBox().y ) , cv::FONT_HERSHEY_SIMPLEX , 0.5,Scalar(255,0,255));

        }
    }
    */

    imshow("Image" , image);
    waitKey(0);
    return 0;
}