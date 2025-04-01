#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "Cone.hpp"

//https://github.com/siromermer/OpenCV-Projects-cpp-python/blob/master/ObjectTracking-ORB/README.md

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

cv::Mat calcolaTraslazione(const cv::Mat& img1, const cv::Mat& img2, const cv::Mat& K) {
    // Rileva i keypoints e i descrittori usando SIFT
    cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    sift->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    sift->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);
    
    // Matcher per confrontare i descrittori
    cv::BFMatcher matcher(cv::NORM_L2, true);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    
    // Estrai i punti corrispondenti
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& match : matches) {
        pts1.push_back(keypoints1[match.queryIdx].pt);
        pts2.push_back(keypoints2[match.trainIdx].pt);
    }
    
    // Calcola la Matrice Essenziale
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0);
    
    // Decomponi la Matrice Essenziale per ottenere Rotazione e Traslazione
    cv::Mat R, t;
    cv::recoverPose(E, pts1, pts2, K, R, t);
    
    return t; // Restituisce il vettore di traslazione normalizzato
}

int main() {
    Mat image = imread("../src/img/frame_1.png");
    if (image.empty()) {
        std::cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }
    Mat image2 =  imread("../src/img/frame_2.png");
    if (image2.empty()) {
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

    std::vector<Cone> orange_cones2 = get_cones_boxes(image2,lower_orange , upper_orange, 0.66 , 500);
    std::vector<Cone> blue_cones2 = get_cones_boxes(image2,lower_blue , upper_blue, 2 , 30);
    std::vector<Cone> yellow_cones2 = get_cones_boxes(image2,lower_yellow , upper_yellow, 2 , 30);

    //ordinare i coni in base all'area (per via della prospettiva)

    std::vector<std::vector<Cone>> cones = {orange_cones , blue_cones , yellow_cones};
    std::vector<std::vector<Cone>> cones2 = {orange_cones2 , blue_cones2 , yellow_cones2};
    std::vector<Point> Rcones , Lcones;
    std::vector<Point2f> Rcones2 , Lcones2;
    
    
    for (std::vector<Cone> row: cones){
        for (Cone cono : row){
            if (cono.getBox().x < (image.size().width / 2)){
                Lcones.push_back(cono.getPos());
            } else{
                Rcones.push_back(cono.getPos());
            }
            
        }
    }

    for (std::vector<Cone> row: cones2){
        for (Cone cono : row){
            if (cono.getBox().x < (image.size().width / 2)){
                Lcones2.push_back(cono.getPos2f());
            } else{
                Rcones2.push_back(cono.getPos2f());
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

    Mat gray1 = imread("../src/img/frame_1.png" , IMREAD_GRAYSCALE);
    Mat gray2 = imread("../src/img/frame_2.png" , IMREAD_GRAYSCALE);
    if (gray1.empty() || gray2.empty()) {
        std::cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }

    cv::Mat K = (cv::Mat_<double>(3,3) << 700, 0  , 320,
                                          0  , 700, 240,
                                          0  , 0  , 1  );
    cv::Mat t = calcolaTraslazione(gray1, gray2, K);
    std::cout << "Vettore di Traslazione:\n" << t << std::endl;
    
    imshow("Image" , image);
    waitKey(0);
    return 0;
}