#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "Cone.hpp"
#include "Detector.hpp"

//https://github.com/siromermer/OpenCV-Projects-cpp-python/blob/master/ObjectTracking-ORB/README.md

using namespace cv;


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
    Mat image2 =  imread("../src/img/frame_2.png");
    if (image.empty() || image2.empty()) {
        std::cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }
    
    Detector detector(image);
    Detector detector2(image2);

    std::vector<Cone> coni = detector.get_cones();

    //ordinare i coni in base all'area (per via della prospettiva)

    
    
    

    /*polylines(image, Rcones ,false ,  Scalar(0,255,0),3);
    polylines(image, Lcones ,false ,  Scalar(0,255,0),3);
    polylines(image, track, false , Scalar(255,0,0) , 3);
    */
    

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