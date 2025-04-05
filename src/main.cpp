#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>
#include <vector>

#include "Cone.hpp"
#include "Detector.hpp"

//https://github.com/siromermer/OpenCV-Projects-cpp-python/blob/master/ObjectTracking-ORB/README.md

// ORB per 2 frame : https://github.com/sunzuolei/orb

using namespace cv;

void calcolaTraslazione(const Mat& img1, const Mat& img2, const cv::Mat& K){
    
    Ptr<ORB> orb = ORB::create();
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    orb->detectAndCompute(img1, noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, noArray(), keypoints2, descriptors2);

    // Match tra descrittori con BFMatcher
    BFMatcher matcher(NORM_HAMMING);
    std::vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // Filtra i match migliori (opzionale)
    std::sort(matches.begin(), matches.end(),
              [](const DMatch& a, const DMatch& b) {
                  return a.distance < b.distance;
              });
    matches.resize(100); // tieni i migliori 100

    // Estrai punti corrispondenti
    std::vector<Point2f> punti1, punti2;
    for (const auto& m : matches) {
        punti1.push_back(keypoints1[m.queryIdx].pt);
        punti2.push_back(keypoints2[m.trainIdx].pt);
    }

    // Calcola matrice essenziale
    Mat E = findEssentialMat(punti1, punti2, K, RANSAC, 0.999, 1.0);

    // Recupera rotazione e traslazione
    Mat R, t;
    int inliers = recoverPose(E, punti1, punti2, K, R, t);

    std::cout << "Inlier corrispondenze: " << inliers << std::endl;
    std::cout << "Rotazione R:\n" << R << std::endl;
    std::cout << "Versore Traslazione t (unità relative):\n" << t << std::endl;
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
    std::vector<Cone> coni2 = detector2.get_cones();

    //ordinare i coni in base all'area (per via della prospettiva)

    std::pair<std::vector<Point2f> , std::vector<Point2f>> track = detector.extract_track_edges(coni);
    std::pair<std::vector<Point2f> , std::vector<Point2f>> track2 = detector2.extract_track_edges(coni2);
    
    //Disegna linee di spostamento
    for (int i = 0;i < track.second.size() && i < track2.second.size();++i){
        line(image , track.second[i] , track2.second[i] , Scalar(0,0,255),3);
        line(image , track.first[i] , track2.first[i] , Scalar(0,0,255),3);
    }

    cv::Mat K = (cv::Mat_<double>(3,3) <<  //  K = matrice intrinseca della fotocamera (valori stimati ad occhio)
    185.36, 0,     320,
    0,      185.36, 240,
    0,      0,     1);

    calcolaTraslazione(image , image2 ,  K);

    imshow("Frame_1" , image);
    imshow("Frame_2" , image2);
    waitKey(0);
    return 0;
}