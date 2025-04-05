#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

#include "Cone.hpp"
#include "Detector.hpp"

using namespace cv;

typedef std::pair<std::vector<Point2f> , std::vector<Point2f>> Traccia_t;

void calcTVersor(const Mat& img1, const Mat& img2, const cv::Mat& K){
    
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

    //std::cout << "Inlier corrispondenze: " << inliers << std::endl;
    //std::cout << "Rotazione R:\n" << R << std::endl;
    std::cout << "Versore Traslazione t (unità relative):\n" << t << std::endl;
}

void drawTrack(Mat& img , Traccia_t& track){
    std::vector<Point> Rpoints , Lpoints;
    for (Point2f p : track.first){
        Lpoints.push_back(p);
    }
    for (Point2f p : track.second){
        Rpoints.push_back(p);
    }
    polylines(img , Lpoints , false , Scalar(0,255,0) , 3);
    polylines(img , Rpoints , false , Scalar(0,255,0) , 3);
}

int main() {
    //Task 1
    Mat image = imread("../src/img/frame_1.png");
    Mat image2 =  imread("../src/img/frame_2.png");
    if (image.empty() || image2.empty()) {
        std::cerr << "Errore: Impossibile caricare l'immagine!\n";
        return -1;
    }
    
    Detector detector(image), detector2(image2);
    
    // Task 2 and 3
    std::vector<Cone> coni = detector.get_cones();
    std::vector<Cone> coni2 = detector2.get_cones();

    // Task 4
    Traccia_t track = detector.extract_track_edges(coni);
    Traccia_t track2 = detector2.extract_track_edges(coni2);
    
    drawTrack(image , track);
    drawTrack(image2 , track2);

    // Task 5
    cv::Mat K = (cv::Mat_<double>(3,3) <<  //  K = matrice intrinseca della fotocamera (valori stimati ad occhio)
    185.36, 0,      320,
    0,      185.36, 240,
    0,      0,      1   );

    calcTVersor(image , image2 ,  K);

    imshow("Frame_1" , image);
    imshow("Frame_2" , image2);
    waitKey(0);
    return 0;
}