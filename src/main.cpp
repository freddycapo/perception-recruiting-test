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

/*void feature_extraction(Mat& img_1, Mat& img_2, std::vector<KeyPoint>& keypoints_1, std::vector<KeyPoint>& keypoints_2){
    //-- inizializzazione
    //std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //KeyPoints Detection using FAST Algorithm
    /*detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- Passaggio 2: calcolare il descrittore BRIEF in base alla posizione dell'angolo
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("ORB",outimg1);

    //-- Passaggio 3: Abbina i descrittori BRIEF nelle due immagini, utilizzando la distanza di Hamming
    std::vector<DMatch> matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- Passaggio 4: filtra le coppie di punti corrispondenti
    double min_dist=10000, max_dist=0;

    //Trova la distanza minima e la distanza massima tra tutte le corrispondenze, ovvero la distanza tra i due insiemi di punti più simili e quelli meno simili

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //Quando la distanza tra i descrittori è maggiore del doppio della distanza minima, l'abbinamento è considerato errato. Ma a volte la distanza minima sarà molto piccola e come limite inferiore viene fissato un valore empirico di 30.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }


    
    //-- Passaggio 5: traccia i risultati corrispondenti
    //Mat img_match;
    Mat img_goodmatch;
    //drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    //imshow ( "1", img_match );
    imshow ( "2", img_goodmatch );
    waitKey(0);
}*/

void calcolaTraslazione(const Mat& img1,
    const Mat& img2,
    const cv::Mat& K){
    
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
    for (int i = 0;i < track.second.size();++i){
        if( i < track2.second.size()){
            line(image , track.second[i] , track2.second[i] , Scalar(0,0,255),3);
            line(image , track.first[i] , track2.first[i] , Scalar(0,0,255),3);
        }
    }

    cv::Mat K = (cv::Mat_<double>(3,3) <<
    185.36, 0,     320,
    0,      185.36, 240,
    0,      0,     1);

    calcolaTraslazione(image , image2 ,  K);

    //std::cout << "W:" << image.size().width << " H: " << image.size().height;
    
    imshow("Frame_1" , image);
    imshow("Frame_2" , image2);
    waitKey(0);
    return 0;
}