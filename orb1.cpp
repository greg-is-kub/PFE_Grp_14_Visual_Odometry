#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tuple>

#include "orb.h"



#include <chrono>


using namespace std;
using namespace cv;

//class Orb
//    {
//
//        public:
//
//        Orb(){}
//        ~Orb(){}
//
//
//        Mat imgG;
//        Mat imgD;
//
//        std::vector<KeyPoint> kg, kd;
//        std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>> list_kp;
//
//        vector<DMatch> matches;
//
//        Ptr<FeatureDetector> detector = ORB::create();
//        Ptr<DescriptorExtractor> descriptor = ORB::create();
//        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//
//
//        Mat dg, dd;
//        std::tuple<Mat,Mat> list_d;
//
//
//        tuple<vector<KeyPoint>,vector<KeyPoint>> get_point(Mat img1, Mat img2)
//        {
//            //Ptr<FeatureDetector> detector = ORB::create();
//
//            std::vector<cv::KeyPoint> kp1, kp2;
//
//            detector->detect(img1,kp1);
//            detector->detect(img2,kp2);
//
//            std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>> to_return (kp1,kp2);
//            return to_return;
//        }
//
//        tuple<Mat,Mat> get_descriptor(Mat img1, vector<KeyPoint> kp1, Mat img2, vector<KeyPoint> kp2)
//        {
//            Mat d1, d2;
//
//            //Ptr<DescriptorExtractor> descriptor = ORB::create();
//            descriptor->compute(img1, kp1, d1);
//            descriptor->compute(img2, kp2, d2);
//
//
//
//
//
//            std::tuple<Mat,Mat> to_return (d1,d2);
//            return to_return;
//        }
//
//    vector<DMatch> get_match(Mat d1, Mat d2)
//    {
//        //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//        vector<DMatch> matches;
//
//        matcher->match(d1, d2, matches);
//
//        auto min_max = minmax_element(matches.begin(), matches.end(),
//                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
//        double min_dist = min_max.first->distance;
//        //double max_dist = min_max.second->distance;
//        //printf("-- Max dist : %f \n", max_dist);
//        //printf("-- Min dist : %f \n", min_dist);
//
//        vector<DMatch> good_matches;
//        for (int i = 0; i < d1.rows; i++) {
//            if (matches[i].distance <= max(2 * min_dist, 30.0)) {
//                good_matches.push_back(matches[i]);
//            }
//        }
//
//        return good_matches;
//    }
//
//
//    tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>> run(Mat img1, Mat img2)
//    {
//        imgG=img1;
//        imgD=img2;
//
//        tuple<vector<KeyPoint>,vector<KeyPoint>> l_k;
//        tuple<Mat,Mat> l_d;
//        //vector<DMatch> matches;
//        l_k=get_point(imgG,imgD);
//        kg=get<0>(l_k);
//        kd=get<1>(l_k);
//
//        l_d=get_descriptor(imgG,kg,imgD,kd);
//        dg=get<0>(l_d);
//        dd=get<1>(l_d);
//        matches=get_match(dg,dd);
//        //a.matches=matches;
//
//        tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>> to_return (kg,kd,matches);
//        return to_return;
//    }
//
//    };

int main(int argc, char **argv)
{

    Orb a;

//    a.imgG=imread(argv[1], IMREAD_COLOR);
//    a.imgD=imread(argv[2], IMREAD_COLOR);
//
//    tuple<vector<KeyPoint>,vector<KeyPoint>> l_k;
//    tuple<Mat,Mat> l_d;
//    vector<DMatch> matches;
//
//    l_k=a.get_point(a.imgG,a.imgD);
//    a.kg=get<0>(l_k);
//    a.kd=get<1>(l_k);
//
//    l_d=a.get_descriptor(a.imgG,a.kg,a.imgD,a.kd);
//    a.dg=get<0>(l_d);
//    a.dd=get<1>(l_d);
//    a.matches=a.get_match(a.dg,a.dd);
//    //a.matches=matches;

//    Mat img_match;
//
//    drawMatches(a.imgG, a.kg, a.imgD, a.kd, a.matches, img_match);
//
//    imshow("all matches", img_match);
//
//    waitKey(0);



    Mat IMAGE1=imread(argv[1], IMREAD_COLOR);
    Mat IMAGE2=imread(argv[2], IMREAD_COLOR);



    //Appelle et récupéartion de la partie ORB
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>> gift;
    vector<KeyPoint> keypoint1, keypoint2;
    vector<DMatch> MATCHES;
    gift=a.run(IMAGE1,IMAGE2);
    keypoint1=get<0>(gift);
    keypoint2=get<1>(gift);
    MATCHES=get<2>(gift);

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time cost = " << time_used.count() << " seconds. " << endl;

    a.show();


//    Mat img_match;
//
//    drawMatches(a.imgG, keypoint1, a.imgD, keypoint2, MATCHES, img_match);
//    namedWindow("Display Matches",WINDOW_NORMAL);
//    resizeWindow("Display Matches",1400,900);
//    imshow("Display Matches", img_match);
//
//    waitKey(0);










    //std::string image_path = samples::findFile("1.png");
    //Mat img = imread(image_path, IMREAD_COLOR);
    //if(img.empty())
    //{
    //    std::cout << "Could not read the image: " << image_path << std::endl;
    //    return 1;
    //}
    //imshow("Display window", img);
    //int k = waitKey(0); // Wait for a keystroke in the window
    //if(k == 's')
    //{
    //    imwrite("1.jpg", img);
    //}
    return 0;
}
