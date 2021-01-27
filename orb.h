#ifndef ORB_H
#define ORB_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tuple>

using namespace std;
using namespace cv;

class Orb
{
    public:
        Orb();
        ~Orb();

        Mat imgG;
        Mat imgD;

        tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>> run(Mat, Mat);

        void show();

    protected:

        std::vector<KeyPoint> kg, kd;
        std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>> list_kp;

        vector<DMatch> matches;

        Ptr<FeatureDetector> detector = ORB::create();
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");


        Mat dg, dd;
        std::tuple<Mat,Mat> list_d;

        tuple<vector<KeyPoint>,vector<KeyPoint>> get_point(Mat, Mat);

        tuple<Mat,Mat> get_descriptor(Mat, vector<KeyPoint>, Mat, vector<KeyPoint>);

        vector<DMatch> get_match(Mat d1, Mat d2);

};

#endif // ORB_H
