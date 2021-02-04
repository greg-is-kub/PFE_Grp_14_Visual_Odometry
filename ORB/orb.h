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

        tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> run(Mat, Mat);

        pair<vector<DMatch>,bool> run_temporal (Mat, vector<KeyPoint>, Mat, vector<KeyPoint>);

        void show();

        vector<DMatch> get_match(Mat d1, Mat d2);

    protected:

        std::vector<KeyPoint> kg, kd;
        std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>> list_kp;

        vector<DMatch> matches;

        Ptr<FeatureDetector> detector = ORB::create(1000);
        Ptr<DescriptorExtractor> descriptor = ORB::create();
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");


        Mat dg, dd;
        std::tuple<Mat,Mat> list_d;

        tuple<vector<KeyPoint>,vector<KeyPoint>> get_point(Mat, Mat);

        tuple<Mat,Mat> get_descriptor(Mat, vector<KeyPoint>, Mat, vector<KeyPoint>);



};

#endif // ORB_H
