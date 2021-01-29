#include "orb.h"

//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <tuple>

using namespace std;
using namespace cv;

Orb::Orb()
{
    //ctor
}

Orb::~Orb()
{
    //dtor
}

tuple<vector<KeyPoint>,vector<KeyPoint>> Orb::get_point(Mat img1, Mat img2)
        {
            std::vector<cv::KeyPoint> kp1, kp2;

            detector->detect(img1,kp1);
            detector->detect(img2,kp2);

            std::tuple<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>> to_return (kp1,kp2);
            return to_return;
        }

tuple<Mat,Mat> Orb::get_descriptor(Mat img1, vector<KeyPoint> kp1, Mat img2, vector<KeyPoint> kp2)
        {
            Mat d1, d2;

            descriptor->compute(img1, kp1, d1);
            descriptor->compute(img2, kp2, d2);

            std::tuple<Mat,Mat> to_return (d1,d2);
            return to_return;
        }

vector<DMatch> Orb::get_match(Mat d1, Mat d2)
    {

        vector<DMatch> matches;

        matcher->match(d1, d2, matches);

        auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
        double min_dist = min_max.first->distance;
        //double max_dist = min_max.second->distance;
        //printf("-- Max dist : %f \n", max_dist);
        //printf("-- Min dist : %f \n", min_dist);

        vector<DMatch> good_matches;
        for (int i = 0; i < d1.rows; i++) {
            if (matches[i].distance <= max(2 * min_dist, 30.0)) {
                good_matches.push_back(matches[i]);
            }
        }

        return good_matches;
    }

tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> Orb::run(Mat img1, Mat img2)
    {
        bool flag=false;
        imgG=img1;
        imgD=img2;

        tuple<vector<KeyPoint>,vector<KeyPoint>> l_k;
        tuple<Mat,Mat> l_d;

        l_k=get_point(imgG,imgD);
        kg=get<0>(l_k);
        kd=get<1>(l_k);
        if (kg.empty() || kd.empty())
        {
            flag=true;
        }

        if (flag==false)
        {
            l_d=get_descriptor(imgG,kg,imgD,kd);
            dg=get<0>(l_d);
            dd=get<1>(l_d);
        }

        if (flag==false)
        {
            matches=get_match(dg,dd);
        }
        if (matches.empty())
        {
            flag=true;
        }
        tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> to_return (kg,kd,matches,flag);
        return to_return;
    }

void Orb::show()
    {
        Mat img_match;

        drawMatches(imgG, kg, imgD, kd, matches, img_match);
        namedWindow("Display Matches",WINDOW_NORMAL);
        resizeWindow("Display Matches",1400,900);
        imshow("Display Matches", img_match);

        waitKey(0);
    }
