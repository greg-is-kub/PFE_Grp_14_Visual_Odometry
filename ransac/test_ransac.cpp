#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tuple>

#include "../ORB/orb.h"
#include "../data_containers/Frame/frame.hpp"
#include "ransac.hpp"
//#include ""

#include <chrono>


using namespace std;
using namespace cv;


int main(int argc, char **argv)
{


    Orb a;
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
    cout <<"debut test ransac"<<endl;
    //a.show();
    //a.waitKey(0);



    tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> input_data ;
    get<0>(input_data)= keypoint1;
    get<1>(input_data)= keypoint2;
    get<2>(input_data)= MATCHES;
    get<3>(input_data) = true;
    int n = 5 ; //nb  points necesssary to determine [3x3] transform matrix
    long w_goal = 0.6;
    Frame f(1 , 0.0 , IMAGE1 , IMAGE2);
    Frame* ptr_f  = &f;
    float error = 10; //error in px
    cv::Matx33f K_d({ 1535.50, 0, 739.86, 0, 1535.00, 606.36, 0, 0, 1 });
    cv::Matx33f K_g ({1534.72, 0, 676.97, 0, 1535.00, 698.28, 0, 0, 1});
    cv::Matx33f R({1.00, 0.0133051171055671,
                                -0.00534470796279029, -0.0133694262332647, 0.999836410542635,
                                -0.0121823887399877,0.00518174551610382,
                                0.0122525920533588, 0.999911507835259});
    cv::Matx31f T({ -50.28, 0.077, 0.45 });
    float focal_length = 0.0036; // focale length
    Ransac ransac(input_data ,  n ,  error , w_goal ,  R , T , K_g , K_d);
    std::cout<<"object created"<<std::endl;

    ransac.check_inliers(ransac.P);

    std::cout <<"nb_samples = " << ransac.nb_samples << std::endl;
    std::cout <<"inlier rate =  " << ransac.best_w << std::endl;
    std::cout << "nb inlier = " << ransac.inlier.size()<<std::endl;
    cout << "Time cost = " << time_used.count() << " seconds. " << endl;

    return 0;
}
