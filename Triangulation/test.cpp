#include <iostream>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "../ORB/orb.h"
#include "../data_containers/Frame/frame.hpp"
#include "../ransac/ransac.hpp"
#include "../Triangulation/triangulation.hpp"
#include "../ICP/ICP.h"
//#include ""

#include <chrono>


using namespace std;
using namespace cv;


int main(int argc, char **argv)
{


    Orb a;
    Mat IMAGE1=imread(argv[1], IMREAD_COLOR);
    Mat IMAGE2=imread(argv[2], IMREAD_COLOR);
    Mat IMAGE3=imread(argv[3], IMREAD_COLOR);
    Mat IMAGE4=imread(argv[4], IMREAD_COLOR);



    //Appelle et récupéartion de la partie ORB
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> gift1, gift2;
    vector<KeyPoint> keypoint1, keypoint2, keypoint3, keypoint4;
    vector<DMatch> MATCHES1, MATCHES2, MATCHES3;

    bool STOP1, STOP2;
    gift1=a.run(IMAGE1,IMAGE2);
    keypoint1=get<0>(gift1);
    keypoint2=get<1>(gift1);
    MATCHES1=get<2>(gift1);
    STOP1=get<3>(gift1);
    
    
    gift2=a.run(IMAGE3,IMAGE4);
    keypoint3=get<0>(gift2);
    keypoint4=get<1>(gift2);
    MATCHES2=get<2>(gift2);
    STOP2=get<3>(gift2);


    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout <<"orb for 2 frames (4 images)"<< time_used.count() <<endl;
    //a.show();
    //a.waitKey(0);


    int n = 5 ; //nb  points necesssary to determine [3x3] transform matrix
    float w_goal = 0.6;
    //Frame f(1 , 0.0 , IMAGE1 , IMAGE2);
    //Frame* ptr_f  = &f; 
    float error = 10; //error in px
    cv::Matx33f K_d({ 1535.50, 0, 739.86, 0, 1535.00, 606.36, 0, 0, 1 });
    cv::Matx33f K_g ({1534.72, 0, 676.97, 0, 1535.00, 698.28, 0, 0, 1});
    cv::Matx33f R({1.00, 0.0133051171055671,
                                -0.00534470796279029, -0.0133694262332647, 0.999836410542635,
                                -0.0121823887399877,0.00518174551610382,
                                0.0122525920533588, 0.999911507835259});
    cv::Matx31f T({ -50.28, 0.077, 0.45 });
    //float focal_length = 0.0036; // focale length
    Ransac ransac1(gift1 ,  n ,  error , w_goal ,  R , T , K_g , K_d);
    Ransac ransac2(gift2 ,  n ,  error , w_goal ,  R , T , K_g , K_d);
    ransac1.check_inliers(ransac1.P);
    ransac2.check_inliers(ransac2.P);
    
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    chrono::duration<double> time_used2 = chrono::duration_cast<chrono::duration<double>>(t3 - t2);
    cout << "Ransac Time cost = " << time_used2.count() << " seconds. " << endl;
    
    std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > inlier_t = ransac1.inlier;
    std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > inlier_t1 = ransac2.inlier;
    
    Triangulation triangu;
    triangu.triangulation(inlier_t);
    unordered_map<Pixel, Point3f> depth_map_t = triangu.depth_map();
    
    triangu.triangulation(inlier_t1);
    unordered_map<Pixel, Point3f> depth_map_t1 = triangu.depth_map();
    
    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_used3 = chrono::duration_cast<chrono::duration<double>>(t4 - t3);
    cout << "Triangulation Time cost = " << time_used3.count() << " seconds. " << endl;
    
    // -----------------------------------temporal matching------------------------------------------------
    
    pair<vector<DMatch>,bool> temp_m = a.run_temporal(IMAGE1, keypoint1, IMAGE3, keypoint3);
    MATCHES3 = get<0>(temp_m);
    
    vector<Point3f> cloud_t, cloud_t1;
    
    triangu.find_points3d(keypoint1, keypoint3, MATCHES3, depth_map_t, depth_map_t1, cloud_t, cloud_t1);
    
    cout << MATCHES3.size() <<endl;
    cout << cloud_t.size() <<endl;
    cout << cloud_t1.size() <<endl;
    
    Icp icp;
    Eigen::Matrix3d Rt;
    Eigen::Vector3d t;
    icp.getTransform(cloud_t, cloud_t1, Rt, t);
    
    chrono::steady_clock::time_point t5 = chrono::steady_clock::now();
    chrono::duration<double> time_used4 = chrono::duration_cast<chrono::duration<double>>(t5 - t4);
    cout << "Triangulation Time cost = " << time_used4.count() << " seconds. " << endl;
    cout << R <<"--------" << t << endl;
    
    return 0;
}
