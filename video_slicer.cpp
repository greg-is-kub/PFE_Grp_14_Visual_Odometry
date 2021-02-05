#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <opencv2/videoio.hpp>

#include "ORB/orb.h"
#include "ransac/ransac.hpp"
//#include "data_containers/Frame/frame.hpp"
#include "Triangulation/triangulation.hpp"
#include "ICP/ICP.h"
#include "trajectory.h"

using namespace std;
using namespace cv;

std::vector<std::pair<cv::Mat,cv::Mat>> video_slicer(string path_to_video, int delay)
{
    std::vector<std::pair<cv::Mat,cv::Mat>> to_return;
    std::pair<cv::Mat, cv::Mat> one_frame;
    int n_frame=0;
    VideoCapture vid(path_to_video);

  // Check if video opened successfully
    if(!vid.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        exit(1);
    }

    //intialisation variable
    Mat frame, ig, id;
    vid >> frame;
    int largeur=frame.cols;
    int mid=largeur/2;

    while(1)
    {

        Mat frame;
        vid >> frame;

        if (n_frame%delay==0)
        {
            ig=frame(Range::all(),Range(0,mid));
            id=frame(Range::all(),Range(mid,largeur));

            one_frame.first=ig;
            one_frame.second=id;
            to_return.push_back(one_frame);

            n_frame=0;
        }

//      imshow( "FrameG", ig );
//      imshow( "FrameD", id );
//      imshow("a",frame);

        char c=(char)waitKey(25);

        if(c==27)
        {
            break;
        }

        if (frame.empty())
        {
            break;
        }

        n_frame++;
        //cout<<n_frame<<endl;
    }

    vid.release(); //destroyAllWindows();
    return to_return;

}

/*
std::tuple< cv::KeyPoint , cv::KeyPoint > custom_pop(  std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > &tuple){
  return tuple.pop(2);
}
*/

//________________________________________________________________________________________________________
//_________________________________MAIN_MAIN_MAIN_MAIN_MAIN_MAIN__________________________________________
//________________________________________________________________________________________________________
int main(){
    //preparation of the frames
    vector<pair<Mat,Mat>> sliced_video;
    string path="test_camera_stereo.mp4";
    //we won't process every frame , only one frame every delay frame
    int delay = 10;
    std::cout<<"precomputing video and extracting frames. . .";
    sliced_video = video_slicer(path,delay);
    std::cout<<"precomputing done" << std::endl;
    ////________________________VAR INIT____________________________
    //ORB var init
    Orb orb;
    Mat IMAGE1, IMAGE2;
    tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> gift;
    vector<KeyPoint> keypoint1, keypoint2;
    vector<DMatch> MATCHES, matches_temp;
    bool STOP=false;

    //RANSAC var init
    
    cv::Matx33f K_d({ 1535.50, 0, 739.86, 0, 1535.00, 606.36, 0, 0, 1 });
    cv::Matx33f K_g ({1534.72, 0, 676.97, 0, 1535.00, 698.28, 0, 0, 1});

    //rotation and translation matrix from right to left camera
    cv::Matx33f R({1.00, 0.0133051171055671,-0.00534470796279029,
                -0.0133694262332647, 0.999836410542635,-0.0121823887399877,
                0.00518174551610382,0.0122525920533588, 0.999911507835259});
    cv::Matx31f T({ -50.28, 0.077, 0.45 });
    int n = 5 ; //nb  points necesssary to determine [3x3] transform matrix
    float w_goal = 0.6;
    //Frame* ptr_f ;
    float error = 8; //max error in px for a point to be considered an inlier

    //Frame var init
    int ind = 0; //frame index


    //ICP var init
    Triangulation triangu;
    Icp icp;
    Eigen::Matrix3d Rt;
    Eigen::Vector3d tt;
    vector<Point3f> cloud_t, cloud_t1;

    //Bundle adjustement var init


    //general purpose var
    Trajectory trajectory;
    

    std::cout<<"entering loop" <<std::endl<<std::endl;
    //iterating over every frame
    
    Mat image_pre;
    vector<KeyPoint> kp_pre;
    unordered_map<Pixel, Point3f> depth_map_pre;
    
    for (auto it = begin (sliced_video); it != end (sliced_video); ++it , ind++){

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        //getting left and right img of a given time
        IMAGE1=it->first;
        IMAGE2=it->second;

        //ORB run
        gift=orb.run(IMAGE1,IMAGE2);
        keypoint1=get<0>(gift);
        keypoint2=get<1>(gift);
        MATCHES=get<2>(gift);
        STOP=get<3>(gift);

        Ransac ransac(gift , n , error , w_goal , R , T , K_g , K_d);
        ransac.check_inliers(ransac.P);

        if (STOP==true){
            cout<<"EMPTY / BLACK FRAME NOT TAKEN IN COUNT "<<endl;
            continue;
        }

        //Frame f(ind , IMAGE1 , IMAGE2);
        //f.features = ransac.inlier ; //we only ned to keep the keypoints, fuck  DMATCH
        //std::cout<<f.features.size() <<std::endl;//<<"\t"<<ransac.inlier.size() <<std::endl;
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "Time cost = " << time_used.count() << " seconds. " << endl;
        
        // triangulation 
        triangu.triangulation(ransac.inlier);
        unordered_map<Pixel, Point3f> depth_map_t = triangu.depth_map();
        
        if(it != begin(sliced_video)){
        
            pair<vector<DMatch>,bool> temp_m = orb.run_temporal(IMAGE1, keypoint1, image_pre, kp_pre);
            matches_temp = get<0>(temp_m);
            
            triangu.find_points3d(keypoint1, kp_pre, matches_temp, depth_map_t, depth_map_pre, cloud_t, cloud_t1);
            
            icp.getTransform(cloud_t, cloud_t1, Rt, tt);
            
            trajectory.add_to_path(Rt, tt);
            trajectory.show();
        }
        
        
        

        //    orb.show();
        //    waitKey(1);
  }

}
