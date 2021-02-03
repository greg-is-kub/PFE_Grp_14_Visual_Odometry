#include <iostream>
#include <chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tuple>
#include <opencv2/videoio.hpp>

#include "orb.h"

using namespace std;
using namespace cv;

std::vector<std::pair<cv::Mat,cv::Mat>> video_slicer(string path_to_video, int delay)
{
    std::vector<std::pair<cv::Mat,cv::Mat>> to_return;
    pair<Mat,Mat> one_frame;
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


        cout<<n_frame<<endl;
    }

    vid.release();

    //destroyAllWindows();

    return to_return;

}

int main()
{
vector<pair<Mat,Mat>> sliced_video;

string path="test_camera_stereo.mp4";

int delay=10;
sliced_video=video_slicer(path,delay);

Orb a;
Mat IMAGE1, IMAGE2;
//initialisation variable de récupération de orb
tuple<vector<KeyPoint>,vector<KeyPoint>,vector<DMatch>,bool> gift;
vector<KeyPoint> keypoint1, keypoint2;
vector<DMatch> MATCHES;
bool STOP=false;

//parcour de toute les frames
for (auto it = begin (sliced_video); it != end (sliced_video); ++it)
{

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    //récupération image gauche et droite de la frame courante
    IMAGE1=it->first;
    IMAGE2=it->second;

    //Appelle de orb
    gift=a.run(IMAGE1,IMAGE2);
    cout<<"HERE3"<<endl;
    keypoint1=get<0>(gift);
    keypoint2=get<1>(gift);
    MATCHES=get<2>(gift);
    STOP=get<3>(gift);
    if (STOP==true)
    {
        cout<<"I WAS HERE, WITNESS ME"<<endl;
        continue;
    }



    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time cost = " << time_used.count() << " seconds. " << endl;

    a.show();
    waitKey(1);


//    cout<<"HERE2"<<endl;
//    cout<<IMAGE1.size<<endl;
//    imshow( "FrameG", IMAGE1 );
//    imshow( "FrameD", IMAGE2 );
//    waitKey(0);

}

}
