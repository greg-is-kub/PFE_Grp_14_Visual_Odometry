#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "triangulation.hpp"

using namespace std;
using namespace cv;

Point2f pixel2cam(const Point2d &p, const Mat &K) {
  return Point2f (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void Triangulation::pixel2cam_vector(const vector<cv::KeyPoint>& keypoint_1, const vector<cv::KeyPoint>& keypoint_2, const std::vector<DMatch>& matches, vector<cv::Point2f>& pts_1, vector<cv::Point2f>& pts_2, vector<Point2d> &pts_3, vector<Point2d> & pts_4)
{   
    for (DMatch m:matches) {
        // 将像素坐标转换至相机坐标
        pts_3.push_back(keypoint_1[m.queryIdx].pt);
        pts_4.push_back(keypoint_2[m.trainIdx].pt);
        pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K_g));
        pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K_d));
        }
        
}

unordered_map<Pixel, Point3f> Triangulation::triangulation(const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const std::vector<DMatch> &matches)
//void triangulation(const vector<KeyPoint> &keypoint_1, const vector<KeyPoint> &keypoint_2, const std::vector<DMatch> &matches, Mat &cloud)
{

    
    Mat T1 = (Mat_<float>(3, 4) <<
                            1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0); 
    Mat T2 = (Mat_<float>(3, 4) <<
                            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    
    vector<Point2f> pts_1, pts_2; // points repere camera
    vector<Point2d> pts_3, pts_4; // points pixel
    
    /*
    for(auto iter:keypoints){
        Mat keypoints_1 = iter->first;
        Mat keypoints_2 = iter->second;
        pts_1.push_back(pixel2cam(keypoints_1.pt, K_d));
        pts_2.push_back(pixel2cam(keypoints_2.pt, K_g));
    }*/
    
    pixel2cam_vector(keypoint_1, keypoint_2, matches, pts_1, pts_2, pts_3, pts_4);
    
    unordered_map<Pixel, Point3f> depth_map;
    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);
    //cout<< pts_4d <<endl;

    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        Point3f p3f(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2,0));
        
        Pixel pix(pts_3[i].x, pts_3[i].y);
        
        depth_map[pix] = p3f; 
        cout << pix.x <<", "<< pix.y <<"-----------"<<depth_map[pix]<<endl;
        
    }
    return depth_map;
    
}





