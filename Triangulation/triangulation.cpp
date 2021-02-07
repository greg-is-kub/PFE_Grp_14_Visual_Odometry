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

void Triangulation::pixel2cam_vector(const vector< tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > & inlier,  vector<cv::Point2f>& pts_1, vector<cv::Point2f>& pts_2)
{   
    for (int i=0; i<inlier.size(); i++) {
        // transform pixel to camera points
        pts_3.push_back(get<0>(inlier[i]).pt); // points pixel of image left
        pts_1.push_back(pixel2cam(get<0>(inlier[i]).pt, K_g));
        pts_2.push_back(pixel2cam(get<1>(inlier[i]).pt, K_d));
        }
}



void Triangulation::triangulation(const vector< tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > & inlier )
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
    
    pixel2cam_vector(inlier, pts_1, pts_2); // pixel to camera
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d); // triangulation:  2d to 3d 
    //cout<< pts_4d <<endl;
    
}

unordered_map<Pixel, Point3f> Triangulation::depth_map(){
    
    unordered_map<Pixel, Point3f> depth_map;
    for (int i = 0; i < pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        x /= 50.;
        Point3f p3f(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2,0));
        
        Pixel pix(pts_3[i].x, pts_3[i].y);
        
        depth_map[pix] = p3f; 
        //cout << pix.x <<", "<< pix.y <<"-----------"<<depth_map[pix]<<endl;
        
    }
    return depth_map;
}

void Triangulation::find_points3d(vector<KeyPoint> & kp_t, vector<KeyPoint> & kp_t1, vector<DMatch> & matches, unordered_map<Pixel, Point3f> & depth_map_t, unordered_map<Pixel, Point3f> & depth_map_t1, vector<Point3f> & cloud_t, vector<Point3f> & cloud_t1){
    for(auto m:matches){
        Point2d pt = kp_t[m.queryIdx].pt;
        Pixel px(pt.x, pt.y);
        cloud_t.push_back(depth_map_t[px]);
        
        Point2d pt1 = kp_t1[m.trainIdx].pt;
        Pixel px1(pt1.x, pt1.y);
        cloud_t1.push_back(depth_map_t1[px1]);
        
    }
}





