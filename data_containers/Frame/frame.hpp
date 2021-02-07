#ifndef FRAME
#define FRAME
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include <opencv2/core/affine.hpp>
#include "../../ORB/orb.h"
#include "../../ransac/ransac.hpp"
#include "../../Triangulation/triangulation.hpp"
#include "../../ICP/ICP.h"

using namespace std;
using namespace cv;

class Frame{
  protected :
    uint64_t index; // frame nb
    //float timestamp;
    bool keyframe;
    uint64_t keyframe_id;
    std::pair< cv::Mat , cv::Mat > img; //respectively left then right img
    vector<KeyPoint> keypoints_left;
    //std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint /*, cv::Mat*/ , cv::DMatch > > features ; //only matched features are kept
    //cv::Affine3f spatial_pose ; transformation matrix between left and right img
    //cv::Affine3f temporal_pose ; transformation matrix between position at t_(n) and t_(n+1)
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Translation;
    unordered_map<Pixel, Point3f> depth_map;
    
    
  public:
    Frame(int ind , /*floattime, */ cv::Mat img_left ,cv::Mat img_right);
    ~Frame(void);
    Mat get_img_left(uint64_t &);
    vector<KeyPoint> get_keypoints(uint64_t &);
    Eigen::Matrix3d get_R(uint64_t &);
    Eigen::Vector3d get_T(uint64_t &);


    bool is_keyframe();

};



#endif
