#ifndef FRAME
#define FRAME
#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include <opencv2/core/affine.hpp>

class Frame{
  public :
    int index ; // frame nb
    //float timestamp ;
    bool keyframe ;
    uint64_t keyframe_id ;
    std::pair< cv::Mat , cv::Mat > img ; //respectively left then right img
    std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > features ; //only matched features are kept
    cv::Affine3f spatial_pose ; //transformation matrix between left and right img
    cv::Affine3f temporal_pose ; //transformation matrix between position at t_(n) and t_(n+1)

    Frame(int ind , float time, cv::Mat img_left ,cv::Mat img_right);
    ~Frame(void);

    cv::Mat get_l_img();

    std::vector<cv::KeyPoint> get_l_kp();

    bool is_keyframe();

};

#endif
