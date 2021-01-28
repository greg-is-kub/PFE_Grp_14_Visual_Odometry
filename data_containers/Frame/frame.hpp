#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>


class Frame{
  public :
    int index ; // frame nb
    long timestamp ;
    bool is_keyframe ;
    uint64_t keyframe_id ;
    std::pair< cv::Mat , cv::Mat > img ; //respectively left then right img
    std::vector< std::pair<cv::KeyPoint , cv::KeyPoint> > features ; //only matched features are kept
    std::pair <cv::Mat ,cv::Mat > Pose ; //transformation matrix between left and right img

    Frame(int ind , long time ,cv::Mat img_left ,cv::Mat img_right);
    ~Frame(void);


    bool is_key_frame();

};
