#include <opencv2/core/core.hpp>
#include <pair>
#include <vector>
#include <iostream>


class Frame{
  public :
    int index ; // frame nb
    long timestamp ;
    bool is_keyframe ;
    unsigned_int keyframe_id ;
    std::pair<cv::mat ,cv::mat>> img ; //respectively left then right img
    std::vector<std::pair<cv::KeyPoint , cv::KeyPoint>> features ; //only matched features are kept
    std::pair < cv::mat , cv::mat > Pose ; //transformation matrix between left and right img

    Frame(int ind , long time , cv::mat img_left , cv::mat img_right){
      index = ind;
      img::first = img_left;
      img::second = img_right;
      timestamp = time;
    }

    bool is_key_frame();

}
