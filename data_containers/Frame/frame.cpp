#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include "frame.hpp"

Frame::Frame(int ind , long time , cv::Mat img_left , cv::Mat img_right){
  index = ind;
  img.first = img_left;
  img.second = img_right;
  timestamp = time;
}

Frame::~Frame(void){}

bool Frame::is_keyframe(){
  return true;
}
