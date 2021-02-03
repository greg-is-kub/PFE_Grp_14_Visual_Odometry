#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include "frame.hpp"

Frame::Frame(int ind , /*long time ,*/ cv::Mat img_left , cv::Mat img_right){
  this->index = ind;
  this->img.first = img_left;
  this->img.second = img_right;
  //this->timestamp = time;
}

Frame::~Frame(void){}

bool Frame::is_keyframe(){
  return true;
}
