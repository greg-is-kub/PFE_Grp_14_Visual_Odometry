#include <opencv2/core/core.hpp>
#include <vector>
#include <iostream>
#include "frame.hpp"

Frame::Frame(int ind , long time , cv::Mat img_left , cv::Mat img_right){
  this->index = ind;
  this->img.first = img_left;
  this->img.second = img_right;
  //this->timestamp = time;
}

Frame::~Frame(void){}

bool Frame::is_keyframe(){
  return true;
}

cv::Mat Frame::get_l_img(){
  //return left img
  return this->img.first;
}

std::vector<cv::KeyPoint> Frame::get_l_kp(){
  //return keypoints of left picture
  std::vector<cv::KeyPoint> res;
  for(int i = 0 ; i < ( this->features.size() ) ; i ++)
    res.push_back(std::get<0>( this->features.at(i) ) );
  return res;
}
