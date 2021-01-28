#include <opencv2/core/core.hpp>
#include <pair>
#include <vector>
#include <iostream>

Frame::Frame(int ind , long time , cv::mat img_left , cv::mat img_right){
  index = ind;
  img::first = img_left;
  img::second = img_right;
  timestamp = time;
}
