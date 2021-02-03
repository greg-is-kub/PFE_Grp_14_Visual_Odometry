#include <opencv2/core/core.hpp>
#include <iostream>
int main(){
  cv::Mat test(cv::Size(3, 1), CV_64FC1);
  //std::get<0,0>(test) = 1;
  test.at<int>(1,0) = 2;
  test.at<int>(2,0) = 3;

  //std::cout<< "oui" <<std::endl << test.at<int>(1,0) <<std::endl;

  cv::Matx41f m({1.0,2.0,3.0,4.0});
  std::cout << m(2,1)<<std::endl;

  return 0;
}
