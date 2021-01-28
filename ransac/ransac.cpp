#include "ransac.hpp"
#include <set>
#include <opencv2/core/core.hpp>


//______________________CONSTRUCTOR & DESTRUCTOR_____________________
Ransac::Ransac(std::tuple< <cv::KeyPoint*> , <cv::KeyPoint*> , < cv::DMatch* > > datapoint , uint64_t n , float error , float w , Frame*  ptr_f){
    data = datapoints; //points matched
    n = nb_point; // number of points necessary to build a model
    max_error = error;  //max error value for a point to be considered an inlier
    w_goal = w; //minimum inlier rate wanted
    ptr_frame = ptr_f; //pointer on the frame where we will stack data
}

~Ransac(){}

//______________________MAIN CLASS FUNCTION _____________________
void Ransac::apply_ransac(){
  std::vector<cv::Keypoint> temp_points;
  long temp_w = 0; //current iteration w value
  cv::mat transform ; // transform created with pair of keypoints
  while w < w_goal {
    keypoints = Ransac::get_n_keypoints();
    transform = Ransac::transform_from_keypoints(keypoints);
    temp_w = Ransac::check_inliers(transform);
    if temp_w > w {
      w = temp_w;
      best_points = keypoints;
      best_transform = transform
    }
  }
}

//_____________________UTILITY FUNCTIONS__________________
std::vector <cv::Keypoint> Ransac::get_n_KeyPoints(){
  /*function to return the keypoints of the first img (left one) in order to create a homo value from it*/
  int i ;
  std::vector<cv::Keypoint> res;//
  std::set<uint64_t> used_val;   // every value has to be different so you create a set to note keypoints that have already been used
  uint64_t temp;
  uint64_t len_match = len(get<2>(data)); //length of vector containing matched values
  for(i = 0 ; i < n ; i ++){
    do{
      temp = rand() % len_match;         // temp in the range 0 to len_match
    }while (used_val.find(temp) != used_val.end() );
    used_val.add(temp);
    res.insert(Ransac[temp]);
  }
  return res;
}

cv::mat Ransac::transform_from_keypoints( std::vector< cv::KeyPoint > kp ){
  /*create homogenous transform matrix from n keypoints*/
  cv::Point2f X , Y ;
  X = kp.at(0);
  Y = kp.at(1);
}



int Ransac::check_inliners(cv::mat trans){
  /*iterate over data Dmatch to check distance and add inlier if
  distance between point after appllying transformation < max_error */
  float distance;
  int cpt_inlier = 0;
  std::vector< std::pair <cv::KeyPoint , cv::KeyPoint> > temp_inliers ; // data points whose reprojection error is under max_eror value
  for (it = get<2>(data).begin(); it < get<2>(data).end() ; it++){
      if it*::distance < max_error{
        temp_inliers.add(Ransac[i]);
        cpt_inlier++;
      }
  }
  if temp_inliers::size > inlier::size {
    inlier = temp_inlier;
  }
  return temp_inlier::size;
}



//______________________OPERATOR OVERLOAD_____________________
std::ostream& operator<< (std::ostream& os){
  //os << ....
  return os;
}

//Allow an easy access of data in form of pairs
std::tuple< <cv::keypoint> , <cv::keypoint> , <cv::DMatch> > operator[](uint64_t index){
  std::tuple< <cv::keypoint> , <cv::keypoint> , <cv::DMatch> > res ;
  std::pair<cv::keypoint , cv::keypoint> res ;

  res<2> = get<2>(data)[index] ;
  res<0> = get<0>(data)[dmatch.queryIdx] ;
  res<1> = get<1>(data)[dmatch.trainIdx] ;

  return res;
}
