#include <set>
#include <opencv2/core/core.hpp>
#include "ransac.hpp"

//______________________CONSTRUCTOR & DESTRUCTOR_____________________
Ransac::Ransac(std::tuple< std::vector<cv::KeyPoint> , std::vector<cv::KeyPoint> , std::vector<cv::DMatch> > datapoint , uint64_t nb_point , float error , float w , Frame*  ptr_f){
    this->data = datapoint; //points matched
    this->n = nb_point; // number of points necessary to build a model
    this->max_error = error;  //max error value for a point to be considered an inlier
    this->w_goal = w; //minimum inlier rate wanted
    this->ptr_frame = ptr_f; //pointer on the frame where we will stack data
    this->nb_samples = this->get<2>(data)::size;
}

Ransac::~Ransac(void){}

//______________________MAIN CLASS FUNCTION _____________________
void Ransac::apply_ransac(){
  std::vector<cv::KeyPoint> temp_points;
  long temp_w = 0; //current iteration w value
  cv::Mat transform ; // transform created with pair of keypoints
  while( temp_w < w_goal) {
    temp_points = this->get_n_keypoints();
    transform = this->transform_from_keypoints(keypoints);
    temp_w = this->check_inliers(transform);
    if() temp_w > w ) {
      w = temp_w;
      best_points = keypoints;
      best_transform = transform
    }
  }
}

//_____________________UTILITY FUNCTIONS__________________
std::vector <std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > Ransac::get_n_KeyPoints(){
  /*function to return the keypoints of the first img (left one) in order to create a homo value from it*/
  int i ;
  std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > res;//result
  std::set<uint64_t> used_val;   // every value has to be different so you create a set to note keypoints that have already been used
  uint64_t temp;

  for(i = 0 ; i < n ; i ++){
    do{
      temp = rand() % get<2>(data)::size;  // temp in the range 0 to length of DMatch
    }while (used_val.find(temp) != used_val.end() ); //as long as temp is already in used_val
    used_val.add(temp);
    res.insert(Ransac[temp]);
  }
  return res;
}

cv::Mat Ransac::transform_from_keypoints( std::vector< cv::KeyPoint > kp ){
  /*create homogenous transform matrix from n keypoints*/
  cv::Point2f X , Y ;
  X = kp.at(0);
  Y = kp.at(1);
}



int Ransac::check_inliners(cv::Mat trans){
  /*iterate over data Dmatch to check distance and add inlier if
  distance between point after appllying transformation < max_error */
  uint64_t i = 0;
  float distance;
  int cpt_inlier = 0;
  std::vector< std::pair <cv::KeyPoint , cv::KeyPoint> > temp_inlier ; // data points whose reprojection error is under max_eror value
  for (it = get<2>(data).begin(); it < get<2>(data).end() ; it++){
      if(it*::distance < max_error){
        temp_inlier.add(Ransac[i]);
        cpt_inlier++;
      }
      i++;
  }
  if(temp_inlier::size > inlier::size) {
    inlier = temp_inlier;
  }
  return temp_inlier::size;
}



//______________________OPERATOR OVERLOAD_____________________
//Allow an easy access of data in form of pairs
std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > Ransac::operator[](uint64_t index){
  //maybe problem de pointeurs à cet endroit à verifier
  //accesseur quand meme pratique on va pas se mentir permet de retourner les matched points et leur dmatch
  std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > res ;
  cv::DMatch match = std::get<2>(data)[index] ;
  std::get<2>(res) = match ;
  cv::KeyPoint temp_point = std::get<0>(data)[dmatch.queryIdx] ;
  std::get<0>(res) = temp_point;
  temp_point = std::get<1>(data)[dmatch.trainIdx] ;
  std::get<1>(res) = temp_point ;

  return res;
}


std::ostream& operator<< (std::ostream& os , Ransac r){
  //os << ....
  return os;
}
