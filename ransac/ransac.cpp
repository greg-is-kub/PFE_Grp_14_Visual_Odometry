#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/sfm/projection.hpp>
#include "ransac.hpp"

//______________________CONSTRUCTOR & DESTRUCTOR_____________________
Ransac::Ransac(std::tuple< std::vector<cv::KeyPoint> , std::vector<cv::KeyPoint> , std::vector<cv::DMatch> > datapoint , uint64_t nb_point , float error , float w , Frame*  ptr_f){
    this->data = datapoint; //points matched
    this->n = nb_point; // number of points necessary to build a model
    this->max_error = error;  //max error value for a point to be considered an inlier
    this->w_goal = w; //minimum inlier rate wanted
    this->ptr_frame = ptr_f; //pointer on the frame containing the img Frame is a data container
    this->nb_samples = std::get<2>(datapoint).size();
    //need to init this->inlier , best_points and best_transform
}

Ransac::~Ransac(void){}

//______________________MAIN CLASS FUNCTION _____________________
cv::Mat Ransac::apply_ransac(){

  uint64_t cpt_attempt = 0 , max_attempt = 500;
  std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch> > temp_point;
  long temp_w = 0 , max_w = 0; //current iteration w value
  cv::Mat transform ; // transform created with pair of keypoints

  while( this->best_w < this->w_goal && cpt_attempt < max_attempt) {
    temp_point = this->get_n_KeyPoints();
    transform = this->transform_from_KeyPoints(temp_point);
    temp_w = this->check_inliers(transform) / this-> nb_samples;

    if( temp_w > max_w ) {
      this->best_w = temp_w;
      this->best_points = temp_point;
      this->best_transform = transform;
    }
  }
  if(cpt_attempt >= max_attempt){
    std::cout<< "max_attempt reached and w_goal could not be reached, best_w = "<< this->best_w << std::endl << "best_points and best_transform are memorized"<<std::endl;
  }
  return this->best_transform;
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
      temp = rand() % std::get<2>(data).size() ;  // temp in the range 0 to length of DMatch
    }while (used_val.find(temp) != used_val.end() ); //as long as temp is already in used_val
    used_val.insert(temp);
    res.push_back(this->operator[](temp));
  }
  return res;
}

cv::Mat Ransac::transform_from_KeyPoints( std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch> > kp ){
  /*create homogenous transform matrix from n keypoints*/
  std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch>  temp_tuple ;
  cv::Mat X,Y;
  for(int i = 0 ; i < kp.size() ; i ++){
      temp_tuple = kp.at(i);
      //convert Point2f value of keypoints contained in tuple into a homogenous vector
      //cv::sfm::euclideanToHomogeneous( std::get<1>(temp_tuple).pt , Y)
      //cv::sfm::euclideanToHomogeneous( std::get<0>(temp_tuple).pt , X)


  }
}



int Ransac::check_inliers(cv::Mat trans){
  /*iterate over data Dmatch to check distance and add inlier if
  distance between point after appllying transformation < max_error */
  uint64_t i = 0;
  float distance;
  int cpt_inlier = 0;
  std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > temp_inlier ; // data points whose reprojection error is under max_eror value
  for (uint64_t i = 0 ; i < (std::get<2>(data)).size(); i++){
      //if(this-> data.distance < max_error){
      //  temp_inlier.push_back(this->operator[](i));
      //  cpt_inlier++;
      //}
      //i++;
  }
  if(temp_inlier.size() > this->inlier.size()) {
    this->inlier = temp_inlier;
  }
  return temp_inlier.size();
}



//______________________OPERATOR OVERLOAD_____________________
std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > Ransac::operator[](uint64_t index){
  //Allow an easy access of data in form of pairs
  //maybe problem de pointeurs à cet endroit à verifier
  //accesseur quand meme pratique on va pas se mentir permet de retourner les matched points et leur dmatch
  std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > res ;
  cv::DMatch match = std::get<2>(data)[index] ;
  std::get<2>(res) = match ;
  cv::KeyPoint temp_point = std::get<0>(data)[match.queryIdx] ;
  std::get<0>(res) = temp_point;
  temp_point = std::get<1>(data)[match.trainIdx] ;
  std::get<1>(res) = temp_point ;

  return res;
}


std::ostream& operator<< (std::ostream& os , Ransac r){
  //os << ....
  return os;
}
