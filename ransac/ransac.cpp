#include <iostream>
#include <set>
#include <opencv2/opencv.hpp>
#include "ransac.hpp"

//______________________CONSTRUCTOR & DESTRUCTOR_____________________
Ransac::    Ransac(std::tuple< std::vector<cv::KeyPoint> , std::vector<cv::KeyPoint> , std::vector<cv::DMatch> , bool > datapoint , uint64_t n , float error , float w , cv::Matx33f R, cv::Matx31f T , cv::Matx33f K_g , cv::Matx33f K_d ){
    //std::cout<<"oui"<<std::endl;
    this->data = datapoint; //points matched
    this->n = n; // number of points necessary to build a model
    this->max_error = error;  //max error value for a point to be considered an inlier
    //std::cout<<"oui"<<std::endl;
    this->w_goal = w; //minimum inlier rate wanted
    //std::cout<<"oui"<<std::endl;
    this->nb_samples = std::get<2>(datapoint).size();
    //need to init this->inlier , best_points and best_transform
    //std::cout<<"assignation affine"<<std::endl;
    this->P =cv::Affine3f( (cv::Mat_<float>)R, (cv::Mat_<float>) T);
    //cv::vconcat(T2 , cv::Matx13f({0,0,1}) , this->P);
    this->K_l = K_l ;
    this->K_r = K_r ;
}

Ransac::~Ransac(void){}




//______________________MAIN CLASS FUNCTION _____________________
cv::Affine3<float> Ransac::apply_ransac(){

  uint64_t cpt_attempt = 0 , max_attempt = 500;
  std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch> > temp_point;
  long temp_w = 0 , max_w = 0; //current iteration w value
  cv::Affine3<float> transform ; // transform created with pair of keypoints

  while( this->best_w < this->w_goal && cpt_attempt < max_attempt) {
    temp_point = this->get_n_KeyPoints();
    transform = this->transform_from_KeyPoints(temp_point);
    temp_w = this->check_inliers( transform ) / this-> nb_samples;

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
      temp = rand() % std::get<2>(data).size();  // temp in the range 0 to length of DMatch
    }while (used_val.find(temp) != used_val.end() ); //as long as temp is already in used_val
    used_val.insert(temp);
    res.push_back( this->operator[](temp) );
  }
  return res;
}

cv::Affine3<float> Ransac::transform_from_KeyPoints(std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch> > kp){
  /*create homogenous transform matrix from n keypoints*/
//  cv::Point2f X , Y ;
//  X = kp.at(0);
//  Y = kp.at(1);
  cv::Affine3<float> M;
  return M;
}



int Ransac::check_inliers(cv::Affine3<float> P){
  //Cameras are supposed to be on the same Z plane
  /*iterate over data Dmatch to check distance and add inlier if
  distance between point after appllying transformation < max_error
  Using epipolar constraint*/
  //std::cout<<"debut inlier"<<std::endl;

  cv::Matx31f ptl_caml , ptr_caml , ptr_camr; //first attribute: point id Left /Right , second attribute :  camera reference Left/Right
  cv::Matx31f ptl_to_ptr; // distance from ptl to ptr in cam r ref


  cv::Matx31f T = (cv::Matx31f) P.translation();
  cv::Matx33f R = (cv::Matx33f) P.rotation();
  cv::Mat_<float> temp = (cv::Mat_<float>(3, 3) << 0.0, -T(2), T(1),
                                          T(2), 0.0, -T(1),
                                          -T(1), T(0), 0.0);
  cv::Matx33f S_t(temp);
  cv::Matx33f E = S_t * R;
  cv::Matx33f F = this->K_l.inv().t() * E * this->K_r.inv() ;

  float w ; //inlier rate
  cv::KeyPoint temp_point;
  std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > temp_tuple;
  std::vector< std::tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > temp_inlier ; // data points whose reprojection error is under max_eror value
  //std::cout <<"typeid temp_inlier = " << typeid(temp_inlier).name() <<std::endl;

  for (int i = 0 ; i <  std::get<2>(data).size() ; i++){
    //extracting first and second point
    //std::cout << typeid(this->operator[](i)).name() ) << endl;
    temp_tuple = this->operator[](i);
    temp_point = std::get<0>( temp_tuple );
    cv::Matx31f ptl_iml({temp_point.pt.x , temp_point.pt.y , 1}); //points are in image reference (px)
    temp_point = std::get<1>( temp_tuple );
    cv::Matx31f ptr_imr({temp_point.pt.x , temp_point.pt.y , 1});//points are in image reference (px)

    //apply transform on point1 and point2
    cv::Matx13f l = ptr_imr.t() * F ; // l = epipolar constraint
    float temp_err = (l * ptl_iml)(0);

    //displaying values
    //std::cout <<"temp err  =  " << temp_err << " \t px" <<std::endl;
    //std::cout<<"max_error = " << max_error << "\t px" << std::endl;

    if(temp_err < this->max_error) {
      temp_inlier.push_back(temp_tuple);
      //std::cout<<"added to list"<<std::endl<<std::endl;
    }
  }

  w  = (float) temp_inlier.size() / (float) this->nb_samples ;

  if( w > this->best_w ){
      this->inlier.clear();
      std::copy(temp_inlier.begin(), temp_inlier.end() , std::back_inserter(this->inlier));
      this->best_w = w;
      this->best_transform = P;
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
  os << "Ransac object of param :"<<std::endl<<"\t w_goal = "<<r.w_goal << std::endl << "\tn = " << r.n  ;
  os<< std::endl<<"\tnb_samples = "<<r.nb_samples<<std::endl <<"\t best_w = " << r.best_w << std::endl<<std::endl;

  return os;
}
