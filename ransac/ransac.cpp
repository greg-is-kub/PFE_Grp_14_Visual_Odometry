#include <iostream>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
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
cv::Affine3<float> Ransac::apply_ransac( int max_attempt ){


  uint64_t cpt_attempt = 0 ;
  std::vector<std::tuple<cv::KeyPoint, cv::KeyPoint, cv::DMatch> > temp_point;
  float temp_w = 0 , max_w = 0 ; //current iteration w value
  float temp_inlier_nb;

  while( this->best_w < this->w_goal && cpt_attempt < max_attempt) {
    //extract n keypoints from the dataset
    temp_point = this->get_n_KeyPoints();
    std::cout<<"oui"<<std::endl;
    std::vector<cv::Point2f> corners1 , corners2 ;
    int i = 0 ;
    for(int i = 0; i < temp_point.size() ; i++){
        corners1.push_back( std::get<0>(temp_point.at(i)).pt );
        corners2.push_back( std::get<1>(temp_point.at(i)).pt );
        //std::cout<<"i = " << i <<std::endl;
        //std::cout<<"corner1 = [" << corners1.at(i).x << " , " << corners1.at(i).y << "]" <<std::endl;
        //std::cout<<"corner2 = [" << corners2.at(i).x << " , " << corners2.at(i).y << "]" <<std::endl;
    }
    //create an homography with it
    cv::Mat_<float> temp ,temp2;
    temp = cv::findHomography( corners1 ,corners2,	temp2 , 0 , 3);
    cv::Affine3f transform(temp) ;
    //apply the homography and check inliers
    //if we get a better score the data will be saved during check_inlier
    temp_inlier_nb = this->check_inliers( transform ) ;
    std::cout << " transform = " << transform.matrix(0,0) <<"\t\t" << transform.matrix(0,1) <<"\t\t"<< transform.matrix(0,2) <<"\t\t"<< transform.matrix(0,3) << std::endl ;
    std::cout << " transform = " << transform.matrix(1,0) <<"\t\t" << transform.matrix(1,1) <<"\t\t"<< transform.matrix(1,2) <<"\t\t"<< transform.matrix(1,3) << std::endl ;
    std::cout << " transform = " << transform.matrix(2,0) <<"\t\t" << transform.matrix(2,1) <<"\t\t"<< transform.matrix(2,2) <<"\t\t"<< transform.matrix(2,3) << std::endl ;
    std::cout << " transform = " << transform.matrix(3,0) <<"\t\t" << transform.matrix(3,1) <<"\t\t"<< transform.matrix(3,2) <<"\t\t"<< transform.matrix(3,3) << std::endl ;
    std::cout<<"temp_inlier_nb = " << temp_inlier_nb <<std::endl;
    temp_w = temp_inlier_nb / this-> nb_samples;

    cpt_attempt ++ ;

    std::cout << "this->best_w  =" << this->best_w << std::endl;
    std::cout << "this->w_goal  =" << this->w_goal << std::endl;
    std::cout << "cpt_attempt  =" << cpt_attempt << std::endl;
    std::cout << "max_attempt  =" << max_attempt << std::endl<<std::endl;


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

    cv::Point2f corners1 , corners2 ;
    corners1 = std::get<0>( this->operator[](temp)).pt;
    corners2 =  std::get<0>(res.at(i)).pt;
    //std::cout<<"corner1 = [" << corners1.x << " , " << corners1.y << "]" <<std::endl;
    }
    //std::cout << "out of get N keypoints"<<std::endl << std::endl;;
  return res;
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
std::tuple< cv::KeyPoint , cv::KeyPoint , cv::lk > Ransac::operator[](uint64_t index){
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
