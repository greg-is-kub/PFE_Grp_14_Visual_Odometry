#ifndef RANSAC
#define RANSAC

#include <iostream>
#include <string>
#include <map>
//#include features descriptor from nicolas ORB
//#include
template<typename datapoint> class Ransac{
  //ransac is going to be applied on 2 different processes (ICP and feature matching), therefore we won't
  //class fctn will therefore be defined with template functions
  public :
    //ordered map of paired points  points can be either 3D or 2D points with a feature map
    Ransac(std::map<uint64_t , std::pair< datapoint* , datapoint* > > , uint64_t n);
    ~Ransac(void);

    //whole process of applying ransac data
    void apply_ransac();

    //application of the transformation in order to get the inlier
    void get_inliners(datapoint* map);

    //extract n random datapoints from the data to process the data
    datapoint* get_n_datapoints();

    //to string
    std::ostream& operator<< (std::ostream& os) ;
    /*{
        os << dt.mo << '/' << dt.da << '/' << dt.yr;
        return os;
    }*/


  protected :
    float w ; // w = inliers / nb_samples
    uint64_t n ; // nb samples necessary ton create a model
    float max_error ; // maximum accepted error for a point to be defined as an inlier
    std::map<uint64_t , std::pair<float* , float* > >  inliers ; // data points whose reprojection error is under max_eror value
    std::map<uint64_t , std::pair<float* , float* > >  outliers ; // data points whose reprojection error is under max_eror value
    const std::map<uint64_t , std::pair<float* , float* > > data ; //every data points given to ransac for analysis

  private :





};

#endif
