#ifndef TRIANGULATION
#define TRIANGULATION

#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>

// #include "extra.h" // used in opencv2
using namespace std;
using namespace cv;


struct Pixel
{
    int x;
    int y;
 
    // constructor
    Pixel(int x, int y): x(x), y(y) {}
 
    // overload operator ==
    bool operator==(const Pixel &ob) const
    {
        return (x == ob.x && y == ob.y);
    }
};

namespace std {

  template <>
  struct hash<Pixel>
  {
    std::size_t operator()(const Pixel& k) const
    {
      using std::size_t;
      using std::hash;

      // Compute individual hash values for first and second
      return ((hash<int>()(k.x)
               ^ (hash<int>()(k.y) << 1)) >> 1);
    }
  };

}

class Triangulation
{
public:

    Mat K_d = (Mat_<double>(3, 3) << 1535.50, 0, 739.86, 0, 1535.00, 606.36, 0, 0, 1);
    Mat K_g = (Mat_<double>(3, 3) << 1534.72, 0, 676.97, 0, 1535.00, 698.28, 0, 0, 1);
    Mat R = (Mat_<double>(3, 3) << 1.00, 0.0133051171055671,
                                -0.00534470796279029, -0.0133694262332647, 0.999836410542635,
                                -0.0121823887399877,0.00518174551610382,
                                0.0122525920533588, 0.999911507835259);
    Mat t = (Mat_<double>(1, 3) << -50.28, 0.077, 0.45);
    Mat pts_4d; // points camera without depth
    vector<Point2d> pts_3; // points pixels
    
    unordered_map<Pixel, Point3f> depth_map();
    void triangulation(const vector< tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > & );
    void pixel2cam_vector(const vector< tuple< cv::KeyPoint , cv::KeyPoint , cv::DMatch > > & , vector<Point2f> &, vector<Point2f> &);
    void find_points3d(std::vector<cv::KeyPoint>&, std::vector<cv::KeyPoint>&, std::vector<cv::DMatch>&, std::unordered_map<Pixel, cv::Point3_<float> >&, std::unordered_map<Pixel, cv::Point3_<float> >&, std::vector<cv::Point3_<float> >&, std::vector<cv::Point3_<float> > &);


};

Point2f pixel2cam(const Point2d &p, const Mat &K);

#endif
