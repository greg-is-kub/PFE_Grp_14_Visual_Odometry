#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/sfm/triangulation.hpp>
// #include "extra.h" // used in opencv2
using namespace std;
using namespace cv;

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
    
    void triangulation(std::pair< vector<KeyPoint>, vector<KeyPoint> > &kp, vector<Point3d> &p);

    
};


void Triangulation::triangulation(std::pair<vector<KeyPoint>, vector<KeyPoint> >& keypoints, vector<Point3d> &points)
{
    Mat T1 = (Mat_<float>(3, 4) <<
                            1, 0, 0, 0,
                            0, 1, 0, 0,
                            0, 0, 1, 0);
    Mat T2 = (Mat_<float>(3, 4) <<
                            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    
    vector<Point2f> pts_1, pts_2; // two
    
    for(auto iter:keypoints){
        Mat keypoints_1 = iter->first;
        Mat keypoints_2 = iter->second;
        pts_1.push_back(pixel2cam(keypoints_1.pt, this->K_d));
        pts_2.push_back(pixel2cam(keypoints_2.pt, this->K_g));
        
    }
}


int main(int argc, char **argv) {
    Mat img_1 = imread(argv[1], 1);
    Mat img_2 = imread(argv[2], 1);
    
    Triangulation tri;
    
    tri.triangulation();
}
