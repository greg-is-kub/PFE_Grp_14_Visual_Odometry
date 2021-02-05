#ifndef ICP_H
#define ICP_H

#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>


using namespace std;
using namespace cv;

class Icp 
{
		
public:
	
	void getTransform(vector<Point3f> &pts1, vector<Point3f> &pts2, Eigen::Matrix3d &R , Eigen::Vector3d &t);

};

#endif
