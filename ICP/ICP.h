#ifndef ICP_H
#define ICP_H

#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>

using namespace std;
using namespace cv;

class Icp 
{
		
public:
	
	Matx44d getTransform(vector<Point3d>, vector<Point3d>);

};

#endif
