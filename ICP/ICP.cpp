#include "ICP.h"

using namespace std;
using namespace cv;


Matx44d Icp::getTransform(vector<Point3d> points1, vector<Point3d> points2)
{
	cv::ppf_match_3d::ICP iterativeClosestPoint;
  	double error;
  	Matx44d transformation;
  	
  	//Turn vector<Point3d> to Mat
  	Mat points11 = Mat(points1, CV_32F).reshape(1);
  	Mat points22 = Mat(points2, CV_32F).reshape(1);
  	
  	//Resize to smallest shape
  	int mini = min(points11.rows, points22.rows);
  	points11=points11(Range(0,mini), Range::all());
  	points22=points22(Range(0,mini), Range::all());
  	
  	//Compute the normals
  	Mat p1, p2;
  	Vec3f f = Vec3f(0,0,0);
  	cv::ppf_match_3d::computeNormalsPC3d(points11, p1, 10, 0, f);
  	cv::ppf_match_3d::computeNormalsPC3d(points22, p2, 10, 0, f);
  	
	
	iterativeClosestPoint.registerModelToScene(p1,p2, error, transformation);
	
	return transformation;
  	
  	
}
