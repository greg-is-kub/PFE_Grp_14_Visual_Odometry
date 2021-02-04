#include "ICP.h"

using namespace std;
using namespace cv;


void Icp::getTransform(vector<Point3f> &pts1, vector<Point3f> &pts2, Eigen::Matrix3d &R , Eigen::Vector3d &t)
{
	// Compute mass center point of pts1 and pts2
	Point3f p1, p2;     // cente
  	int N = pts1.size();
  	for (int i = 0; i < N; i++) {
    		p1 += pts1[i];
 		p2 += pts2[i];
	}
  	p1 = Point3f(Vec3f(p1) / N);
  	p2 = Point3f(Vec3f(p2) / N);
	
	// Substract mass center from pts1 and pts2 => q1, q2
	vector<Point3f> q1(N), q2(N); 
  	for (int i = 0; i < N; i++) {
    		q1[i] = pts1[i] - p1;
    		q2[i] = pts2[i] - p2;
  	}
  	
  	// compute q1*q2^T => W
  	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  	for (int i = 0; i < N; i++) {
    		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
  	}
  	
  	// Singular Value Deco;position of W => U, V
  	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  	Eigen::Matrix3d U = svd.matrixU();
  	Eigen::Matrix3d V = svd.matrixV();
  	
  	//Compute Transformation
  	R = U * (V.transpose());
  	if (R.determinant() < 0)  R = -R;
  	t = Eigen::Vector3d(p1.x, p1.y, p1.z) - R * Eigen::Vector3d(p2.x, p2.y, p2.z);
  	
  	
 
}
