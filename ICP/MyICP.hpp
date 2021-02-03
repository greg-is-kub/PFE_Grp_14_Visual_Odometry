#ifndef MyICP_H
#define MyICP_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/icp.hpp>
#include <opencv2/surface_matching/ppf_helpers.hpp>
#include <opencv2/surface_matching/ppf_match_3d.hpp>


class MyICP
{
	public:
		MyICP();
		~MyICP();
	private:
		std::vector<Point3d> points1,points2;
		
};


#endif
