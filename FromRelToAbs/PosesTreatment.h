#ifndef FROMRELTOABS_POSESTREATMENT_H
#define FROMRELTOABS_POSESTREATMENT_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <Eigen/Geometry>

//#include <g2o/types/slam3d/types_slam3d.h>
//#include <g2o/core/block_solver.h>
//#include <g2o/core/optimization_algorithm_levenberg.h>
//#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

class PosesTreatment {
public:
    vector<Eigen::Matrix4d> buffer_poses_absolute_Rt; // save all absolute Transformation matrix (4 X 4)
    vector<vector<double>> buffer_poses_absolute_Quaternion; // save all absolute transformation in Quaternion form (t ... q...)
    void RelToAbs_with_push_back(Eigen::Matrix3d R, Eigen::Vector3d t); // one R,T ----stack to----> all t,q absolute
    void OutputTxt_with_buffer_abs(vector<vector<double>> buffer_abs_qt); // save as txt with order (index 1 2 3 4...)
};

#endif //FROMRELTOABS_POSESTREATMENT_H
