#ifndef BAPOSEGRAPH_BAPOSEGRAPH_H
#define BAPOSEGRAPH_BAPOSEGRAPH_H

#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

class BAPoseGraph{
public:
    void optimizePoses(vector<vector<double>> poses_absolute, vector<vector<double>> poses_relative);
};

#endif //BAPOSEGRAPH_BAPOSEGRAPH_H
