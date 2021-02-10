#ifndef POSE_GRAPH_POSE_GRAPH_H
#define POSE_GRAPH_POSE_GRAPH_H

#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

class pose_graph{
public:
    void pose_graph_g2o_SE3(string filepath);
};

#endif //POSE_GRAPH_POSE_GRAPH_H
