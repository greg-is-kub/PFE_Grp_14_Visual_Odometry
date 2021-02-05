#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <Eigen/Core>
using namespace std;

class Trajectory
{
    public:
        Trajectory();
        ~Trajectory();
        double tx_M, ty_M, tz_M, qx_M, qy_M, qz_M, qw_M;
        vector<vector<double>> path;

        void show();

        void replace_path(vector<vector<double>>);

        void add_to_path(Eigen::Matrix3d, Eigen::Vector3d);

    protected:

    private:
};

#endif // TRAJECTORY_H
