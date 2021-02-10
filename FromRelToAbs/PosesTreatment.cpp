#include "PosesTreatment.h"

void PosesTreatment::RelToAbs_with_push_back(Eigen::Matrix3d R, Eigen::Vector3d t) {
    Eigen::Matrix4d Transformation; // define the newest transformatin matrix (between frames)
    Transformation.setIdentity(); // set to 1
    Transformation.block<3,3>(0,0) = R;
    Transformation.block<3,1>(0,3) = t;

    // if it's the first transformation, then save as absolute transformation directly
    if (buffer_poses_absolute_Quaternion.empty()){
        Eigen::Quaterniond q(R);
        buffer_poses_absolute_Quaternion.push_back({t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()});
        buffer_poses_absolute_Rt.push_back(Transformation);
    }
    // if not, compute the absolute transformation using the last saved transformation
    else {
        Eigen::Matrix4d Transformation_last;
        Transformation_last = buffer_poses_absolute_Rt.back(); // load the last saved absolute transformation
        Eigen::Matrix4d Transformation_abs;
        Transformation_abs = Transformation * Transformation_last; // multiplication to obtain the newest
        Eigen::Matrix3d R_abs = Transformation_abs.block<3,3>(0,0); // get R_abs
        Eigen::Vector3d t_abs = Transformation_abs.block<3,1>(0,3); // get t_abs
        Eigen::Quaterniond q_abs(R_abs); //get q_abs from R_abs
        buffer_poses_absolute_Quaternion.push_back({t_abs.x(), t_abs.y(), t_abs.z(), q_abs.x(), q_abs.y(), q_abs.z(), q_abs.w()}); // save q_abs and t_abs
    }
}

void PosesTreatment::OutputTxt_with_buffer_abs(vector<vector<double>> buffer_abs_qt) {
    ofstream fout("AllPosesQuaternionAbsolute.txt");
    for (int i=0; i<buffer_abs_qt.size(); i++){
        double tx = buffer_abs_qt[i][0];
        double ty = buffer_abs_qt[i][1];
        double tz = buffer_abs_qt[i][2];
        double qx = buffer_abs_qt[i][3];
        double qy = buffer_abs_qt[i][4];
        double qz = buffer_abs_qt[i][5];
        double qw = buffer_abs_qt[i][6];

        int index = i;

        fout <<index<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;
    }
    fout.close();
}
