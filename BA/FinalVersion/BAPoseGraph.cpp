#include "BAPoseGraph.h"


void BAPoseGraph::Rt_to_Quaternion_and_stack_all(Eigen::Matrix3d R, Eigen::Vector3d t) {
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    vector<double> tq = {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
    buffer_poses_relative.push_back(tq);
}

void BAPoseGraph::outputOptimizedPoses(string optimized_filepath) {
    ifstream fin(optimized_filepath);
    if (!fin) {
        cout << "file " << optimized_filepath << " does not exist." << endl;
    }

    double tx, ty, tz, qx, qy, qz, qw;

    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            int index = 0;
            fin >> index;
            fin >> tx;
            fin >> ty;
            fin >> tz;
            fin >> qx;
            fin >> qy;
            fin >> qz;
            fin >> qw;
        }
    }

    vector<double> tq = {tx, ty, tz, qx, qy, qz, qw};
    buffer_poses_absolute_optimized.push_back(tq);

}

void BAPoseGraph::read_and_merge_poses(vector<vector<double>> poses_absolute, vector<vector<double>> poses_relative,
                                       string all_poses_filepath) {
    double tx, ty, tz, qx, qy, qz, qw;
    int index, idx1, idx2;
    ofstream fout(all_poses_filepath);

    //read and write absolute poses
    for (int i; i<poses_absolute.size(); i++){
        index = i;
        tx = poses_absolute[i][0];
        ty = poses_absolute[i][1];
        tz = poses_absolute[i][2];
        qx = poses_absolute[i][3];
        qy = poses_absolute[i][4];
        qz = poses_absolute[i][5];
        qw = poses_absolute[i][6];

        //write in a txt file
        fout <<"VERTEX_SE3:QUAT"<<" "<<index<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<endl;

    }

    //read and write relative poses
    for (int i; i<poses_absolute.size()-1; i++){
        idx1 = i;
        idx2 = i+1;
        tx = poses_relative[i][0];
        ty = poses_relative[i][1];
        tz = poses_relative[i][2];
        qx = poses_relative[i][3];
        qy = poses_relative[i][4];
        qz = poses_relative[i][5];
        qw = poses_relative[i][6];

        //write in a txt file
        fout <<"EDGE_SE3:QUAT"<<" "<<idx1<<" "<<idx2<<" "<<tx<<" "<<ty<<" "<<tz<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<" "<<10000<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<10000<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<10000<<" "<<0<<" "<<0<<" "<<0<<" "<<40000<<" "<<0<<" "<<0<<" "<<40000<<" "<<0<<" "<<40000<<endl;

    }
    fout.close();
}



// Define optimization function
void BAPoseGraph::optimizePoses(string filepath, string optimized_filepath) {

    ifstream fin(filepath);
    if (!fin) {
        cout << "file " << filepath << " does not exist." << endl;
    }

    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // SE3 顶点
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 边
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
                        //set the information matrix
            //Eigen::Matrix<double, 6, 6> InfoMatrix = Eigen::Matrix<double, 6, 6>::Identity();
            //InfoMatrix(0,0) = InfoMatrix(1,1) = InfoMatrix(2,2) = 10000;
            //InfoMatrix(3,3) = InfoMatrix(4,4) = InfoMatrix(5,5) = 40000;

            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save("OptimizedPoses.txt");

}


int main() {
//    if (argc != 2) {
//        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
//        return 1;
//    }
//    ifstream fin(argv[1]);
//    if (!fin) {
//        cout << "file " << argv[1] << " does not exist." << endl;
//        return 1;
//    }
//    return 0;

    BAPoseGraph myBA;

//    myBA.optimizePoses("/home/yuancheng/桌面/BA/TestWithSphere/sphere.g2o");

    // test read and merge poses
//    vector<vector<double>> pose_abs, pose_rel;
//
//    pose_abs.push_back({-0.125664, -1.53894e-17, 99.9999, 0.706662, 4.32706e-17, 0.707551, -4.3325e-17});
//    pose_abs.push_back({-0.250786, -0.0328449, 99.981, 0.705413, 0.0432253, 0.705946 ,-0.0465295});
//    pose_abs.push_back({-0.384479, -0.102155, 99.9722, 0.701473, 0.0869233, 0.701311, -0.0924285});
//    pose_abs.push_back({-0.488061, -0.195958, 99.9833, 0.689802, 0.131752, 0.698923, -0.135355});
//    pose_abs.push_back({-0.577441, -0.319656, 99.9749, 0.683889, 0.173189, 0.684097, -0.185235 });
//    pose_abs.push_back({-0.626233, -0.455465, 99.9639, 0.675686, 0.20944, 0.672757, -0.216751});
//
//    pose_rel.push_back({-0.0187953, 0.0328449, -0.125146, 0.0634648, -0.000250128, 0.00237634, 0.997981});
//    pose_rel.push_back({-0.00836587, 0.0518559, -0.141405, 0.0636098, -0.000538, 0.00162238, 0.997973});
//    pose_rel.push_back({0.0116518, 0.0646362, -0.123843, 0.0628369, 0.0060981, -0.00213561, 0.998003});
//    pose_rel.push_back({-0.00635662, 0.0817346, -0.128989, 0.0661221, -0.0051306, 0.00750755, 0.99777});
//    pose_rel.push_back({-0.00900338, 0.0946003, -0.109164, 0.0498363, -0.00340117, -0.00255259, 0.998748});
//
//    string all_poses_filepath = "./PosesAbsRel.txt";
//    string save_optimized_poses = "./OptimizedPoses.txt";
//    myBA.read_and_merge_poses(pose_abs, pose_rel, all_poses_filepath);
//    myBA.optimizePoses(all_poses_filepath, save_optimized_poses);

    // test quaternion
    Eigen::Matrix3d R;
    R<<1,0,0,0,1,0,0,0,1;

    Eigen::Vector3d t(1,2,3);

    cout<<R<<endl;
    Eigen::Quaterniond q(R);
    cout<<q.x()<<endl;
    cout<<q.y()<<endl;
    cout<<q.z()<<endl;
    cout<<q.w()<<endl;

    Eigen::Isometry3d T;
    T.rotate(R);
    T.translate(t);

    Eigen::Quaterniond q_ew(T.rotation());
    cout<<"q_new: "<<endl;
    cout<<q_new.x()<<endl;
    cout<<q_new.y()<<endl;
    cout<<q_new.z()<<endl;
    cout<<q_new.w()<<endl;




}