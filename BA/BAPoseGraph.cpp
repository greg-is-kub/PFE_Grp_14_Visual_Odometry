#include "BAPoseGraph.h"

void BAPoseGraph::optimizePoses(vector<vector<double>> poses_absolute, vector<vector<double>> poses_relative) {

    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出

    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
//    while (!fin.eof()) {
//        string name;
//        fin >> name;
//        if (name == "VERTEX_SE3:QUAT") {
//            // SE3 顶点
//            g2o::VertexSE3 *v = new g2o::VertexSE3();
//            int index = 0;
//            fin >> index;
//            v->setId(index);
//            v->read(fin);
//            optimizer.addVertex(v);
//            vertexCnt++;
//            if (index == 0)
//                v->setFixed(true);
//        } else if (name == "EDGE_SE3:QUAT") {
//            // SE3-SE3 边
//            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
//            int idx1, idx2;     // 关联的两个顶点
//            fin >> idx1 >> idx2;
//            e->setId(edgeCnt++);
//            e->setVertex(0, optimizer.vertices()[idx1]);
//            e->setVertex(1, optimizer.vertices()[idx2]);
//            e->read(fin);
//            optimizer.addEdge(e);
//        }
//        if (!fin.good()) break;
//    }

    //initialization of parameters
    double tx;
    double ty;
    double tz;
    double q0;
    double q1;
    double q2;
    double q3;
    //double idx1, idx2;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    for (int i=0; i<poses_absolute.size(); i++){
        tx = poses_absolute[i][0];
        ty = poses_absolute[i][1];
        tz = poses_absolute[i][2];
        q1 = poses_absolute[i][3];
        q2 = poses_absolute[i][4];
        q3 = poses_absolute[i][5];
        q0 = poses_absolute[i][6];

        //write in a txt file
        ofstream OutFile("Test.txt");
        OutFile <<tx<<ty<<tz<<q1<<q2<<q3<<q0;
        OutFile.close();
        //open txt file
        ifstream fin("Test.txt");

        T.rotate(Eigen::Quaternion<double>(q1, q2, q3, q0));
        T.pretranslate(Eigen::Vector3d(tx, ty, tz));

        g2o::VertexSE3 *v = new g2o::VertexSE3();
        int index = 0;
        index = i;
        v->setId(index);
        v->read(fin);
        //v->read(fin);
        optimizer.addVertex(v);
        vertexCnt++;
        if (index == 0)
            v->setFixed(true);

        //end of reading txt file
        fin.close();

        //clear txt file
        string dbfile="Test.txt";
        ofstream fileout(dbfile.c_str(), ios::out|ios::trunc );
        fileout.close();
    }

    for (int i=0; i<poses_relative.size()-1; i++){

        tx = poses_relative[i][0];
        ty = poses_relative[i][1];
        tz = poses_relative[i][2];
        q1 = poses_relative[i][3];
        q2 = poses_relative[i][4];
        q3 = poses_relative[i][5];
        q0 = poses_relative[i][6];

        ofstream OutFile("Test.txt");
        OutFile <<tx<<ty<<tz<<q1<<q2<<q3<<q0;
        OutFile.close();

        ifstream fin("Test.txt");


        T.rotate(Eigen::Quaternion<double>(q1, q2, q3, q0));
        T.pretranslate(Eigen::Vector3d(tx, ty, tz));

        g2o::EdgeSE3 *e = new g2o::EdgeSE3();
        int idx1, idx2;     // 2 connected vertex 关联的两个顶点
        idx1 = i; idx2 = i+1;
        e->setId(edgeCnt++);
        e->setVertex(0, optimizer.vertices()[idx1]);
        e->setVertex(1, optimizer.vertices()[idx2]);
        //set the information matrix
        Eigen::Matrix<double, 6, 6> InfoMatrix = Eigen::Matrix<double, 6, 6>::Identity();
        InfoMatrix(0,0) = InfoMatrix(1,1) = InfoMatrix(2,2) = 10000;
        InfoMatrix(3,3) = InfoMatrix(4,4) = InfoMatrix(5,5) = 40000;
        e->setInformation(InfoMatrix);
        e->read(fin);
        //e->read(fin);
        optimizer.addEdge(e);

        fin.close();
        string dbfile="Test.txt";
        ofstream fileout(dbfile.c_str(), ios::out|ios::trunc );
        fileout.close();
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save("result.g2o");
}

int main(int argc, char **argv){

    //read file
    if (argc != 2) {
        cout << "Usage: BAPoseGraph sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    //initialize pose vectors
    vector<vector<double>> poses_abs; vector<vector<double>> poses_rel;


    //read file and load to poses
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {

            int index = 0;
            double tx, ty, tz, q0, q1, q2, q3;

            fin >> index;
            fin >> tx;
            fin >> ty;
            fin >> tz;
            fin >> q1;
            fin >> q2;
            fin >> q3;
            fin >> q0;

            vector<double> pa = {tx, ty, tz, q1, q2, q3, q0};
            poses_abs.push_back(pa);


        } else if (name == "EDGE_SE3:QUAT") {

            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            double tx, ty, tz, q0, q1, q2, q3;

            fin >> tx;
            fin >> ty;
            fin >> tz;
            fin >> q1;
            fin >> q2;
            fin >> q3;
            fin >> q0;

            vector<double> pr = {tx, ty, tz, q1, q2, q3, q0};
            poses_rel.push_back(pr);
        }
        if (!fin.good()) break;
    }

    BAPoseGraph myBA;
    myBA.optimizePoses(poses_abs, poses_rel);


    return 0;
}