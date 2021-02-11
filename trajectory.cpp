#include "trajectory.h"

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>
using namespace std;
using namespace Eigen;

Trajectory::Trajectory()
{
    //ctor
}

Trajectory::~Trajectory()
{
    //dtor
}

void Trajectory::show()
{
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    double tx, ty, tz, qx, qy, qz, qw;

    for (int i=0; i<path.size(); i++)
    {
        tx=path[i][0];
        ty=path[i][1];
        tz=path[i][2];
        qw=path[i][6];
        qx=path[i][3];
        qy=path[i][4];
        qz=path[i][5];
        //cout<<tx<<ty<<tz<<qw<<qx<<qy<<qz<<endl;
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*
      // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    
    
    
    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glLineWidth(2);
        for (size_t i = 0; i < poses.size(); i++)
            {
                
                Vector3d Ow = poses[i].translation();
                Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
                Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
                Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
                
                glBegin(GL_LINES);
                glColor3f(1.0, 0.0, 0.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Xw[0], Xw[1], Xw[2]);
                glColor3f(0.0, 1.0, 0.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Yw[0], Yw[1], Yw[2]);
                glColor3f(0.0, 0.0, 1.0);
                glVertex3d(Ow[0], Ow[1], Ow[2]);
                glVertex3d(Zw[0], Zw[1], Zw[2]);
                glEnd();
                

            }

        for (size_t i = 0; i < poses.size(); i++)
        {
            glColor3f(0.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
*/

  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    const float w=0.1;
    const float h=w*0.75;
    const float z=w*0.6;
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glLineWidth(4);

            
    for (size_t i = 0; i < poses.size() - 1; i++) {
    glPushMatrix();
      glColor3f(0.0f, 1.0f, 0.0f);  // blue for ground truth
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
/*        
        glColor3f(0.0f,0.0f,1.0f);
        float p1x = p1.translation()[0];
        float p1y = p1.translation()[1];
        float p1z = p1.translation()[2];
        float p2x = p2.translation()[0];
        float p2y = p2.translation()[1];
        float p2z = p2.translation()[2];
        glVertex3f(p1x, p1y, p1z);		glVertex3f(p1x+w,p1y+h,p1z+z);
        glVertex3f(p1x, p1y, p1z);		glVertex3f(p1x+w,p1y-h,p1z+z);
        glVertex3f(p1x, p1y, p1z);      glVertex3f(p1x-w,p1y-h,p1z+z);
        glVertex3f(p1x, p1y, p1z);      glVertex3f(p1x-w,p1y+h,p1z+z);
        glVertex3f(p2x+w,p2y+h,p2z+z);		glVertex3f(p2x+w,p2y-h,p2z+z);
        glVertex3f(p2x-w,p2y+h,p2z+z);		glVertex3f(p2x-w,p2y-h,p2z+z);
        glVertex3f(p2x-w,p2y+h,p2z+z);		glVertex3f(p2x+w,p2y+h,p2z+z);
        glVertex3f(p2x-w,p2y-h,p2z+z);		glVertex3f(p2x+w,p2y-h,p2z+z);

        glEnd();
        glPopMatrix();*/
    }

    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }





}

void Trajectory::replace_path(vector<vector<double>>new_path)
{
    Trajectory::path=new_path;
    vector<double> last= new_path.back();

    Quaterniond q_;
    q_.x()=last[3];
    q_.y()=last[4];
    q_.z()=last[5];
    q_.w()=last[6];


    Isometry3d Twr(q_);
    Twr.pretranslate(Vector3d(last[0], last[1], last[2]));

    Trajectory::last=Twr;

//    Trajectory::tx_M=last[0];
//    Trajectory::ty_M=last[1];
//    Trajectory::tz_M=last[2];
//
//    Trajectory::qx_M=last[3];
//    Trajectory::qy_M=last[4];
//    Trajectory::qz_M=last[5];
//
//    Trajectory::qw_M=last[6];
}

void Trajectory::add_to_path(Eigen::Matrix3d R_, Eigen::Vector3d t_)
{
    //Récupère la nouvelle transformation et calcule la matrice de transformation
    Isometry3d In_ =Isometry3d::Identity();
    In_.rotate(R_);
    In_.pretranslate(t_);
    In_ = In_.inverse();
    

    //Obtiens la nouvelle matrice de transformation projeté dans le repère monde
    Isometry3d New_ = Trajectory::last * In_.inverse();
    cout << In_.matrix() <<endl;
    cout << New_.matrix() <<endl;




    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Eigen::Matrix3d R_n=New_.rotate();
    //Eigen::Vector3d t_n=New_.pretranslate();

//    Eigen::Matrix3d R_n=
//    Eigen::Vector3d t_n = (New_.translation()(0), New_.translation()(1), New_.translation()(2));

    Eigen::Matrix3d R_n=New_.rotation();
    Eigen::Vector3d t_n=New_.translation();
    
    cout << R_n << endl;



    //Transform les entrées en Quaternion et récupère les translations
    Quaterniond q(R_n);

    double ltx, lty, ltz, lqx, lqy, lqz, lqw;
    ltx=t_n(0,0);
    lty=t_n(1,0);
    ltz=t_n(2,0);
    lqx=q.x();
    lqy=q.y();
    lqz=q.z();
    lqw=q.w();


//    Addition avec _M pour obtenir les coordonnés monde
//    double ntx, nty, ntz, nqx, nqy, nqz, nqw;
//    ntx=ltx+Trajectory::tx_M;
//    nty=lty+Trajectory::ty_M;
//    ntz=ltz+Trajectory::tz_M;
//    nqw=lqw*Trajectory::qw_M;
//    nqx=lqx*Trajectory::qx_M;
//    nqy=lqy*Trajectory::qy_M;
//    nqz=lqz*Trajectory::qz_M;

    vector<double> to_add {ltx, lty, ltz, lqw, lqx, lqy, lqz};
    Trajectory::path.push_back(to_add);


    Trajectory::last = New_;


//    Trajectory::tx_M=ntx;
//    Trajectory::ty_M=nty;
//    Trajectory::tz_M=ntz;
//    Trajectory::qw_M=nqw;
//    Trajectory::qx_M=nqx;
//    Trajectory::qy_M=nqy;
//    Trajectory::qz_M=nqz;
}
