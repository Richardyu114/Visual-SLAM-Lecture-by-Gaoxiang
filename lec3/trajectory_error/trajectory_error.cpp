#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
 
using namespace std;
using namespace Eigen;
 
void ReadData(string FileName ,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses);
double ErrorTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e);
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e);
 
int main(int argc, char **argv)
{
    string GroundFile = "./groundtruth.txt";
    string ErrorFile = "./estimated.txt";
    double trajectory_error_RMSE = 0;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e;
 
    ReadData(GroundFile,poses_g);
    ReadData(ErrorFile,poses_e);
    trajectory_error_RMSE = ErrorTrajectory(poses_g, poses_e);
    cout<<"trajectory_error_RMSE = "<< trajectory_error_RMSE<<endl;
    DrawTrajectory(poses_g,poses_e);
 
}
 
 
/***************************读取文件的数据，并存储到vector类型的pose中**************************************/
void ReadData(string FileName ,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> &poses)
{
    ifstream fin(FileName);  //从文件中读取数据
   //这句话一定要加上，保证能够正确读取文件。如果没有正确读取，结果显示-nan
    if(!fin.is_open()){
        cout<<"No "<<FileName<<endl;
        return;
    }
    double t,tx,ty,tz,qx,qy,qz,qw;
    string line;
    while(getline(fin,line)) 
    {
        istringstream record(line);    //从string读取数据
        record >> t >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Vector3d p(tx, ty, tz);
        Eigen::Quaterniond q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();  //四元数的顺序要注意
        Sophus::SE3 SE3_qp(q, p);
        poses.push_back(SE3_qp);
    }
 
}
 
/*******************************计算轨迹误差*********************************************/
double ErrorTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e )
{
    double RMSE = 0;
    Matrix<double ,6,1> se3;
    vector<double> error;
    for(int i=0;i<poses_g.size();i++){
        se3=(poses_g[i].inverse()*poses_e[i]).log();  //这里的se3为向量形式，求log之后是向量形式
        //cout<<se3.transpose()<<endl;
        error.push_back( se3.squaredNorm() );  //二范数的平方
       // cout<<error[i]<<endl;
    }
 
    for(int i=0; i<poses_g.size();i++){
        RMSE += error[i];
    }
    RMSE /= double(error.size());
    RMSE = sqrt(RMSE);
    return RMSE;
}
/*****************************绘制轨迹*******************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_g,
        vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_e) {
    if (poses_g.empty() || poses_e.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }
 
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);  //创建一个窗口
    glEnable(GL_DEPTH_TEST);   //启动深度测试
    glEnable(GL_BLEND);       //启动混合
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//混合函数glBlendFunc( GLenum sfactor , GLenum dfactor );sfactor 源混合因子dfactor 目标混合因子
 
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0) //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
    );
 
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
 
 
    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//绘制窗口设定
 
        glLineWidth(2);
        for (size_t i = 0; i < poses_g.size() - 1; i++) {
            glColor3f(1 - (float) i / poses_g.size(), 0.0f, (float) i / poses_g.size());
            glBegin(GL_LINES);
            auto p1 = poses_g[i], p2 = poses_g[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
 
        for (size_t j = 0; j < poses_e.size() - 1; j++) {
            //glColor3f(1 - (float) j / poses_e.size(), 0.0f, (float) j / poses_e.size());
            glColor3f(1.0f, 1.0f, 0.f);//为了区分第二条轨迹，用不同的颜色代替,黄色
            glBegin(GL_LINES);
            auto p1 = poses_e[j], p2 = poses_e[j + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
 
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
 
}