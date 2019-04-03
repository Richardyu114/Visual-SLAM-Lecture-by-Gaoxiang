#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)

   // 第一种方法，用fstream的getline分行读取stringstream按空格拆分传入数组

    ifstream fin(trajectory_file); //从文件中读取数据
    if(!fin.is_open()){
        cout<<"No "<<trajectory_file<<endl;
        return 0;
    }
    double t,tx,ty,tz,qx,qy,qz,qw;
    string line;
    while(getline(fin,line))
    {
       istringstream record(line); //从string读取数据
       record>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
       Eigen::Vector3d p(tx,ty,tz);
       Eigen::Quaterniond q = Eigen::Quaterniond(qw,qx,qy,qz).normalized();
       Sophus::SE3 SE3_qp(q,p);
       poses.push_back(SE3_qp);
    }

    //第二种方法

    // ifstream in(trajectory_file);//创建输入流

    // if(!in){
    //     cout<<"open posefile failture!!!"<<endl;
    //     return 0;
    // }

    // for(int i=0; i<620; i++){

    //     double data[8]={0};
    //     for(auto& d:data) in>>d;//按行依次去除数组中的值

    //     Eigen::Quaterniond q(data[7], data[8], data[5], data[6]);
    //     Eigen::Vector3d t(data[1], data[2], data[3]);
    //     Sophus::SE3 SE3(q,t);
    //     poses.push_back(SE3);

    // }
    // end your code here

    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

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
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//窗口，rgba,最后一个参数是透明度

        glLineWidth(2); //线宽
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size()); //颜色随位置变化而变化
            //glColor3f(0.f, 0.8f, 0.f);//绿色
    
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1]; //只显示tx,ty,tz
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}


     
