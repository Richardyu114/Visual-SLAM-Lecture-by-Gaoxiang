#include<iostream>
#include<cmath>
using namespace std;

//Eigen几何模块
#include <Eigen/Core>
#include<Eigen/Geometry>


int main(int argc, char** argv)
{

Eigen::Quaterniond q1(0.55,0.3,0.2,0.2);
q1 = q1.normalized();

Eigen::Quaterniond q2(-0.1,0.3,-0.7,0.2);
q2 = q2.normalized();

Eigen::Matrix <double, 3,1> t1;
t1 << 0.7,1.1,0.2;

Eigen::Matrix <double, 3,1> t2;
t2 << -0.1,0.4,0.8;

Eigen::Matrix <double, 3,1> p1;
p1 << 0.5,-0.1,0.2;

Eigen::Matrix <double, 3,1> p2;

Eigen::Matrix <double, 3,1> p; //世界坐标系的点

//利用变换矩阵的方法

// Eigen::Isometry3d Tcw1 = Eigen::Isometry3d::Identity();//变换矩阵1
// Tcw1.rotate(q1);
// Tcw1.pretranslate(t1);

// Eigen::Isometry3d Tcw2 = Eigen::Isometry3d::Identity();//变换矩阵2
// Tcw2.rotate(q2);
// Tcw2.pretranslate(t2);

// p = Tcw1.inverse()*p1;
// p2 = Tcw2*p;


//直接利用旋转矩阵和平移向量进行组合运算
Eigen::Matrix<double,3,3> R1_inverse;
R1_inverse = q1.matrix().inverse();

Eigen::Matrix<double,3,3> R2;
R2 = q2.matrix();

p2 = R2 * R1_inverse *(p1 - t1) + t2;


cout << "p2 = " << p2.transpose() << endl;

return 0;

}