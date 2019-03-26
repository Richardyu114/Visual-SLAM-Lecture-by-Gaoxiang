#include <iostream>
using namespace std;

#include <ctime>

// Eigen部分
#include <Eigen/Core>
//Eigen稠密矩阵的代数运算（逆和特征值等）
#include <Eigen/Dense>


int main (int argc, char** argv)
{

Eigen::Matrix <double,Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic; //Eigen固定大小矩阵最大支持到50
Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> matrix_A;
Eigen::Matrix<double,Eigen::Dynamic, 1> x;
Eigen::Matrix<double,Eigen::Dynamic,1> v_right;

matrix_dynamic = Eigen::MatrixXd::Random(100,100); //随机化取值

matrix_A = matrix_dynamic.transpose()*matrix_dynamic; //cholesky分解需要A为正定矩阵

v_right = Eigen::MatrixXd::Random(100, 1); //方程右边的值随机取值


//QR Decomposition
clock_t time_stt = clock();

x = matrix_A.colPivHouseholderQr().solve(v_right);
cout<<"the time used in QR decomposition is "<< 1000* (clock() - time_stt)/(double) CLOCKS_PER_SEC<<"ms"<< endl;
cout<<x<<endl;

//Cholesky Decomposition
time_stt = clock();

x = matrix_A.llt().solve(v_right);
cout<<"the time used in Cholesky decomposition is "<< 1000* (clock() - time_stt)/(double) CLOCKS_PER_SEC<<"ms"<< endl;
cout<<x<<endl;
return 0;

}

