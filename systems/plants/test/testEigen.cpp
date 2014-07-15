#include <Eigen/Core>
#include <iostream>

template <int i>
Eigen::Matrix<double, 6, Eigen::Dynamic> bla()
{
  return Eigen::Matrix<double, 6, i>::Random();
}

int main(int argc, char **argv) {
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  auto mat = bla<3>();
  std::cout << mat.RowsAtCompileTime << std::endl << std::endl;
//  Matrix6d m = Matrix6d::Random();
//  Vector6d v = Vector6d::Random();
//  std::cout << v << std::endl << std::endl;

//  std::cout << m * v << std::endl << std::endl;

//  Eigen::Matrix3d m = Eigen::Matrix3d::Random();
  std::cout << mat << std::endl << std::endl;

  return 0;
}
