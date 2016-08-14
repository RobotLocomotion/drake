
#include <iostream>
using namespace std;

#define EIGEN_DONT_ALIGN

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

/// A column vector of dynamic size, templated on scalar type.
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

int main() {
  typedef AutoDiffScalar<Vector2d> T;

  VectorX<T> a(3);
  a << 1.0, 3.14, 2.18;

  const double constant = 2.0;
  VectorX<T> b =  constant * a;

  return 0;
}
