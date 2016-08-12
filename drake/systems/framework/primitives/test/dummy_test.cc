
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

int main() {
  typedef AutoDiffScalar<Vector2d> T;

  Vector3<T> input_vector(1.0, 3.14, 2.18);

  Vector3<T> expected;

  const double kGain = 2.0;
  expected =  kGain * input_vector;

  return 0;
}
