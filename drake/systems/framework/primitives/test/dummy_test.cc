
#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "gtest/gtest.h"

using Eigen::AutoDiffScalar;
using Eigen::Vector2d;
using Eigen::Vector3d;

/// A column vector of size 3, templated on scalar type.
template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

GTEST_TEST(WindowsCITest, AutoDiff) {
  typedef AutoDiffScalar<Vector2d> T;

  Vector3<T> input_vector(1.0, 3.14, 2.18);

  Vector3<T> expected;

  const double kGain = 2.0;
  expected =  kGain * input_vector;

  const double tolerance = Eigen::NumTraits<double>::epsilon();
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(
        expected(i).value(), kGain * input_vector(i).value(), tolerance);
  }
}
