#include "drake/math/evenly_distributed_pts_on_sphere.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace math {
namespace {
void EvenlyDistributedPtsOnSphereFibonacciTest(int num_samples, double tol) {
  const auto& pts = UniformPtsOnSphereFibonacci(num_samples);
  // Check the size of the output.
  EXPECT_EQ(pts.rows(), 3);
  EXPECT_EQ(pts.cols(), num_samples);
  // Check if all the points have unit length.
  EXPECT_TRUE(CompareMatrices(pts.colwise().norm(),
                              Eigen::RowVectorXd::Ones(num_samples), 1E-6,
                              MatrixCompareType::absolute));

  // Now take a cone with axis `a` and half angle θ, compute the number of
  // points in `pts` that are within this cone. The ratio between the number of
  // points in the cone and `num_samples`, should be roughly equal to the ratio
  // between the intersection of cone and sphere surface, and the sphere area.
  std::vector<std::pair<Eigen::Vector3d, double>> cones;
  cones.emplace_back(Eigen::Vector3d(1, 0, 0), M_PI / 10);
  cones.emplace_back(Eigen::Vector3d(0, 1, 0), M_PI / 5);
  cones.emplace_back(Eigen::Vector3d(0, 0, 1), M_PI / 6);
  cones.emplace_back(Eigen::Vector3d(-1.0 / 3, 2.0 / 3, -2.0 / 3), M_PI / 5);
  cones.emplace_back(Eigen::Vector3d(1 / std::sqrt(2), 0, 1 / std::sqrt(2)),
                     M_PI / 3);
  cones.emplace_back(
      Eigen::Vector3d(1 / std::sqrt(2), -1 / std::sqrt(3), -1 / std::sqrt(6)),
      M_PI / 4);
  for (const auto& cone : cones) {
    const auto& a = cone.first;
    const double theta = cone.second;
    EXPECT_NEAR(a.norm(), 1.0, 1E-6);
    int num_pts_in_cone = 0;
    const double cos_theta = cos(theta);
    for (int i = 0; i < num_samples; ++i) {
      num_pts_in_cone += static_cast<int>(a.dot(pts.col(i)) >= cos_theta);
    }
    // The area of the intersection region is 2π(1-cosθ), the area of the unit
    // sphere is 4π, so the ratio #pts_in_cone / #samples should be
    // approximately equal to 2π(1-cosθ) / 4π = (1 - cosθ) / 2
    EXPECT_NEAR(num_pts_in_cone / static_cast<double>(num_samples),
                (1 - cos_theta) / 2, tol);
  }
}

GTEST_TEST(TestEvenlyDistributedPtsOnSphere, Fibonacci) {
  EvenlyDistributedPtsOnSphereFibonacciTest(100, 0.03);
  EvenlyDistributedPtsOnSphereFibonacciTest(1000, 0.01);
}
}  // namespace
}  // namespace math
}  // namespace drake
