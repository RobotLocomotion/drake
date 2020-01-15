#include "drake/math/random_rotation.h"

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace math {
namespace {

/**
 * According to wikipedia
 * https://en.wikipedia.org/wiki/Rotation_matrix#Uniform_random_rotation_matrices
 * the cdf of θ angle should be 1/π (θ - sinθ) if 0 <= θ <= π.
 * Basic confidence interval statistics.  See, for instance,
 *    Simulation and the Monte Carlo Method (Third Edition)
 *      by Rubinstein and Kroese, 2017
 *    page 108,
 * where I've used Y as the indicator function of a ≤ X ≤ a+h,
 * E[Y] = Phi(a+h)-Phi(a), Var(Y) = E[Y]-E[Y]², since E[Y²]=E[Y],
 * and Phi(x) is the cdf of the standard normal distribution.
 * We expect a perfect sampler to obtain a value
 *    count/N = E[Y] ± 1.645 √(Var(Y)/N)
 * with 95% confidence.  Checks that our pseudo-random sampler
 * accomplishes this bound within a factor of fudge_factor.
 */
template <typename Orientation>
void CheckUniformAngleAxes(const std::vector<Orientation>& orientations,
                           int num_intervals, double fudge_factor) {
  const double h = M_PI / num_intervals;
  std::vector<int> count(num_intervals, 0);
  for (const auto& orientation : orientations) {
    const int interval_index =
        floor(Eigen::AngleAxisd(orientation).angle() / h);
    DRAKE_DEMAND(interval_index < num_intervals);
    count[interval_index]++;
  }
  // cdf[i] is the cdf of theta up to h * i
  std::vector<double> cdf(num_intervals + 1, 0);
  for (int i = 1; i < num_intervals + 1; ++i) {
    const double theta_up = h * i;
    cdf[i] = 1. / M_PI * (theta_up - std::sin(theta_up));
  }

  for (int i = 0; i < num_intervals; ++i) {
    const double EY = cdf[i + 1] - cdf[i];
    const double VarY = EY - EY * EY;
    EXPECT_NEAR(static_cast<double>(count[i]) / orientations.size(), EY,
                fudge_factor * 1.645 * std::sqrt(VarY / orientations.size()));
  }
}

GTEST_TEST(RandomRotationTest, UniformlyRandomRotation) {
  RandomGenerator generator;

  const int num_samples = 100000;
  std::vector<Eigen::Quaterniond> quaternions(num_samples);
  std::vector<Eigen::AngleAxisd> angle_axes(num_samples);
  std::vector<Eigen::Matrix3d> rotation_matrices(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    quaternions[i] = UniformlyRandomQuaternion<double>(&generator);
    angle_axes[i] = UniformlyRandomAngleAxis<double>(&generator);
    rotation_matrices[i] =
        UniformlyRandomRotationMatrix<double>(&generator).matrix();
  }
  const int num_intervals = 20;
  const double fudge_factor = 2;
  CheckUniformAngleAxes(quaternions, num_intervals, fudge_factor);
  CheckUniformAngleAxes(angle_axes, num_intervals, fudge_factor);
  CheckUniformAngleAxes(rotation_matrices, num_intervals, fudge_factor);
}

GTEST_TEST(RandomRotationTest, Symbolic) {
  RandomGenerator generator;

  const Eigen::AngleAxis<symbolic::Expression> axis_angle =
      UniformlyRandomAngleAxis<symbolic::Expression>(&generator);
  EXPECT_EQ(axis_angle.angle().GetVariables().size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(axis_angle.axis()[i].GetVariables().size(), 3);
  }

  const Eigen::Quaternion<symbolic::Expression> quaternion =
      UniformlyRandomQuaternion<symbolic::Expression>(&generator);
  EXPECT_EQ(quaternion.w().GetVariables().size(), 2);
  EXPECT_EQ(quaternion.x().GetVariables().size(), 2);
  EXPECT_EQ(quaternion.y().GetVariables().size(), 2);
  EXPECT_EQ(quaternion.z().GetVariables().size(), 2);

  const RotationMatrix<symbolic::Expression> rotation_matrix =
      UniformlyRandomRotationMatrix<symbolic::Expression>(&generator);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_EQ(rotation_matrix.matrix()(i, j).GetVariables().size(), 3);
    }
  }

  // TODO(russt): Add UniformlyRandomRPY test once RollPitchYaw supports
  // symbolic::Expression.
}

}  // namespace
}  // namespace math
}  // namespace drake
