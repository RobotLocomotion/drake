#include "drake/planning/distance_and_interpolation_provider.h"

#include <limits>

#include <common_robotics_utilities/math.hpp>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"

namespace drake {
namespace planning {
namespace {

// "Broken" implementation that returns invalid distance and interpolated qs.
class BrokenDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BrokenDistanceAndInterpolationProvider);

  BrokenDistanceAndInterpolationProvider() = default;

  ~BrokenDistanceAndInterpolationProvider() final = default;

 private:
  double DoComputeConfigurationDistance(const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& to) const final {
    unused(from);
    unused(to);
    // Throws because distances cannot be negative.
    return -1.0;
  }

  Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const final {
    unused(to);
    unused(ratio);
    // Throws because interpolated configurations must be the same size as the
    // provided from and to configurations.
    return Eigen::VectorXd::Zero(from.size() + 1);
  }
};

// Basic linear implementation of distance and interpolation.
class SimpleLinearDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleLinearDistanceAndInterpolationProvider);

  SimpleLinearDistanceAndInterpolationProvider() = default;

  ~SimpleLinearDistanceAndInterpolationProvider() final = default;

 private:
  double DoComputeConfigurationDistance(const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& to) const final {
    return (to - from).norm();
  }

  Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const final {
    return common_robotics_utilities::math::InterpolateXd(from, to, ratio);
  }
};

GTEST_TEST(BrokenDistanceAndInterpolationProviderTest, Test) {
  const BrokenDistanceAndInterpolationProvider provider;

  // Distance with different length qs throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                            Eigen::VectorXd::Zero(3)),
      ".* condition 'from\\.size\\(\\) == to\\.size\\(\\)' failed.*");

  // Interpolation with different length qs throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Zero(2),
                                                Eigen::VectorXd::Zero(3), 0.0),
      ".* condition 'from\\.size\\(\\) == to\\.size\\(\\)' failed.*");

  // Interpolation with ratios outside [0, 1] throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Zero(2),
                                                Eigen::VectorXd::Zero(2), -0.1),
      ".* condition 'ratio >= 0\\.0' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Zero(2),
                                                Eigen::VectorXd::Zero(2), 1.1),
      ".* condition 'ratio <= 1\\.0' failed.*");

  // "Broken" provider causes all distance queries to throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                            Eigen::VectorXd::Zero(2)),
      ".* condition 'distance >= 0\\.0' failed.*");

  // "Broken" provider causes all interpolation queries to throw.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Zero(2),
                                                Eigen::VectorXd::Zero(2), 0.0),
      ".* condition 'from\\.size\\(\\) == interpolated\\.size\\(\\)' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Zero(2),
                                                Eigen::VectorXd::Zero(2), 1.0),
      ".* condition 'from\\.size\\(\\) == interpolated\\.size\\(\\)' failed.*");
}

GTEST_TEST(SimpleLinearDistanceAndInterpolationProviderTest, Test) {
  const SimpleLinearDistanceAndInterpolationProvider provider;

  const Eigen::VectorXd kOnes = Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd kZeros = Eigen::VectorXd::Zero(2);

  const Eigen::VectorXd kInfs =
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());
  const Eigen::VectorXd kNans =
      Eigen::VectorXd::Constant(2, std::numeric_limits<double>::infinity());

  // Distance with different length qs throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.ComputeConfigurationDistance(kZeros, Eigen::VectorXd::Zero(3)),
      ".* condition 'from\\.size\\(\\) == to\\.size\\(\\)' failed.*");

  // Interpolation with different length qs throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(kZeros,
                                                Eigen::VectorXd::Zero(3), 0.0),
      ".* condition 'from\\.size\\(\\) == to\\.size\\(\\)' failed.*");

  // Interpolation with ratios outside [0, 1] throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(kZeros, kZeros, -0.1),
      ".* condition 'ratio >= 0\\.0' failed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      provider.InterpolateBetweenConfigurations(kZeros, kZeros, 1.1),
      ".* condition 'ratio <= 1\\.0' failed.*");

  // Distance queries.
  EXPECT_EQ(provider.ComputeConfigurationDistance(kZeros, kZeros), 0.0);
  EXPECT_EQ(provider.ComputeConfigurationDistance(kOnes, kZeros),
            std::sqrt(2.0));

  EXPECT_THROW(provider.ComputeConfigurationDistance(kOnes, kInfs),
               std::exception);
  EXPECT_THROW(provider.ComputeConfigurationDistance(kInfs, kOnes),
               std::exception);
  EXPECT_THROW(provider.ComputeConfigurationDistance(kOnes, kNans),
               std::exception);
  EXPECT_THROW(provider.ComputeConfigurationDistance(kNans, kOnes),
               std::exception);

  // Interpolation queries.
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(kOnes, kZeros, 0.0), kOnes));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(kOnes, kZeros, 1.0), kZeros));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(kOnes, kZeros, 0.5),
      kOnes * 0.5));

  EXPECT_THROW(provider.InterpolateBetweenConfigurations(kOnes, kInfs, 0),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(kInfs, kOnes, 0),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(kOnes, kNans, 0),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(kNans, kOnes, 0),
               std::exception);
}

}  // namespace
}  // namespace planning
}  // namespace drake
