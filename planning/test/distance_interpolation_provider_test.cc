#include "drake/planning/distance_interpolation_provider.h"

#include <common_robotics_utilities/math.hpp>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/unused.h"

namespace drake {
namespace planning {
namespace {

// "Broken" implementation that returns invalid distance and interpolated qs.
class BrokenDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  BrokenDistanceAndInterpolationProvider() = default;

  ~BrokenDistanceAndInterpolationProvider() final = default;

 private:
  // Copy constructor for use in Clone().
  BrokenDistanceAndInterpolationProvider(
      const BrokenDistanceAndInterpolationProvider& other) = default;

  std::unique_ptr<DistanceAndInterpolationProvider> DoClone() const final {
    return std::unique_ptr<DistanceAndInterpolationProvider>(
        new BrokenDistanceAndInterpolationProvider(*this));
  }

  double DoComputeConfigurationDistance(const Eigen::VectorXd& from,
                                        const Eigen::VectorXd& to) const final {
    drake::unused(from);
    drake::unused(to);
    // Throws because distances cannot be negative.
    return -1.0;
  }

  Eigen::VectorXd DoInterpolateBetweenConfigurations(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      double ratio) const final {
    drake::unused(to);
    drake::unused(ratio);
    // Throws because interpolated configurations must be the same size as the
    // provided from and to configurations.
    return Eigen::VectorXd::Zero(from.size() + 1);
  }
};

// Basic linear implementation of distance and interpolation.
class SimpleLinearDistanceAndInterpolationProvider final
    : public DistanceAndInterpolationProvider {
 public:
  SimpleLinearDistanceAndInterpolationProvider() = default;

  ~SimpleLinearDistanceAndInterpolationProvider() final = default;

 private:
  // Copy constructor for use in Clone().
  SimpleLinearDistanceAndInterpolationProvider(
      const SimpleLinearDistanceAndInterpolationProvider& other) = default;

  std::unique_ptr<DistanceAndInterpolationProvider> DoClone() const final {
    return std::unique_ptr<DistanceAndInterpolationProvider>(
        new SimpleLinearDistanceAndInterpolationProvider(*this));
  }

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

  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // Distance with different length qs throws.
  EXPECT_THROW(provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                                     Eigen::VectorXd::Zero(3)),
               std::exception);

  // Interpolation with different length qs throws.
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(3), 0.0),
               std::exception);

  // Interpolation with ratios outside [0, 1] throws.
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), -0.1),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 1.1),
               std::exception);

  // "Broken" provider causes all distance queries to throw.
  EXPECT_THROW(provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                                     Eigen::VectorXd::Zero(2)),
               std::exception);
  EXPECT_THROW(provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                                     Eigen::VectorXd::Zero(2)),
               std::exception);

  // "Broken" provider causes all interpolation queries to throw.
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 0.0),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 1.0),
               std::exception);
}

GTEST_TEST(SimpleLinearDistanceAndInterpolationProviderTest, Test) {
  const SimpleLinearDistanceAndInterpolationProvider provider;

  // Test Clone().
  const auto cloned = provider.Clone();
  EXPECT_NE(cloned, nullptr);

  // Distance with different length qs throws.
  EXPECT_THROW(provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                                     Eigen::VectorXd::Zero(3)),
               std::exception);

  // Interpolation with different length qs throws.
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(3), 0.0),
               std::exception);

  // Interpolation with ratios outside [0, 1] throws.
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), -0.1),
               std::exception);
  EXPECT_THROW(provider.InterpolateBetweenConfigurations(
                   Eigen::VectorXd::Zero(2), Eigen::VectorXd::Zero(2), 1.1),
               std::exception);

  // Distance queries.
  EXPECT_EQ(provider.ComputeConfigurationDistance(Eigen::VectorXd::Zero(2),
                                                  Eigen::VectorXd::Zero(2)),
            0.0);
  EXPECT_EQ(provider.ComputeConfigurationDistance(Eigen::VectorXd::Ones(2),
                                                  Eigen::VectorXd::Zero(2)),
            std::sqrt(2.0));

  // Interpolation queries.
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Ones(2),
                                                Eigen::VectorXd::Zero(2), 0.0),
      Eigen::VectorXd::Ones(2)));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Ones(2),
                                                Eigen::VectorXd::Zero(2), 1.0),
      Eigen::VectorXd::Zero(2)));
  EXPECT_TRUE(CompareMatrices(
      provider.InterpolateBetweenConfigurations(Eigen::VectorXd::Ones(2),
                                                Eigen::VectorXd::Zero(2), 0.5),
      Eigen::VectorXd::Constant(2, 0.5)));
}

}  // namespace
}  // namespace planning
}  // namespace drake
