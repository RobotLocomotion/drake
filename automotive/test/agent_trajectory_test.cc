#include "drake/automotive/agent_trajectory.h"

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace automotive {
namespace {

using Eigen::Isometry3d;
using Eigen::Quaternion;
using Eigen::Translation;
using Eigen::Vector3d;

// Checks the defaults.
GTEST_TEST(AgentTrajectoryValuesTest, Defaults) {
  const AgentTrajectoryValues actual;

  EXPECT_TRUE(CompareMatrices(actual.isometry3().matrix(),
                              Isometry3d::Identity().matrix()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().rotational(), Vector3d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().translational(), Vector3d::Zero()));
}

// Checks the accessors.
GTEST_TEST(AgentTrajectoryValuesTest, AgentTrajectoryValues) {
  using std::pow;
  using std::sqrt;

  const Translation<double, 3> translation{1., 2., 3.};
  const Vector3d rpy{0.4, 0.5, 0.6};
  const Quaternion<double> rotation = math::RollPitchYawToQuaternion(rpy);
  const Vector3d w{8., 9., 10.};
  const Vector3d v{11., 12., 13.};
  const SpatialVelocity<double> velocity{w, v};
  const AgentTrajectoryValues actual(rotation, translation, velocity);

  Isometry3d expected_isometry(translation);
  expected_isometry.rotate(rotation);
  EXPECT_TRUE(
      CompareMatrices(actual.isometry3().matrix(), expected_isometry.matrix()));
  EXPECT_TRUE(CompareMatrices(actual.velocity().rotational(), w));
  EXPECT_TRUE(CompareMatrices(actual.velocity().translational(), v));
  const Vector3d expected_pose3{translation.x(), translation.y(), rpy.z()};
  EXPECT_TRUE(CompareMatrices(actual.pose3(), expected_pose3));
  EXPECT_EQ(actual.speed(), sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2)));
}

// Mismatched-sized vectors are rejected.
GTEST_TEST(AgentTrajectoryTest, MismatchedSizes) {
  std::vector<double> times{0., 1., 2.};  // A 3D vector.
  const AgentTrajectoryValues dummy_value;
  std::vector<AgentTrajectoryValues> values{dummy_value};  // A 1D vector.
  EXPECT_THROW(
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, values),
      std::exception);
}

// Accepts all interpolation types.
GTEST_TEST(AgentTrajectoryTest, InterpolationType) {
  using Type = InterpolationType;

  std::vector<double> times{0., 1., 2.};
  const AgentTrajectoryValues dummy_value;
  std::vector<AgentTrajectoryValues> values{dummy_value, dummy_value,
                                            dummy_value};
  for (const auto& type : {Type::kZeroOrderHold, Type::kFirstOrderHold,
                           Type::kCubic, Type::kPchip}) {
    EXPECT_NO_THROW(AgentTrajectory::Make(type, times, values));
  }
}

// Checks that all data fields should agree with the input data vectors at the
// knot points.
GTEST_TEST(AgentTrajectoryTest, AgentTrajectory) {
  std::vector<double> times{0., 1., 2.};
  std::vector<AgentTrajectoryValues> values{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};
  std::vector<Vector3d> w_vector{};
  std::vector<Vector3d> v_vector{};

  for (const double time : times) {
    translations.push_back({1. + time, 2., 3.});
    rotations.push_back({4. + time, 5., 6., 7.});
    rotations.back().normalize();
    w_vector.push_back({8. + time, 9., 10.});
    v_vector.push_back({11. + time, 12., 13.});
    const SpatialVelocity<double> velocity{w_vector.back(), v_vector.back()};
    values.push_back({rotations.back(), translations.back(), velocity});
  }
  AgentTrajectory trajectory =
      AgentTrajectory::Make(InterpolationType::kFirstOrderHold, times, values);

  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const AgentTrajectoryValues actual = trajectory.value(times[i]);
    EXPECT_TRUE(CompareMatrices(actual.isometry3().translation(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.isometry3().rotation().matrix(),
                                rotations[i].matrix(), 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().rotational(), w_vector[i], 1e-12));
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), v_vector[i], 1e-12));
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
