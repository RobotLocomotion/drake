#include "drake/automotive/agent_trajectory.h"

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/roll_pitch_yaw.h"

namespace drake {
namespace automotive {
namespace {

using multibody::SpatialVelocity;
using Eigen::Isometry3d;
using Eigen::Quaternion;
using Eigen::Translation;
using Eigen::Vector3d;

// Checks the defaults.
GTEST_TEST(PoseVelocityTest, Defaults) {
  const PoseVelocity actual;

  EXPECT_TRUE(CompareMatrices(actual.translation().vector(),
                              Translation<double, 3>::Identity().vector()));
  EXPECT_TRUE(CompareMatrices(actual.rotation().matrix(),
                              Quaternion<double>::Identity().matrix()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().rotational(), Vector3d::Zero()));
  EXPECT_TRUE(
      CompareMatrices(actual.velocity().translational(), Vector3d::Zero()));
}

// Checks the accessors.
GTEST_TEST(PoseVelocityTest, PoseVelocity) {
  using std::pow;
  using std::sqrt;

  const Translation<double, 3> translation{1., 2., 3.};
  const Vector3d rpy{0.4, 0.5, 0.6};
  const Quaternion<double> rotation =
      math::RollPitchYaw<double>(rpy).ToQuaternion();
  const Vector3d w{8., 9., 10.};
  const Vector3d v{11., 12., 13.};
  const SpatialVelocity<double> velocity{w, v};
  const PoseVelocity actual(rotation, translation, velocity);

  EXPECT_TRUE(
      CompareMatrices(actual.translation().vector(), translation.vector()));
  EXPECT_TRUE(
      CompareMatrices(actual.rotation().matrix(), rotation.matrix(), 1e-12));
  EXPECT_TRUE(CompareMatrices(actual.velocity().rotational(), w));
  EXPECT_TRUE(CompareMatrices(actual.velocity().translational(), v));
  const Vector3d expected_pose3{translation.x(), translation.y(), rpy.z()};
  EXPECT_TRUE(CompareMatrices(actual.pose3(), expected_pose3));
  EXPECT_EQ(actual.speed(), sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2)));
}

// Mismatched-sized vectors are rejected.
GTEST_TEST(AgentTrajectoryTest, MismatchedSizes) {
  std::vector<double> times{0., 1., 2.};  // A 3D vector.
  const PoseVelocity dummy_value;
  std::vector<PoseVelocity> values{dummy_value};  // A 1D vector.
  EXPECT_THROW(AgentTrajectory::Make(times, values), std::exception);
}

// Accepts all interpolation types.
GTEST_TEST(AgentTrajectoryTest, InterpolationType) {
  using Type = InterpolationType;

  std::vector<double> times{0., 1., 2.};
  const PoseVelocity dummy_value;
  std::vector<PoseVelocity> values{dummy_value, dummy_value, dummy_value};
  for (const auto& type : {Type::kZeroOrderHold, Type::kFirstOrderHold,
                           Type::kCubic, Type::kPchip}) {
    EXPECT_NO_THROW(AgentTrajectory::Make(times, values, type));
  }
}

// Checks that all data fields should agree with the input data vectors at the
// knot points.
GTEST_TEST(AgentTrajectoryTest, AgentTrajectory) {
  std::vector<double> times{0., 1., 2.};
  std::vector<PoseVelocity> values{};
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
  AgentTrajectory trajectory = AgentTrajectory::Make(times, values);

  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    const PoseVelocity actual = trajectory.value(times[i]);
    EXPECT_TRUE(CompareMatrices(actual.translation().vector(),
                                translations[i].vector(), 1e-12));
    EXPECT_TRUE(CompareMatrices(actual.rotation().matrix(),
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
