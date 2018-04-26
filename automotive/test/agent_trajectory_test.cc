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

static constexpr double kTol = 1e-12;

using math::IsQuaternionValid;
using math::RollPitchYaw;
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
GTEST_TEST(PoseVelocityTest, Accessors) {
  using std::pow;
  using std::sqrt;

  const Translation<double, 3> translation{1., 2., 3.};
  const Vector3d rpy{0.4, 0.5, 0.6};
  const Quaternion<double> rotation = RollPitchYaw<double>(rpy).ToQuaternion();
  const Vector3d w{8., 9., 10.};
  const Vector3d v{11., 12., 13.};
  const SpatialVelocity<double> velocity{w, v};
  const PoseVelocity actual(rotation, translation, velocity);

  EXPECT_TRUE(
      CompareMatrices(actual.translation().vector(), translation.vector()));
  EXPECT_TRUE(
      CompareMatrices(actual.rotation().matrix(), rotation.matrix(), kTol));
  EXPECT_TRUE(CompareMatrices(actual.velocity().rotational(), w));
  EXPECT_TRUE(CompareMatrices(actual.velocity().translational(), v));
  const Vector3d expected_pose3{translation.x(), translation.y(), rpy.z()};
  EXPECT_TRUE(CompareMatrices(actual.pose3(), expected_pose3));
  EXPECT_EQ(actual.speed(), sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2)));
}

// Mismatched-sized vectors are rejected.
GTEST_TEST(AgentTrajectoryTest, MismatchedSizes) {
  const std::vector<double> times{0., 1., 2.};  // A 3D vector.
  const Isometry3d dummy_value(Quaternion<double>::Identity());
  std::vector<Isometry3d> values{dummy_value};  // A 1D vector.
  EXPECT_THROW(AgentTrajectory::Make(times, values), std::exception);
}

// Accepts all interpolation types.
GTEST_TEST(AgentTrajectoryTest, InterpolationType) {
  using Type = AgentTrajectory::InterpolationType;

  const std::vector<double> times{0., 1., 2.};
  const Isometry3d dummy_value(Quaternion<double>::Identity());
  std::vector<Isometry3d> values{dummy_value, dummy_value, dummy_value};
  for (const auto& type : {Type::kZeroOrderHold, Type::kFirstOrderHold,
                           Type::kCubic, Type::kPchip}) {
    EXPECT_NO_THROW(AgentTrajectory::Make(times, values, type));
  }
}

// Checks that all data fields agree with the input data vectors at the knot
// points, and checks that the velocity components are correctly inferred.
GTEST_TEST(AgentTrajectoryTest, AgentTrajectory) {
  using Type = AgentTrajectory::InterpolationType;

  const double delta_t = 1.;
  const std::vector<double> times{0., delta_t, 2 * delta_t};
  std::vector<Isometry3d> poses{};
  std::vector<Translation<double, 3>> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (double time : times) {
    translations.push_back({1. + time, 2. + time, 3. + time});
    rotations.push_back({0.4 - 0.1 * time,  // BR
                         0.5 + 0.2 * time,  // BR
                         0.6 + 0.3 * time,  // BR
                         0.7 + 0.4 * time});
    rotations.back().normalize();
    Isometry3d pose(translations.back());
    pose.rotate(rotations.back());
    poses.push_back(pose);
  }

  const AgentTrajectory trajectory =
      AgentTrajectory::Make(times, poses, Type::kFirstOrderHold);

  for (int i{0}; i < static_cast<int>(times.size()); i++) {
    if (i < static_cast<int>(times.size()) - 1) {
      // Check that the quaternions are valid when evaluated away from the knot
      // points.
      const double time = times[i] + 0.5 * (times[i] + times[i + 1]);
      const PoseVelocity actual = trajectory.value(time);
      EXPECT_TRUE(IsQuaternionValid(actual.rotation(), kTol));
    }

    // Check that the translations and rotations match the input data at the
    // knot points.
    const PoseVelocity actual = trajectory.value(times[i]);
    EXPECT_TRUE(IsQuaternionValid(actual.rotation(), kTol));
    EXPECT_TRUE(CompareMatrices(actual.translation().vector(),
                                translations[i].vector()));
    EXPECT_TRUE(CompareMatrices(actual.rotation().matrix(),
                                rotations[i].matrix(), kTol));

    // Check that the velocities are consistent with the translational movement
    // under linear interpolation and the velocities are as expected given the
    // change in rotations between time steps.
    const Vector3d v_expected{1. / delta_t, 1. / delta_t, 1. / delta_t};
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), v_expected, kTol));
    EXPECT_LT(0., actual.velocity().rotational().x());
    EXPECT_LT(0., actual.velocity().rotational().y());
    EXPECT_LT(0., actual.velocity().rotational().z());
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
