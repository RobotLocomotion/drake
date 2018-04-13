#include "drake/automotive/agent_trajectory.h"

#include <algorithm>
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
static constexpr double kDeltaT = 2.;  // The expected time interval to traverse
                                       // a pair of waypoints.

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
  for (const auto& type : {Type::kFirstOrderHold, Type::kCubic, Type::kPchip}) {
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

enum class SpeedProfile { kPiecewiseConstant, kPiecewiseLinear };

// Computes a vector of x-y-z positions (with y, z fixed), either under a
// piecewise-constant or piecewise-linear interpolation of the vector of speeds,
// enforcing the expected time increment `kDeltaT` between each waypoint.  Pose
// starts from x-y-z position [1., 2., 3.].  Orientation is held constant at an
// orientation (r-p-y) of [0., 0., 0.] unless otherwise specified.
void MakePoses(const SpeedProfile speed_profile,
               const std::vector<double>& speeds,
               std::vector<Isometry3d>* poses,
               const Vector3d& rpy = Vector3d{0., 0., 0.}) {
  std::vector<Translation<double, 3>> translations(speeds.size());
  std::vector<Quaternion<double>> rotations(speeds.size());
  double displacement = 0.;
  for (int i{0}; i < static_cast<int>(speeds.size()); i++) {
    translations[i] = Translation<double, 3>(1. + displacement, 2., 3.);
    rotations[i] = RollPitchYaw<double>(rpy).ToQuaternion();
    poses->push_back(Isometry3d{translations[i]});
    poses->back().rotate(rotations[i]);

    if (i == static_cast<int>(speeds.size())) break;

    double interval_speed{0.};
    if (speed_profile == SpeedProfile::kPiecewiseConstant) {
      interval_speed = speeds[i];
    } else if (speed_profile == SpeedProfile::kPiecewiseLinear) {
      interval_speed = 0.5 * (speeds[i] + speeds[i + 1]);
    }
    displacement += kDeltaT * interval_speed;
  }
}

// Negative speeds are rejected.
GTEST_TEST(AgentTrajectoryTest, NegativeSpeeds) {
  const std::vector<double> speeds{-1, 5.};
  std::vector<Isometry3d> poses{};
  MakePoses(SpeedProfile::kPiecewiseConstant, speeds, &poses);

  EXPECT_THROW(AgentTrajectory::MakeCubicFromWaypoints(poses, speeds),
               std::exception);
  EXPECT_THROW(AgentTrajectory::MakeCubicFromWaypoints(poses, -1.),
               std::exception);
}

// Deadlock detection rejects unreachable waypoints.
GTEST_TEST(AgentTrajectoryTest, UnreachableCubicWaypoints) {
  const std::vector<double> speeds{0., 0., 1.};
  std::vector<Isometry3d> poses{};
  MakePoses(SpeedProfile::kPiecewiseLinear, speeds, &poses);

  EXPECT_THROW(AgentTrajectory::MakeCubicFromWaypoints(poses, speeds),
               std::exception);
}

// Deadlock detection rejects unreachable waypoints.
GTEST_TEST(AgentTrajectoryTest, UnreachableFohWaypoints) {
  const std::vector<double> speeds{0., 1., 1.};
  std::vector<Isometry3d> poses{};
  MakePoses(SpeedProfile::kPiecewiseConstant, speeds, &poses);
}

struct RpyCase {
  RpyCase(const Vector3d& rpy, const Vector3d& vel)
      : rpy_value(rpy), expected_velocity_basis(vel) {}
  const Vector3d rpy_value{};
  const Vector3d expected_velocity_basis{};  // Basis vector for velocity in
                                             // x-y-z coordinates.
};

// Checks that the provided speeds and waypoints yield correctly-formed
// trajectories using InterpolationType::kCubic on the linear quantities.
GTEST_TEST(AgentTrajectoryTest, MakeCubicFromWaypoints) {
  using std::max_element;
  using std::min_element;

  const std::vector<double> speeds{1., 5., 0.};
  std::vector<RpyCase> rpy_cases{};
  rpy_cases.push_back(RpyCase({0., 0., 0.}, {1., 0., 0.}));
  rpy_cases.push_back(RpyCase({0., 0., M_PI_2}, {0., 1., 0.}));
  rpy_cases.push_back(RpyCase({0., M_PI_2, 0.}, {0., 0., -1.}));
  rpy_cases.push_back(RpyCase({M_PI_2, 0., 0.}, {1., 0., 0.}));

  for (const auto& rpy_case : rpy_cases) {
    std::vector<Eigen::Isometry3d> poses{};
    MakePoses(SpeedProfile::kPiecewiseLinear, speeds, &poses,
              rpy_case.rpy_value);

    AgentTrajectory trajectory =
        AgentTrajectory::MakeCubicFromWaypoints(poses, speeds);

    double time{0.};
    for (int i{0}; i < static_cast<int>(speeds.size()); i++, time += kDeltaT) {
      // Evaluate at the expected time corresponding to the i-th waypoint.
      const PoseVelocity actual_at = trajectory.value(time);
      EXPECT_TRUE(CompareMatrices(actual_at.translation().vector(),
                                  poses[i].translation(), kTol));
      EXPECT_TRUE(CompareMatrices(actual_at.velocity().translational(),
                                  rpy_case.expected_velocity_basis * speeds[i],
                                  kTol));
      EXPECT_NEAR(actual_at.speed(), speeds[i], kTol);
      EXPECT_TRUE(CompareMatrices(actual_at.rotation().matrix(),
                                  poses[i].rotation().matrix(), kTol));
      EXPECT_TRUE(CompareMatrices(actual_at.velocity().rotational(),
                                  Vector3d{0., 0., 0.}, kTol));

      if (i == static_cast<int>(speeds.size()) - 1) break;

      // Evaluate between waypoints i and i+1.
      const PoseVelocity actual_between = trajectory.value(time + kDeltaT / 2.);
      EXPECT_GT(actual_between.translation().x(), poses[i].translation().x());
      EXPECT_LT(actual_between.translation().x(),
                poses[i + 1].translation().x());
      EXPECT_GT(actual_between.speed(),
                *min_element(speeds.begin(), speeds.end()));
      EXPECT_LT(actual_between.speed(),
                *max_element(speeds.begin(), speeds.end()));
      EXPECT_TRUE(CompareMatrices(actual_between.rotation().matrix(),
                                  poses[i].rotation().matrix(), kTol));
      EXPECT_TRUE(CompareMatrices(actual_between.velocity().rotational(),
                                  Vector3d{0., 0., 0.}, kTol));
    }
  }
}

// Checks that the provided waypoints yield correctly-formed trajectories with
// the constant-speed constructor.
GTEST_TEST(AgentTrajectoryTest, MakeCubicFromWaypointsWithConstantSpeed) {
  const double speed = 5.;
  const std::vector<double> speeds{speed, speed};  // Only for sizing `poses`.
  std::vector<Isometry3d> poses{};
  MakePoses(SpeedProfile::kPiecewiseConstant, speeds, &poses);

  AgentTrajectory trajectory =
      AgentTrajectory::MakeCubicFromWaypoints(poses, speed);

  const std::vector<double> expected_times{0., kDeltaT};
  for (int i{0}; i < static_cast<int>(expected_times.size()); i++) {
    const double time = expected_times[i];
    // Evaluate at the expected time corresponding to the i-th waypoint.
    const PoseVelocity actual_at = trajectory.value(time);
    EXPECT_TRUE(CompareMatrices(actual_at.translation().vector(),
                                poses[i].translation(), kTol));
    EXPECT_TRUE(CompareMatrices(actual_at.rotation().matrix(),
                                poses[i].rotation().matrix(), kTol));
    EXPECT_EQ(actual_at.speed(), speed);

    if (i == static_cast<int>(speeds.size()) - 1) break;

    // Evaluate between waypoints i and i+1.
    const PoseVelocity actual_between = trajectory.value(time + kDeltaT / 2.);
    EXPECT_NEAR(actual_between.speed(), speed, kTol);
  }
}

}  // namespace
}  // namespace automotive
}  // namespace drake
