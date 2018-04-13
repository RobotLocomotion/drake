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

using Eigen::Quaternion;
using Eigen::Vector3d;
using math::IsQuaternionValid;
using multibody::SpatialVelocity;

// Checks the defaults.
GTEST_TEST(PoseVelocityTest, Defaults) {
  const PoseVelocity actual;

  EXPECT_TRUE(CompareMatrices(actual.translation(), Vector3d::Zero()));
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

  const Vector3d translation{1., 2., 3.};
  const math::RollPitchYaw<double> rpy{0.4, 0.5, 0.6};
  const Quaternion<double> quaternion = rpy.ToQuaternion();
  const Vector3d w{8., 9., 10.};
  const Vector3d v{11., 12., 13.};
  const SpatialVelocity<double> velocity{w, v};
  const PoseVelocity actual(quaternion, translation, velocity);

  EXPECT_TRUE(CompareMatrices(actual.translation(), translation));
  EXPECT_TRUE(
      CompareMatrices(actual.rotation().matrix(), quaternion.matrix(), kTol));
  EXPECT_TRUE(CompareMatrices(actual.velocity().rotational(), w));
  EXPECT_TRUE(CompareMatrices(actual.velocity().translational(), v));
  const Vector3d expected_pose3{translation.x(), translation.y(),
                                rpy.get_yaw_angle()};
  EXPECT_TRUE(CompareMatrices(actual.pose3(), expected_pose3));
  EXPECT_EQ(actual.speed(), sqrt(pow(v(0), 2) + pow(v(1), 2) + pow(v(2), 2)));
}

void CheckAllConstructors(const std::vector<double>& times,
                          const std::vector<Quaternion<double>>& rotations,
                          const std::vector<Vector3d>& translations,
                          const std::vector<double> speeds) {
  EXPECT_THROW(AgentTrajectory::Make(times, rotations, translations),
               std::exception);
  EXPECT_THROW(
      AgentTrajectory::MakeCubicFromWaypoints(rotations, translations, speeds),
      std::exception);
  EXPECT_THROW(AgentTrajectory::MakeCubicFromWaypoints(rotations, translations,
                                                       speeds[0]),
               std::exception);
}

// Empty or mismatched-sized vectors are rejected.
GTEST_TEST(AgentTrajectoryTest, InvalidSizes) {
  const std::vector<double> times_3d{0., 1., 2.};
  const Quaternion<double> dummy_rotation = Quaternion<double>::Identity();
  const Vector3d dummy_translation = Vector3d::Zero();
  const std::vector<Quaternion<double>> rotations_empty{};
  const std::vector<Quaternion<double>> rotations_1d{dummy_rotation};
  const std::vector<Quaternion<double>> rotations_3d{
      dummy_rotation, dummy_rotation, dummy_rotation};
  const std::vector<Vector3d> translations_empty{};
  const std::vector<Vector3d> translations_1d{dummy_translation};
  const std::vector<Vector3d> translations_3d{
      dummy_translation, dummy_translation, dummy_translation};
  const std::vector<double> speeds_3d{3., 4., 5.};

  CheckAllConstructors(times_3d, rotations_empty, translations_3d, speeds_3d);
  CheckAllConstructors(times_3d, rotations_3d, translations_empty, speeds_3d);
  CheckAllConstructors(times_3d, rotations_1d, translations_1d, speeds_3d);
  CheckAllConstructors(times_3d, rotations_3d, translations_1d, speeds_3d);
  CheckAllConstructors(times_3d, rotations_1d, translations_3d, speeds_3d);
}

// Accepts all interpolation types.
GTEST_TEST(AgentTrajectoryTest, InterpolationType) {
  using Type = AgentTrajectory::InterpolationType;

  const std::vector<double> times{0., 1., 2.};
  const Quaternion<double> dummy_rotation = Quaternion<double>::Identity();
  const Vector3d dummy_translation = Vector3d::Zero();
  std::vector<Quaternion<double>> rotations{dummy_rotation, dummy_rotation,
                                            dummy_rotation};
  std::vector<Vector3d> translations{dummy_translation, dummy_translation,
                                     dummy_translation};
  for (const auto& type : {Type::kFirstOrderHold, Type::kCubic, Type::kPchip}) {
    EXPECT_NO_THROW(
        AgentTrajectory::Make(times, rotations, translations, type));
  }
}

// Checks that all data fields agree with the input data vectors at the knot
// points, and checks that the velocity components are correctly inferred.
GTEST_TEST(AgentTrajectoryTest, AgentTrajectory) {
  using Type = AgentTrajectory::InterpolationType;

  const std::vector<double> times{0., kDeltaT, 2 * kDeltaT};
  std::vector<Vector3d> translations{};
  std::vector<Quaternion<double>> rotations{};

  for (double time : times) {
    translations.push_back({1. + time, 2. + time, 3. + time});
    rotations.push_back({0.4 - 0.1 * time,  // BR
                         0.5 + 0.2 * time,  // BR
                         0.6 + 0.3 * time,  // BR
                         0.7 + 0.4 * time});
    rotations.back().normalize();
  }

  const AgentTrajectory trajectory = AgentTrajectory::Make(
      times, rotations, translations, Type::kFirstOrderHold);

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
    EXPECT_TRUE(CompareMatrices(actual.translation(), translations[i]));
    EXPECT_TRUE(CompareMatrices(actual.rotation().matrix(),
                                rotations[i].matrix(), kTol));

    // Check that the velocities are consistent with the translational movement
    // under linear interpolation and the velocities are as expected given the
    // change in rotations between time steps.
    const Vector3d v_expected{1., 1., 1.};
    EXPECT_TRUE(
        CompareMatrices(actual.velocity().translational(), v_expected, kTol));
    EXPECT_LT(0., actual.velocity().rotational().x());
    EXPECT_LT(0., actual.velocity().rotational().y());
    EXPECT_LT(0., actual.velocity().rotational().z());
  }
}

// Computes a vector of x-y-z positions (with y, z fixed), under
// piecewise-linear interpolation of the vector of speeds, enforcing the
// expected time increment `kDeltaT` between each waypoint.  `translations`
// starts from x-y-z position [1., 2., 3.].  `rotations` is held constant at an
// orientation (r-p-y) of [0., 0., 0.] unless otherwise specified.
void MakePoses(const std::vector<double>& speeds,
               std::vector<Quaternion<double>>* rotations,
               std::vector<Vector3d>* translations,
               const Vector3d& rpy = Vector3d{0., 0., 0.}) {
  rotations->resize(speeds.size());
  translations->resize(speeds.size());
  double displacement = 0.;
  for (int i{0}; i < static_cast<int>(speeds.size()); i++) {
    (*rotations)[i] = math::RollPitchYaw<double>(rpy).ToQuaternion();
    (*translations)[i] = Vector3d(1. + displacement, 2., 3.);

    double interval_speed{0.};
    interval_speed = 0.5 * (speeds[i] + speeds[i + 1]);
    displacement += kDeltaT * interval_speed;
  }
}

// Negative speeds are rejected.
GTEST_TEST(AgentTrajectoryTest, NegativeSpeeds) {
  const std::vector<double> speeds{-1, 5.};
  std::vector<Quaternion<double>> rotations{};
  std::vector<Vector3d> translations{};
  MakePoses(speeds, &rotations, &translations);

  EXPECT_THROW(
      AgentTrajectory::MakeCubicFromWaypoints(rotations, translations, speeds),
      std::exception);
  EXPECT_THROW(
      AgentTrajectory::MakeCubicFromWaypoints(rotations, translations, -1.),
      std::exception);
}

// Deadlock detection rejects unreachable waypoints.
GTEST_TEST(AgentTrajectoryTest, UnreachableCubicWaypoints) {
  const std::vector<double> speeds{0., 0., 1.};
  std::vector<Quaternion<double>> rotations{};
  std::vector<Vector3d> translations{};
  MakePoses(speeds, &rotations, &translations);

  EXPECT_THROW(
      AgentTrajectory::MakeCubicFromWaypoints(rotations, translations, speeds),
      std::exception);
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
    std::vector<Quaternion<double>> rotations{};
    std::vector<Vector3d> translations{};
    MakePoses(speeds, &rotations, &translations, rpy_case.rpy_value);

    const AgentTrajectory trajectory = AgentTrajectory::MakeCubicFromWaypoints(
        rotations, translations, speeds);

    double time{0.};
    for (int i{0}; i < static_cast<int>(speeds.size()); i++, time += kDeltaT) {
      // Evaluate at the expected time corresponding to the i-th waypoint.
      const PoseVelocity actual_at = trajectory.value(time);
      EXPECT_TRUE(
          CompareMatrices(actual_at.translation(), translations[i], kTol));
      EXPECT_TRUE(CompareMatrices(actual_at.velocity().translational(),
                                  rpy_case.expected_velocity_basis * speeds[i],
                                  kTol));
      EXPECT_NEAR(actual_at.speed(), speeds[i], kTol);
      EXPECT_TRUE(CompareMatrices(actual_at.rotation().matrix(),
                                  rotations[i].matrix(), kTol));
      EXPECT_TRUE(CompareMatrices(actual_at.velocity().rotational(),
                                  Vector3d{0., 0., 0.}, kTol));

      if (i == static_cast<int>(speeds.size()) - 1) break;

      // Evaluate between waypoints i and i + 1.
      const PoseVelocity actual_between = trajectory.value(time + kDeltaT / 2.);
      EXPECT_GT(actual_between.translation().x(), translations[i].x());
      EXPECT_LT(actual_between.translation().x(), translations[i + 1].x());
      EXPECT_GT(actual_between.speed(),
                *min_element(speeds.begin(), speeds.end()));
      EXPECT_LT(actual_between.speed(),
                *max_element(speeds.begin(), speeds.end()));
      EXPECT_TRUE(CompareMatrices(actual_between.rotation().matrix(),
                                  rotations[i].matrix(), kTol));
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
  std::vector<Quaternion<double>> rotations{};
  std::vector<Vector3d> translations{};
  MakePoses(speeds, &rotations, &translations);

  const AgentTrajectory trajectory =
      AgentTrajectory::MakeCubicFromWaypoints(rotations, translations, speed);

  const std::vector<double> expected_times{0., kDeltaT};
  for (int i{0}; i < static_cast<int>(expected_times.size()); i++) {
    const double time = expected_times[i];
    // Evaluate at the expected time corresponding to the i-th waypoint.
    const PoseVelocity actual_at = trajectory.value(time);
    EXPECT_TRUE(
        CompareMatrices(actual_at.translation(), translations[i], kTol));
    EXPECT_TRUE(CompareMatrices(actual_at.rotation().matrix(),
                                rotations[i].matrix(), kTol));
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
