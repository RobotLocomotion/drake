#include "drake/multibody/inverse_kinematics/constraint_relaxing_ik.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/random.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {

// N random samples are taken from the configuration space (q), and
// the corresponding end effector poses are computed with forward
// kinematics.  We use inverse kinematics to try to recover a set of
// joint angles that would achieve these poses. This test checks that
// an IK solution can be computed, and that the resulting pose lies
// within the given tolerance from the forward kinematics poses.
GTEST_TEST(ConstraintRelaxingIkTest, SolveIkFromFk) {
  const std::string kModelPath = PackageMap{}.ResolveUrl(
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  MultibodyPlant<double> iiwa(0);
  Parser(&iiwa).AddModels(kModelPath);
  iiwa.WeldFrames(iiwa.world_frame(), iiwa.GetBodyByName("base").body_frame());
  iiwa.Finalize();

  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  const RigidBody<double>& end_effector =
      iiwa.GetBodyByName(kEndEffectorLinkName);

  ConstraintRelaxingIk ik_planner(kModelPath, kEndEffectorLinkName);

  ConstraintRelaxingIk::IkCartesianWaypoint wp;
  wp.pos_tol = Vector3<double>(0.001, 0.001, 0.001);
  wp.rot_tol = 0.005;
  wp.constrain_orientation = true;
  std::vector<ConstraintRelaxingIk::IkCartesianWaypoint> waypoints(1, wp);

  std::unique_ptr<systems::Context<double>> context =
      iiwa.CreateDefaultContext();

  const VectorX<double> kQcurrent = iiwa.GetPositions(*context);

  const double kEpsilon = 1e-8;
  const Vector3<double> kUpperBound =
      wp.pos_tol + kEpsilon * Vector3<double>::Ones();
  const Vector3<double> kLowerBound =
      -wp.pos_tol - kEpsilon * Vector3<double>::Ones();
  RandomGenerator rand_generator(1234);

  for (int i = 0; i < 100; ++i) {
    iiwa.SetRandomState(*context, &context->get_mutable_state(),
                        &rand_generator);

    math::RigidTransformd fk_pose =
        iiwa.EvalBodyPoseInWorld(*context, end_effector);
    waypoints[0].pose = fk_pose;

    std::vector<Eigen::VectorXd> q_sol;
    const bool ret =
        ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &q_sol);
    ASSERT_TRUE(ret);

    iiwa.SetPositions(context.get(), q_sol.front());
    const math::RigidTransformd ik_pose =
        iiwa.EvalBodyPoseInWorld(*context, end_effector);

    const Vector3<double> pos_diff =
        ik_pose.translation() - fk_pose.translation();
    const math::RotationMatrix R_diff =
        ik_pose.rotation().transpose() * fk_pose.rotation();
    const double rot_diff = R_diff.ToAngleAxis().angle();

    EXPECT_TRUE((pos_diff.array() <= kUpperBound.array()).all());
    EXPECT_TRUE((pos_diff.array() >= kLowerBound.array()).all());

    // cos(ang_diff) >= cos(tol) is the actual constraint in the IK.
    EXPECT_TRUE(std::cos(rot_diff) + kEpsilon >= std::cos(wp.rot_tol));
  }

  // Check the keep_going feature.
  // We'll need several waypoints.
  waypoints.push_back(waypoints.front());
  waypoints.push_back(waypoints.front());
  waypoints.push_back(waypoints.front());
  int prior_index = -1;
  std::vector<Eigen::VectorXd> q_sol;
  auto give_up_early = [&prior_index](int waypoint_index) {
    // The waypoint index should be monotonic.
    EXPECT_GE(waypoint_index, prior_index);
    // The waypoint index should increment one at a time.
    EXPECT_LE(waypoint_index, prior_index + 1);
    // Remember the current waypoint.
    prior_index = waypoint_index;
    // Stop at the third waypoint.
    const bool keep_going = (waypoint_index < 2);
    return keep_going;
  };
  // Giving up early should return a planner failure.
  EXPECT_FALSE(ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &q_sol,
                                                   give_up_early));
  EXPECT_EQ(prior_index, 2);
  // Bash on regardless.
  auto never_stop = [](int) {
    return true;
  };
  EXPECT_TRUE(ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &q_sol,
                                                  never_stop));
}

}  // namespace multibody
}  // namespace drake
