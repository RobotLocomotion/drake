#include "drake/manipulation/planner/constraint_relaxing_ik.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/random.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace planner {

// N random samples are taken from the configuration space (q), and
// the corresponding end effector poses are computed with forward
// kinematics.  We use inverse kinematics to try to recover a set of
// joint angles that would achieve these poses. This test checks that
// an IK solution can be computed, and that the resulting pose lies
// within the given tolerance from the forward kinematics poses.
GTEST_TEST(ConstraintRelaxingIkTest, SolveIkFromFk) {
  const std::string kModelPath = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");
  multibody::MultibodyPlant<double> iiwa(0);
  multibody::Parser(&iiwa).AddModelFromFile(kModelPath);
  iiwa.WeldFrames(iiwa.world_frame(),
                  iiwa.GetBodyByName("base").body_frame());
  iiwa.Finalize();

  const std::string kEndEffectorLinkName = "iiwa_link_ee";
  const multibody::Body<double>& end_effector =
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
    iiwa.SetRandomState(
        *context, &context->get_mutable_state(), &rand_generator);

    math::RigidTransformd fk_pose = iiwa.EvalBodyPoseInWorld(
        *context, end_effector);
    waypoints[0].pose = fk_pose;

    std::vector<Eigen::VectorXd> q_sol;
    const bool ret =
        ik_planner.PlanSequentialTrajectory(waypoints, kQcurrent, &q_sol);
    ASSERT_TRUE(ret);

    iiwa.SetPositions(context.get(), q_sol.front());
    const math::RigidTransformd ik_pose = iiwa.EvalBodyPoseInWorld(
        *context, end_effector);

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
}

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
