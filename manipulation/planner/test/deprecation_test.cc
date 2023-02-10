// This file serves to check that our deprecation shims compile successfully,
// aside from warning messages.  We can remove this file entirely once the
// deprecation period ends.

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace manipulation {
namespace planner {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

const char* const kIiwaUrdf =
    "drake/manipulation/models/iiwa_description/urdf/"
    "iiwa14_polytope_collision.urdf";

GTEST_TEST(ManipulationPlannerPathsDeprecation, ConstraintRelaxingIk) {
  // Exercise the *single* type declared in the header.
  EXPECT_NO_THROW(
      ConstraintRelaxingIk(FindResourceOrThrow(kIiwaUrdf), "iiwa_link_7"));
}

GTEST_TEST(ManipulationPlannerPathsDeprecation, RobotPlanInterpolator) {
  // Include *both* types declared in the header file.
  EXPECT_NO_THROW(RobotPlanInterpolator(FindResourceOrThrow(kIiwaUrdf),
                                        InterpolatorType::ZeroOrderHold));
}

GTEST_TEST(ManipulationPlannerPathsDeprecation, DifferentialIK) {
  auto status = DifferentialInverseKinematicsStatus::kSolutionFound;
  EXPECT_EQ(status, DifferentialInverseKinematicsStatus::kSolutionFound);

  EXPECT_NO_THROW(DifferentialInverseKinematicsResult(
      {.joint_velocities = std::nullopt, .status = status}));

  EXPECT_NO_THROW(DifferentialInverseKinematicsParameters(1, 1));

  // All three spellings of DoDifferentialInverseKinematics; we'll get them
  // to throw.
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser parser(&plant);
  const std::string filename = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "planar_iiwa14_spheres_dense_elbow_collision.urdf");
  parser.AddModels(filename);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"));
  plant.Finalize();
  const multibody::Frame<double>& frame_7 = plant.GetFrameByName("iiwa_link_7");
  auto context = plant.CreateDefaultContext();

  // Configure the Diff IK.
  DifferentialInverseKinematicsParameters params(plant.num_positions(),
                                                 plant.num_velocities());
  const int num_joints = plant.num_positions();
  params.set_nominal_joint_position(VectorX<double>::Zero(num_joints));
  params.set_time_step(1e-3);
  // The planar model can only move in x, z, and rotations around the link 7
  // y-axis.
  params.set_end_effector_velocity_flag(
      (Vector6<bool>() << false, true, false, true, false, true).finished());

  // Set the initial plant state.  These values were randomly generated.
  const Vector3d q{0.1, 1.2, 1.6};
  const VectorXd v = VectorXd::Zero(plant.num_velocities());
  plant.SetPositions(context.get(), q);
  plant.SetVelocities(context.get(), v);

  multibody::SpatialVelocity<double> V_WE_W_desired(Vector3d(0, 1, 0),
                                                    Vector3d(0.1, 0, 0.3));
  EXPECT_NO_THROW(DoDifferentialInverseKinematics(
      plant, *context, V_WE_W_desired.get_coeffs(), frame_7, params));

  math::RigidTransform<double> X_WE_desired =
      math::RigidTransform<double>(Vector3d(-0.02, -0.01, -0.03));
  EXPECT_NO_THROW(DoDifferentialInverseKinematics(plant, *context, X_WE_desired,
                                                  frame_7, params));

  const int size = plant.num_velocities();
  EXPECT_NO_THROW(DoDifferentialInverseKinematics(
      plant.GetPositions(*context), plant.GetVelocities(*context), v,
      MatrixX<double>::Identity(size, size), params));

  // Random free function.
  EXPECT_NO_THROW(ComputePoseDiffInCommonFrame(X_WE_desired, X_WE_desired));
}

GTEST_TEST(ManipulationPlannerPathsDeprecation, DifferentialIKIntegrator) {
  // Load the IIWA SDF, welding link_0 to the world.
  auto robot = std::make_unique<multibody::MultibodyPlant<double>>(0.01);
  multibody::Parser parser(robot.get());
  const std::string filename = FindResourceOrThrow(
      "drake/manipulation/models/"
      "iiwa_description/sdf/iiwa14_no_collision.sdf");
  parser.AddModels(filename);
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();

  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());

  const double time_step = 0.1;
  EXPECT_NO_THROW(DifferentialInverseKinematicsIntegrator diff_ik(
      *robot, frame_E, time_step, params));
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
