#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"  // noqa

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {
namespace planner {

namespace {

std::unique_ptr<multibody::MultibodyPlant<double>> MakeIiwa(void) {
  // Load the IIWA SDF, welding link_0 to the world.
  auto robot = std::make_unique<multibody::MultibodyPlant<double>>(0.01);
  multibody::Parser parser(robot.get());
  const std::string filename = FindResourceOrThrow(
      "drake/manipulation/models/"
      "iiwa_description/sdf/iiwa14_no_collision.sdf");
  parser.AddModelFromFile(filename, "iiwa");
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();
  return robot;
}


// Tests that the LeafSystem behavior matches the function API.
GTEST_TEST(DifferentialInverseKinematicsIntegatorTest, BasicTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());

  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(
      *robot, frame_E, time_step, params);
  auto diff_ik_context = diff_ik.CreateDefaultContext();

  Eigen::VectorXd q(7);
  q << 0.1, 0.2, -.15, 0.43, -0.32, -0.21, 0.11;

  // Test SetPosition and ForwardKinematics.
  robot->SetPositions(robot_context.get(), q);
  diff_ik.SetPositions(diff_ik_context.get(), q);
  // Note: BodyPoseInWorld == FramePoseInWorld because this is a body frame.
  EXPECT_TRUE(diff_ik.ForwardKinematics(*diff_ik_context)
                  .IsExactlyEqualTo(robot->EvalBodyPoseInWorld(
                      *robot_context, frame_E.body())));

  // Test that discrete update is equivalent to calling diff IK directly.
  math::RigidTransformd X_WE_desired(Eigen::Vector3d(0.2, 0.0, 0.2));
  diff_ik.get_input_port(0).FixValue(diff_ik_context.get(), X_WE_desired);
  auto discrete_state = diff_ik.AllocateDiscreteVariables();
  diff_ik.CalcDiscreteVariableUpdates(*diff_ik_context, discrete_state.get());

  params.set_timestep(time_step);  // intentionally set this after diff_ik call
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      *robot, *robot_context, X_WE_desired.GetAsIsometry3(), frame_E, params);

  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(q + time_step * result.joint_velocities.value(),
                              discrete_state->get_vector(0).get_value()));
}

GTEST_TEST(DifferentialInverseKinematicsIntegatorTest, ParametersTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());
  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(
      *robot, frame_E, time_step, params);

  EXPECT_EQ(diff_ik.get_parameters().get_timestep(), time_step);
  diff_ik.get_mutable_parameters().set_timestep(0.2);
  EXPECT_EQ(diff_ik.get_parameters().get_timestep(), 0.2);
}

}  // namespace
}  // namespace planner
}  // namespace manipulation
}  // namespace drake
