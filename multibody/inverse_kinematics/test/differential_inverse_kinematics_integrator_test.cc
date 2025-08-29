#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_integrator.h"  // noqa

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace multibody {

namespace {

std::unique_ptr<multibody::MultibodyPlant<double>> MakeIiwa(void) {
  // Load the IIWA SDF, welding link_0 to the world.
  auto robot = std::make_unique<multibody::MultibodyPlant<double>>(0.01);
  multibody::Parser parser(robot.get());
  parser.AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  robot->WeldFrames(robot->world_frame(), robot->GetFrameByName("iiwa_link_0"));
  robot->Finalize();
  return robot;
}

// Tests that the LeafSystem behavior matches the function API.
GTEST_TEST(DifferentialInverseKinematicsIntegratorTest, BasicTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());

  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(*robot, frame_E, time_step,
                                                  params);
  auto diff_ik_context = diff_ik.CreateDefaultContext();

  // Set the results status state to a failure state.
  diff_ik_context->get_mutable_discrete_state(1).SetAtIndex(
      0, static_cast<double>(
             DifferentialInverseKinematicsStatus::kNoSolutionFound));

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
  for (const auto& [data, events] : diff_ik.MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(diff_ik, *diff_ik_context, discrete_state.get());
  }

  // Confirm that the result status state was updated properly.
  EXPECT_EQ(
      discrete_state->get_vector(1).GetAtIndex(0),
      static_cast<double>(DifferentialInverseKinematicsStatus::kSolutionFound));

  params.set_time_step(time_step);  // intentionally set this after diff_ik call
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      *robot, *robot_context, X_WE_desired, frame_E, params);

  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(q + time_step * result.joint_velocities.value(),
                              discrete_state->get_vector(0).get_value()));

  // Add infeasible constraints, and confirm the result state.
  // clang-format off
  const auto A = (MatrixX<double>(2, 7) <<
      1, -1,  0, 0, 0, 0, 0,
      0,  1, -1, 0, 0, 0, 0).finished();
  // clang-format on
  const auto b = Eigen::VectorXd::Zero(A.rows());
  diff_ik.get_mutable_parameters().AddLinearVelocityConstraint(
      std::make_shared<solvers::LinearConstraint>(A, b, b));
  Eigen::VectorXd last_q = robot->GetPositions(*robot_context);
  for (const auto& [data, events] : diff_ik.MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(diff_ik, *diff_ik_context, discrete_state.get());
  }
  EXPECT_EQ(discrete_state->get_vector(1).GetAtIndex(0),
            static_cast<double>(DifferentialInverseKinematicsStatus::kStuck));
  // kStuck does *not* imply that the positions will not advance.
  EXPECT_FALSE(CompareMatrices(discrete_state->get_value(0), last_q, 1e-12));
}

// Sets frame_A to be iiwa_link_0 instead of the world frame.
GTEST_TEST(DifferentialInverseKinematicsIntegratorTest, FrameATest) {
  // This time the iiwa will _not_ be welded to the world.
  auto robot = std::make_unique<multibody::MultibodyPlant<double>>(0.01);
  multibody::Parser parser(robot.get());
  parser.AddModelsFromUrl(
      "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  robot->Finalize();

  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_A =
      robot->GetFrameByName("iiwa_link_0");
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());

  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(*robot, frame_A, frame_E,
                                                  time_step, params);
  auto diff_ik_context = diff_ik.CreateDefaultContext();

  // Set the results status state to a failure state.
  diff_ik_context->get_mutable_discrete_state(1).SetAtIndex(
      0, static_cast<double>(
             DifferentialInverseKinematicsStatus::kNoSolutionFound));

  Eigen::VectorXd q(14);
  // clang-format off
  q << 0.1, 0.2,  -.15, 0.43, -0.32, -0.21, 0.11,
       0.4, 0.21, -.25, 0.67, -0.21, -0.53, 0.21;
  // clang-format on

  // Test SetPosition and ForwardKinematics.
  robot->SetPositions(robot_context.get(), q);
  diff_ik.SetPositions(diff_ik_context.get(), q);
  EXPECT_TRUE(diff_ik.ForwardKinematics(*diff_ik_context)
                  .IsExactlyEqualTo(frame_E.CalcPose(*robot_context, frame_A)));

  // Test that discrete update is equivalent to calling diff IK directly.
  math::RigidTransformd X_AE_desired(Eigen::Vector3d(0.2, 0.0, 0.2));
  diff_ik.GetInputPort("X_AE_desired")
      .FixValue(diff_ik_context.get(), X_AE_desired);
  auto discrete_state = diff_ik.AllocateDiscreteVariables();
  for (const auto& [data, events] : diff_ik.MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(diff_ik, *diff_ik_context, discrete_state.get());
  }

  // Confirm that the result status state was updated properly.
  EXPECT_EQ(
      discrete_state->get_vector(1).GetAtIndex(0),
      static_cast<double>(DifferentialInverseKinematicsStatus::kSolutionFound));

  params.set_time_step(time_step);  // intentionally set this after diff_ik call
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      *robot, *robot_context, X_AE_desired, frame_A, frame_E, params);

  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  Eigen::VectorXd qdot(robot->num_positions());
  robot->MapVelocityToQDot(*robot_context, result.joint_velocities.value(),
                           &qdot);
  EXPECT_TRUE(CompareMatrices(q + time_step * qdot,
                              discrete_state->get_vector(0).get_value()));
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(DeprecatedTest, BasicTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());

  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(*robot, frame_E, time_step,
                                                  params);
  auto diff_ik_context = diff_ik.CreateDefaultContext();

  // Set the results status state to a failure state.
  diff_ik_context->get_mutable_discrete_state(1).SetAtIndex(
      0, static_cast<double>(
             DifferentialInverseKinematicsStatus::kNoSolutionFound));

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
  diff_ik.GetInputPort("X_WE_desired")
      .FixValue(diff_ik_context.get(), X_WE_desired);
  auto discrete_state = diff_ik.AllocateDiscreteVariables();
  for (const auto& [data, events] : diff_ik.MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(diff_ik, *diff_ik_context, discrete_state.get());
  }

  // Confirm that the result status state was updated properly.
  EXPECT_EQ(
      discrete_state->get_vector(1).GetAtIndex(0),
      static_cast<double>(DifferentialInverseKinematicsStatus::kSolutionFound));

  params.set_time_step(time_step);  // intentionally set this after diff_ik call
  DifferentialInverseKinematicsResult result = DoDifferentialInverseKinematics(
      *robot, *robot_context, X_WE_desired, frame_E, params);

  EXPECT_EQ(result.status, DifferentialInverseKinematicsStatus::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(q + time_step * result.joint_velocities.value(),
                              discrete_state->get_vector(0).get_value()));

  // Add infeasible constraints, and confirm the result state.
  // clang-format off
  const auto A = (MatrixX<double>(2, 7) <<
      1, -1,  0, 0, 0, 0, 0,
      0,  1, -1, 0, 0, 0, 0).finished();
  // clang-format on
  const auto b = Eigen::VectorXd::Zero(A.rows());
  diff_ik.get_mutable_parameters().AddLinearVelocityConstraint(
      std::make_shared<solvers::LinearConstraint>(A, b, b));
  Eigen::VectorXd last_q = robot->GetPositions(*robot_context);
  for (const auto& [data, events] : diff_ik.MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(diff_ik, *diff_ik_context, discrete_state.get());
  }
  EXPECT_EQ(discrete_state->get_vector(1).GetAtIndex(0),
            static_cast<double>(DifferentialInverseKinematicsStatus::kStuck));
  // kStuck does *not* imply that the positions will not advance.
  EXPECT_FALSE(CompareMatrices(discrete_state->get_value(0), last_q, 1e-12));
}
#pragma GCC diagnostic pop

GTEST_TEST(DifferentialInverseKinematicsIntegratorTest, ParametersTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());
  const double time_step = 0.1;
  DifferentialInverseKinematicsIntegrator diff_ik(*robot, frame_E, time_step,
                                                  params);

  EXPECT_EQ(diff_ik.get_parameters().get_time_step(), time_step);
  diff_ik.get_mutable_parameters().set_time_step(0.2);
  EXPECT_EQ(diff_ik.get_parameters().get_time_step(), 0.2);
}

// Confirm that we can act like a difference equation system when the warning
// logic is disabled.
GTEST_TEST(DifferentialInverseKinematicsIntegratorTest,
           DifferenceEquationSystemTest) {
  auto robot = MakeIiwa();
  auto robot_context = robot->CreateDefaultContext();
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());
  const double time_step = 0.1;

  DifferentialInverseKinematicsIntegrator diff_ik_with_status_state(
      *robot, frame_E, time_step, params);
  EXPECT_EQ(diff_ik_with_status_state.num_discrete_state_groups(), 2);

  DifferentialInverseKinematicsIntegrator diff_ik_without_status_state(
      *robot, frame_E, time_step, params, nullptr, false);
  EXPECT_EQ(diff_ik_without_status_state.num_discrete_state_groups(), 1);
  EXPECT_TRUE(diff_ik_without_status_state.IsDifferenceEquationSystem());
}

GTEST_TEST(DifferentialInverseKinematicsIntegratorTest, InitializationEvent) {
  systems::DiagramBuilder<double> builder;
  auto robot = builder.AddSystem(MakeIiwa());
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());
  const double time_step = 0.1;
  auto diff_ik = builder.AddSystem<DifferentialInverseKinematicsIntegrator>(
      *robot, frame_E, time_step, params);
  builder.Connect(robot->get_state_output_port(),
                  diff_ik->GetInputPort("robot_state"));
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  systems::Context<double>& context = simulator.get_mutable_context();
  systems::Context<double>& robot_context =
      robot->GetMyMutableContextFromRoot(&context);
  robot->SetPositions(&robot_context, Eigen::VectorXd::Constant(7, 0.1));
  systems::Context<double>& diff_ik_context =
      diff_ik->GetMyMutableContextFromRoot(&context);
  diff_ik_context.FixInputPort(0, Value<math::RigidTransform<double>>());

  // Confirm that the initialization event sets the diff_ik positions to the
  // robot positions.
  EXPECT_FALSE(
      CompareMatrices(diff_ik_context.get_discrete_state(0).get_value(),
                      robot->GetPositions(robot_context)));
  simulator.Initialize();
  EXPECT_TRUE(CompareMatrices(diff_ik_context.get_discrete_state(0).get_value(),
                              robot->GetPositions(robot_context)));
}

GTEST_TEST(DifferentialInverseKinematicsIntegratorTest, UseRobotStatePort) {
  systems::DiagramBuilder<double> builder;
  auto robot = builder.AddSystem(MakeIiwa());
  const multibody::Frame<double>& frame_E =
      robot->GetFrameByName("iiwa_link_7");

  DifferentialInverseKinematicsParameters params(robot->num_positions(),
                                                 robot->num_velocities());
  const double time_step = 0.1;
  auto diff_ik = builder.AddSystem<DifferentialInverseKinematicsIntegrator>(
      *robot, frame_E, time_step, params);
  builder.Connect(robot->get_state_output_port(),
                  diff_ik->GetInputPort("robot_state"));
  auto diagram = builder.Build();
  systems::Simulator<double> simulator(*diagram);

  systems::Context<double>& context = simulator.get_mutable_context();
  systems::Context<double>& robot_context =
      robot->GetMyMutableContextFromRoot(&context);
  robot->SetPositions(&robot_context, Eigen::VectorXd::Constant(7, 0.1));
  systems::Context<double>& diff_ik_context =
      diff_ik->GetMyMutableContextFromRoot(&context);
  const math::RigidTransform<double> X_G =
      robot->GetBodyByName("iiwa_link_7").EvalPoseInWorld(robot_context);
  diff_ik_context.FixInputPort(0, Value<math::RigidTransform<double>>(X_G));
  auto discrete_state = diff_ik->AllocateDiscreteVariables();

  // Note: we intentionally do not call simulator.Initialize() nor send an
  // initialization event.

  // use_robot_state port is unset.
  for (const auto& [data, events] : diff_ik->MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(*diff_ik, diff_ik_context, discrete_state.get());
  }
  EXPECT_FALSE(CompareMatrices(discrete_state->get_value(0),
                               robot->GetPositions(robot_context)));

  // use_robot_state port is set to false.
  diff_ik_context.FixInputPort(
      diff_ik->GetInputPort("use_robot_state").get_index(), Value<bool>{false});
  for (const auto& [data, events] : diff_ik->MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(*diff_ik, diff_ik_context, discrete_state.get());
  }
  EXPECT_FALSE(CompareMatrices(discrete_state->get_value(0),
                               robot->GetPositions(robot_context)));

  // use_robot_state port is set to true.
  diff_ik_context.FixInputPort(
      diff_ik->GetInputPort("use_robot_state").get_index(), Value<bool>{true});
  for (const auto& [data, events] : diff_ik->MapPeriodicEventsByTiming()) {
    dynamic_cast<const systems::DiscreteUpdateEvent<double>*>(events[0])
        ->handle(*diff_ik, diff_ik_context, discrete_state.get());
  }
  // We set the desired pose to the current pose, so updating with true should
  // result in the integrator positions matching the robot positions.
  EXPECT_TRUE(CompareMatrices(discrete_state->get_value(0),
                              robot->GetPositions(robot_context), 1e-12));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
