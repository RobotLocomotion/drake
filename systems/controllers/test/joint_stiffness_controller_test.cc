#include "drake/systems/controllers/joint_stiffness_controller.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using multibody::MultibodyPlant;

GTEST_TEST(JointStiffnessControllerTest, SimpleDoublePendulum) {
  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<MultibodyPlant>(0.0);
  std::string full_name = FindResourceOrThrow(
      "drake/multibody/benchmarks/acrobot/double_pendulum.urdf");
  multibody::Parser(plant).AddModels(full_name);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"));
  plant->Finalize();

  Vector2d kp{0.3, 0.4}, kd{0.1, 0.2};

  auto controller = builder.AddSystem<JointStiffnessController>(*plant, kp, kd);
  EXPECT_EQ(&controller->get_multibody_plant(), plant);
  builder.Connect(plant->get_state_output_port(),
                  controller->get_input_port_estimated_state());
  builder.Connect(controller->get_output_port(),
                  plant->get_actuation_input_port());
  builder.ExportInput(controller->get_input_port_desired_state(),
                      "desired_state");

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant->GetMyMutableContextFromRoot(context.get());
  auto& controller_context =
      controller->GetMyMutableContextFromRoot(context.get());

  Vector4d x{-0.7, 0.1, 0.5, 0.4}, x_d{-0.75, 0.3, 0.02, -0.5};
  plant->SetPositionsAndVelocities(&plant_context, x);
  diagram->get_input_port().FixValue(context.get(), x_d);

  // We expect the controller to cancel gravity and damping, and add the
  // stiffness terms.
  const double kDamping = 0.1;  // must match double_pendulum.urdf
  VectorXd tau_expected = -plant->CalcGravityGeneralizedForces(plant_context) +
                          kDamping * x.tail<2>() +
                          (kp.array() * (x_d.head<2>() - x.head<2>()).array() +
                           kd.array() * (x_d.tail<2>() - x.tail<2>()).array())
                              .matrix();
  VectorXd tau = controller->get_output_port().Eval(controller_context);

  EXPECT_TRUE(CompareMatrices(tau, tau_expected, 1e-14));
}

GTEST_TEST(JointStiffnessControllerTest, ScalarConversion) {
  auto mbp = std::make_unique<MultibodyPlant<double>>(0.0);
  const std::string full_name = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf");
  multibody::Parser(mbp.get()).AddModels(full_name);
  mbp->WeldFrames(mbp->world_frame(), mbp->GetFrameByName("iiwa_link_0"));
  mbp->Finalize();
  const int num_states = mbp->num_multibody_states();
  const int num_q = mbp->num_positions();

  VectorXd kp = VectorXd::Constant(num_q, 0.12),
           kd = VectorXd::Constant(num_q, 0.34);

  JointStiffnessController<double> controller(*mbp, kp, kd);

  // Test AutoDiffXd.
  auto controller_ad = systems::System<double>::ToAutoDiffXd(controller);
  // Check the multibody plant.
  EXPECT_EQ(controller_ad->get_input_port_estimated_state().size(), num_states);

  // Test Expression.
  auto controller_sym = systems::System<double>::ToSymbolic(controller);
  EXPECT_EQ(controller_sym->get_input_port_estimated_state().size(),
            num_states);

  JointStiffnessController<double> controller_with_ownership(std::move(mbp), kp,
                                                             kd);
  // Test AutoDiffXd.
  controller_ad =
      systems::System<double>::ToAutoDiffXd(controller_with_ownership);
  // Check the multibody plant.
  EXPECT_EQ(controller_ad->get_input_port_estimated_state().size(), num_states);

  // Test Expression.
  controller_sym =
      systems::System<double>::ToSymbolic(controller_with_ownership);
  EXPECT_EQ(controller_sym->get_input_port_estimated_state().size(),
            num_states);
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
