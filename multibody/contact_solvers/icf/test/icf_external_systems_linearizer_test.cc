#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"

#include <cmath>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using systems::ConstantVectorSource;
using systems::Context;
using systems::DiagramBuilder;
using systems::FirstOrderTaylorApproximation;
using systems::controllers::PidController;

// MJCF model of an actuated double pendulum.
const char actuated_pendulum_xml[] = R"""(
<?xml version="1.0"?>
<mujoco model="robot">
  <worldbody>
    <body>
      <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1"/>
      <geom type="capsule" size="0.01 0.1"/>
      <body>
        <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1"/>
        <geom type="capsule" size="0.01 0.1" pos="0 0 -0.2"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="joint1" ctrlrange="-2 2"/>
    <motor joint="joint2" ctrlrange="-3 3"/>
  </actuator>
</mujoco>
)""";

// Run tests with an actuated pendulum and an external (PID) controller.
GTEST_TEST(IcfExternalSystemsLinearizerTest, Basic) {
  // Build a diagram containing an actuated double pendulum and PID controller.
  DiagramBuilder<double> builder;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  Parser(&plant, &scene_graph)
      .AddModelsFromString(actuated_pendulum_xml, "xml");
  plant.Finalize();
  auto target_source = builder.AddSystem<ConstantVectorSource<double>>(
      Vector4d{M_PI_2, M_PI_2, 0.0, 0.0});
  auto pid_controller = builder.AddSystem<PidController>(
      Vector2d{0.24, 0.19}, Vector2d{0.35, 0.3}, Vector2d::Zero());
  builder.Connect(target_source->get_output_port(),
                  pid_controller->get_input_port_desired_state());
  builder.Connect(plant.get_state_output_port(),
                  pid_controller->get_input_port_estimated_state());
  builder.Connect(pid_controller->get_output_port(),
                  plant.get_actuation_input_port());
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());
  const Context<double>& pid_controller_context =
      pid_controller->GetMyContextFromRoot(*diagram_context);

  // Set an interesting initial state.
  plant.SetPositions(&plant_context, Vector2d{0.1, 0.2});
  plant.SetVelocities(&plant_context, Vector2d{0.3, 0.4});

  // Linearize the non-plant dynamics around the current state.
  const IcfExternalSystemsLinearizer<double> dut(&plant);
  const int nv = plant.num_velocities();
  IcfLinearFeedbackGains<double> actuation_feedback;
  IcfLinearFeedbackGains<double> external_feedback;  // unused
  actuation_feedback.Resize(nv);
  external_feedback.Resize(nv);
  bool has_actuation_forces{};
  bool has_feedback_forces{};
  const double h = 0.01;
  std::tie(has_actuation_forces, has_feedback_forces) =
      dut.LinearizeExternalSystem(h, plant_context, &actuation_feedback,
                                  &external_feedback);
  ASSERT_TRUE(has_actuation_forces);
  unused(has_feedback_forces);
  const VectorXd& K = actuation_feedback.K;
  const VectorXd& b = actuation_feedback.b;

  // Compute linearization τ = D⋅x + y around x₀ via autodiff.
  auto expected_linearization = FirstOrderTaylorApproximation(
      *pid_controller, pid_controller_context,
      pid_controller->get_input_port_estimated_state().get_index(),
      pid_controller->get_output_port().get_index());
  const MatrixXd& D = expected_linearization->D();
  const VectorXd& y = expected_linearization->y0();
  const MatrixXd dtau_dq = D.leftCols(2);
  const MatrixXd dtau_dv = D.rightCols(2);
  const MatrixXd dtau_tilde_dv = dtau_dv + h * dtau_dq;  // N(q) = I

  // Since the linearization is exact for a PID controller, we can compute
  // τ(x̃₀) = D⋅x̃₀ + y.
  const VectorXd x0 = plant.GetPositionsAndVelocities(plant_context);
  const VectorXd v0 = x0.tail(nv);
  VectorXd x_tilde0 = x0;
  x_tilde0.head(nv) += h * v0;
  const VectorXd tau_tilde0 = y + D * x_tilde0;

  // And thus the expected linearization τ(v) = b - K⋅v is:
  const VectorXd K_ref = -dtau_tilde_dv.diagonal();
  const VectorXd b_ref = tau_tilde0 + K_ref.asDiagonal() * v0;

  // Confirm that our finite difference linearization is close to the reference.
  const double kTol = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(K, K_ref, kTol, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(b, b_ref, kTol, MatrixCompareType::relative));
}

// TODO(jwnimmer-tri) We should be sure to include an external controller where
// the linearization changes significantly with q. That's surprisingly not very
// common. For instance a PD controller has constant derivatives, and an inverse
// dynamics controller only changes slowly with q.

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
