#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"

#include <cmath>
#include <memory>
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
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FirstOrderTaylorApproximation;
using systems::controllers::PidController;

class IcfExternalSystemsLinearizerTest : public ::testing::Test {
 protected:
  // Builds the diagram and creates a context.
  // The plant must already have been finalized.
  void Build() {
    diagram_ = builder_->Build();
    builder_.reset();
    diagram_context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &plant_.GetMyMutableContextFromRoot(diagram_context_.get());
  }

  // A convenience wrapper around the DUT's LinearizeExternalSystem method that
  // uses optional return values instead of output arguments.
  struct Result {
    std::optional<IcfLinearFeedbackGains<double>> actuation_feedback;
    std::optional<IcfLinearFeedbackGains<double>> external_feedback;
  };
  Result LinearizeExternalSystem(double h) const {
    DRAKE_DEMAND(plant_context_ != nullptr);
    const IcfExternalSystemsLinearizer<double> dut(&plant_);
    const int nv = plant_.num_velocities();
    Result result;
    result.actuation_feedback.emplace();
    result.actuation_feedback->Resize(nv);
    result.external_feedback.emplace();
    result.external_feedback->Resize(nv);
    bool has_actuation_forces{};
    bool has_feedback_forces{};
    std::tie(has_actuation_forces, has_feedback_forces) =
        dut.LinearizeExternalSystem(h, *plant_context_,
                                    &result.actuation_feedback.value(),
                                    &result.external_feedback.value());
    if (!has_actuation_forces) {
      result.actuation_feedback.reset();
    }
    if (!has_feedback_forces) {
      result.external_feedback.reset();
    }
    return result;
  }

  // Only available prior to calling Build().
  std::unique_ptr<DiagramBuilder<double>> builder_{
      std::make_unique<DiagramBuilder<double>>()};

  // Always available.
  MultibodyPlant<double>& plant_{
      AddMultibodyPlantSceneGraph(builder_.get(), 0.0).plant};

  // Only available after calling Build().
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
  Context<double>* plant_context_{};
};

// Checks the short-circuit logic for a plant with no external input.
TEST_F(IcfExternalSystemsLinearizerTest, NoFeedback) {
  // Build a diagram with just the plant and scene graph (no controller).
  plant_.Finalize();
  Build();

  // No feedback.
  const auto result = LinearizeExternalSystem(/* h = */ 0.01);
  EXPECT_FALSE(result.actuation_feedback.has_value());
  EXPECT_FALSE(result.external_feedback.has_value());
}

// MJCF model of an actuated double pendulum.
constexpr char kActuatedPendulumXml[] = R"""(
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
TEST_F(IcfExternalSystemsLinearizerTest, ActuationInput) {
  // Build a diagram containing an actuated double pendulum and PID controller.
  Parser(builder_.get()).AddModelsFromString(kActuatedPendulumXml, "xml");
  plant_.Finalize();
  auto target_source = builder_->AddSystem<ConstantVectorSource<double>>(
      Vector4d{M_PI_2, M_PI_2, 0.0, 0.0});
  auto pid_controller = builder_->AddSystem<PidController>(
      Vector2d{0.24, 0.19}, Vector2d{0.35, 0.3}, Vector2d::Zero());
  builder_->Connect(target_source->get_output_port(),
                    pid_controller->get_input_port_desired_state());
  builder_->Connect(plant_.get_state_output_port(),
                    pid_controller->get_input_port_estimated_state());
  builder_->Connect(pid_controller->get_output_port(),
                    plant_.get_actuation_input_port());
  Build();
  const Context<double>& pid_controller_context =
      pid_controller->GetMyContextFromRoot(*diagram_context_);

  // Set an interesting initial state.
  plant_.SetPositions(plant_context_, Vector2d{0.1, 0.2});
  plant_.SetVelocities(plant_context_, Vector2d{0.3, 0.4});

  // Linearize the non-plant dynamics around the current state.
  const double h = 0.01;
  const auto result = LinearizeExternalSystem(h);
  ASSERT_TRUE(result.actuation_feedback.has_value());
  EXPECT_FALSE(result.external_feedback.has_value());
  const VectorXd& K = result.actuation_feedback->K;
  const VectorXd& b = result.actuation_feedback->b;

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
  const int nq = plant_.num_positions();
  const int nv = plant_.num_velocities();
  const VectorXd x0 = plant_.GetPositionsAndVelocities(*plant_context_);
  const VectorXd v0 = x0.tail(nv);
  VectorXd x_tilde0 = x0;
  x_tilde0.head(nq) += h * v0;
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
