#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
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
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::FirstOrderTaylorApproximation;
using systems::controllers::PidController;

class LinearizerTestFixture {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearizerTestFixture);

  LinearizerTestFixture() = default;
  virtual ~LinearizerTestFixture() = default;

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

class IcfExternalSystemsLinearizerTest : public ::testing::Test,
                                         public LinearizerTestFixture {};

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

// Run a test with a pendulum and an external (PID) controller.
// When `choose_plant_port == 0`, MbP actuation input is used.
// When `choose_plant_port == 1`, MbP applied generalized force input is used.
class IcfExternalSystemsLinearizerPidTest
    : public LinearizerTestFixture,
      public ::testing::TestWithParam<int /* choose_plant_port */> {};

TEST_P(IcfExternalSystemsLinearizerPidTest, ActuationInput) {
  const int choose_plant_port = GetParam();

  // Build a diagram containing a double pendulum and PID controller.
  Parser(builder_.get()).AddModelsFromString(kActuatedPendulumXml, "xml");
  if (choose_plant_port == 1) {
    // We'll be using generalized force input, not actuation.
    plant_.RemoveJointActuator(
        plant_.get_joint_actuator(JointActuatorIndex(1)));
    plant_.RemoveJointActuator(
        plant_.get_joint_actuator(JointActuatorIndex(0)));
  }
  plant_.Finalize();
  auto target_source = builder_->AddSystem<ConstantVectorSource<double>>(
      Vector4d{M_PI_2, M_PI_2, 0.0, 0.0});
  auto pid_controller = builder_->AddSystem<PidController>(
      Vector2d{0.24, 0.19}, Vector2d{0.35, 0.3}, Vector2d::Zero());
  builder_->Connect(target_source->get_output_port(),
                    pid_controller->get_input_port_desired_state());
  builder_->Connect(plant_.get_state_output_port(),
                    pid_controller->get_input_port_estimated_state());
  if (choose_plant_port == 0) {
    builder_->Connect(pid_controller->get_output_port(),
                      plant_.get_actuation_input_port());
  } else {
    DRAKE_DEMAND(choose_plant_port == 1);
    builder_->Connect(pid_controller->get_output_port(),
                      plant_.get_applied_generalized_force_input_port());
  }

  Build();
  const Context<double>& pid_controller_context =
      pid_controller->GetMyContextFromRoot(*diagram_context_);

  // Set an interesting initial state.
  plant_.SetPositions(plant_context_, Vector2d{0.1, 0.2});
  plant_.SetVelocities(plant_context_, Vector2d{0.3, 0.4});

  // Linearize the non-plant dynamics around the current state.
  const double h = 0.01;
  const auto result = LinearizeExternalSystem(h);
  const std::optional<IcfLinearFeedbackGains<double>>& relevant_feedback =
      (choose_plant_port == 0) ? result.actuation_feedback
                               : result.external_feedback;
  const std::optional<IcfLinearFeedbackGains<double>>& empty_feedback =
      (choose_plant_port == 0) ? result.external_feedback
                               : result.actuation_feedback;
  ASSERT_TRUE(relevant_feedback.has_value());
  EXPECT_FALSE(empty_feedback.has_value());
  const VectorXd& K = relevant_feedback->K;
  const VectorXd& b = relevant_feedback->b;

  // Compute linearization τ̃ = D⋅x + y around x₀ via autodiff.
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

  // And thus the expected linearization τ̃(v) = b - K⋅v is:
  const VectorXd K_ref = -dtau_tilde_dv.diagonal();
  const VectorXd b_ref = tau_tilde0 + K_ref.asDiagonal() * v0;

  // Confirm that our finite difference linearization is close to the reference.
  const double kTol = std::sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(K, K_ref, kTol, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(b, b_ref, kTol, MatrixCompareType::relative));
}

INSTANTIATE_TEST_SUITE_P(AllPorts, IcfExternalSystemsLinearizerPidTest,
                         ::testing::Values(0, 1));

// TODO(#23918) We should test an external controller where qdot != v.

// TODO(#23918) We should test an external controller where the linearization
// changes significantly with q. That's surprisingly not very common. For
// instance a PD controller has constant derivatives, and an inverse dynamics
// controller only changes slowly with q.

// A feedback "controller" that takes the plant's state_output_port as input and
// produces applied spatial forces as output. It assumes a 1-dof plant -- a ball
// with a prismatic joint to the world moving on the +x axis.
class SpatialForceFeedback final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpatialForceFeedback);

  static constexpr double kGain = 22;

  explicit SpatialForceFeedback(BodyIndex body_index)
      : body_index_(body_index) {
    const int nq = 1;
    const int nv = 1;
    DeclareVectorInputPort("in", nq + nv);
    DeclareAbstractOutputPort("out", &SpatialForceFeedback::CalcOutput);
  }

 private:
  void CalcOutput(
      const Context<double>& context,
      std::vector<ExternallyAppliedSpatialForce<double>>* output) const {
    // Initialize the output.
    output->resize(1);
    ExternallyAppliedSpatialForce<double>& result = output->at(0);

    // Our input is connected to the the plant's state_output_port.
    auto q_v = get_input_port().Eval(context);
    const double q = q_v(0);
    const double v = q_v(1);

    // The feedback term is f = -(K v + q).
    const double f_x = -(kGain * v + q);
    result.body_index = body_index_;
    result.p_BoBq_B = Vector3d::Zero();
    result.F_Bq_W = SpatialForce<double>(/* tau = */ Vector3d::Zero(),
                                         /* f = */ f_x * Vector3d::UnitX());
  }

  const BodyIndex body_index_;
};

// Run tests of the plant's applied_spatial_force input port.
TEST_F(IcfExternalSystemsLinearizerTest, ExternalSpatialForce) {
  const RigidBody<double>& ball =
      plant_.AddRigidBody("ball", default_model_instance(),
                          SpatialInertia<double>::SolidSphereWithMass(
                              /* mass = */ 1.0, /* radius = */ 0.01));
  plant_.AddJoint<PrismaticJoint>("prismatic", plant_.world_body(), {}, ball,
                                  {}, Vector3d::UnitX());

  plant_.Finalize();
  auto controller = builder_->AddSystem<SpatialForceFeedback>(ball.index());
  builder_->Connect(plant_.get_state_output_port(),
                    controller->get_input_port());
  builder_->Connect(controller->get_output_port(),
                    plant_.get_applied_spatial_force_input_port());
  Build();

  // Set an interesting initial state.
  const double q = 0.1;
  const double v = 0.2;
  plant_.SetPositions(plant_context_, Vector1d{q});
  plant_.SetVelocities(plant_context_, Vector1d{v});

  // Linearize the non-plant dynamics around the current state.
  const double h = 0.01;
  const auto result = LinearizeExternalSystem(h);
  ASSERT_TRUE(result.external_feedback.has_value());
  EXPECT_FALSE(result.actuation_feedback.has_value());
  const VectorXd& K = result.external_feedback->K;
  const VectorXd& b = result.external_feedback->b;

  // Check that the feedback was linearized as expected.
  const Vector1d K_expected{SpatialForceFeedback::kGain};
  const Vector1d b_expected{-q};
  EXPECT_TRUE(CompareMatrices(K, K_expected, 1e-2));
  EXPECT_TRUE(CompareMatrices(b, b_expected, 1e-6));
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
