#include "drake/examples/qp_inverse_dynamics/manipulator_joint_space_controller.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/setpoint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace {

using systems::controllers::VectorSetpoint;
using systems::controllers::qp_inverse_dynamics::ConstraintType;
using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::qp_inverse_dynamics::QpInput;

// Builds a test diagram that gives a ManipulatorJointSpaceController and
// a systems::InverseDynamicsController the exact same inputs (estimated state,
// desired state and desired acceleration), and expects their outputs (torque)
// to be the same assuming the computed torques are within the torque limits.
class ManipulatorJointSpaceControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string kModelPath = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/urdf/"
        "iiwa14_polytope_collision.urdf");

    const std::string kAliasGroupsPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/test/"
        "iiwa.alias_groups");

    const std::string kControlConfigPath = FindResourceOrThrow(
        "drake/systems/controllers/qp_inverse_dynamics/test/"
        "iiwa.id_controller_config");

    auto robot = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        kModelPath, multibody::joints::kFixed, robot.get());

    // Esimated state.
    VectorX<double> q(7);
    VectorX<double> v(7);
    VectorX<double> x(14);
    q << -1, 1, -0.4, 0.7, 0.99, -0.4, 0.33;
    v << -0.3, 0.8, 2, 1, -0.55, 0.06, 0;
    x << q, v;

    // Desired state.
    VectorX<double> q_r(7);
    VectorX<double> v_r(7);
    VectorX<double> vd_r(7);
    VectorX<double> x_r(14);
    q_r << -1.3, 1.2, -0.45, 0.3, 0.6, -0.1, 0;
    v_r << -1, 1, 2.3, 1.2, -0.8, 0, 0.1;
    vd_r << 1, 2, 0, 3, 4, -3, -3;
    x_r << q_r, v_r;

    // Makes a diagram for testing.
    systems::DiagramBuilder<double> builder;
    auto qp_id_controller =
        builder.AddSystem<ManipulatorJointSpaceController>(
            kModelPath, kAliasGroupsPath, kControlConfigPath, 0.02);
    params_ = &qp_id_controller->get_paramset();

    // Use the same kp kd gains from qp_id_controller.
    VectorX<double> kp, kd;
    params_->LookupDesiredDofMotionGains(&kp, &kd);
    auto vanilla_id_controller =
        builder.AddSystem<systems::controllers::InverseDynamicsController>(
            std::move(robot), kp, VectorX<double>::Zero(7), kd, true);

    // Estimated state source.
    auto estimated_state_source =
        builder.AddSystem<systems::ConstantVectorSource<double>>(x);

    // Desired state source.
    auto desired_state_source =
        builder.AddSystem<systems::ConstantVectorSource<double>>(x_r);
    auto desired_acceleration_source =
        builder.AddSystem<systems::ConstantVectorSource<double>>(vd_r);

    // Estimated state -> controllers.
    builder.Connect(estimated_state_source->get_output_port(),
                    qp_id_controller->get_input_port_estimated_state());

    builder.Connect(estimated_state_source->get_output_port(),
                    vanilla_id_controller->get_input_port_estimated_state());

    // Desired -> controllers
    builder.Connect(desired_state_source->get_output_port(),
                    qp_id_controller->get_input_port_desired_state());
    builder.Connect(desired_acceleration_source->get_output_port(),
                    qp_id_controller->get_input_port_desired_acceleration());

    builder.Connect(desired_state_source->get_output_port(),
                    vanilla_id_controller->get_input_port_desired_state());
    builder.Connect(
        desired_acceleration_source->get_output_port(),
        vanilla_id_controller->get_input_port_desired_acceleration());

    // Expose the output ports for QpInput and QpOutput.
    plan_eval_output_index_ =
        builder.ExportOutput(qp_id_controller->get_output_port_qp_input());
    qp_controller_output_index_ =
        builder.ExportOutput(qp_id_controller->get_output_port_control());
    vanilla_controller_output_index_ =
        builder.ExportOutput(vanilla_id_controller->get_output_port_control());

    diagram_ = builder.Build();

    context_ = diagram_->CreateDefaultContext();
    context_->set_time(0);
    output_ = diagram_->AllocateOutput(*context_);

    // Initializes.
    qp_id_controller->Initialize(
        &diagram_->GetMutableSubsystemContext(
            *qp_id_controller, context_.get()));

    // Computes results.
    auto events = diagram_->AllocateCompositeEventCollection();
    diagram_->CalcNextUpdateTime(*context_, events.get());

    std::unique_ptr<systems::State<double>> state = context_->CloneState();

    // Generates QpInput from the plan eval block within
    // ManipulatorJointSpaceController.
    diagram_->CalcUnrestrictedUpdate(
        *context_, events->get_unrestricted_update_events(), state.get());
    context_->get_mutable_state().CopyFrom(*state);

    // Generates QpOuput from the inverse dynamics block within
    // ManipulatorJointSpaceController.
    diagram_->CalcUnrestrictedUpdate(
         *context_, events->get_unrestricted_update_events(), state.get());
    context_->get_mutable_state().CopyFrom(*state);

    // Gets output.
    diagram_->CalcOutput(*context_, output_.get());

    // Computes expected acceleration input:
    // vd_d = kp * (q_r - q) + kd * (v_r - v) + vd_r
    VectorSetpoint<double> tracker(q_r, v_r, vd_r, kp, kd);
    expected_vd_d_ = tracker.ComputeTargetAcceleration(q, v);
  }

  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<systems::Context<double>> context_{nullptr};
  std::unique_ptr<systems::SystemOutput<double>> output_{nullptr};

  const ParamSet* params_;

  VectorX<double> expected_vd_d_;

  int plan_eval_output_index_{};
  int qp_controller_output_index_{};
  int vanilla_controller_output_index_{};
};

// Tests QpInput from the plan eval component. The desired generalized
// acceleration should equal to expected_vd_d_, and various modes and weights
// should be specified by params_ loaded from kControlConfigPath.
TEST_F(ManipulatorJointSpaceControllerTest, PlanEvalTest) {
  const QpInput& qp_input =
      output_->get_data(plan_eval_output_index_)->GetValue<QpInput>();

  // Desired generalized acceleration should match expected.
  EXPECT_TRUE(drake::CompareMatrices(
      expected_vd_d_, qp_input.desired_dof_motions().values(), 1e-12,
      drake::MatrixCompareType::absolute));
  VectorX<double> expected_weights = params_->MakeDesiredDofMotions().weights();
  EXPECT_TRUE(drake::CompareMatrices(
      expected_weights, qp_input.desired_dof_motions().weights(), 1e-12,
      drake::MatrixCompareType::absolute));
  std::vector<ConstraintType> expected_constraint_type =
      params_->MakeDesiredDofMotions().constraint_types();
  for (size_t i = 0; i < expected_constraint_type.size(); ++i) {
    EXPECT_EQ(qp_input.desired_dof_motions().constraint_type(i),
              expected_constraint_type[i]);
  }

  // Contact force basis regularization weight is irrelevant here since there
  // is not contacts, but its value should match params'.
  EXPECT_EQ(qp_input.w_basis_reg(), params_->get_basis_regularization_weight());

  // Not tracking Cartesian motions.
  EXPECT_TRUE(qp_input.desired_body_motions().empty());

  // No contacts.
  EXPECT_TRUE(qp_input.contact_information().empty());

  // Doesn't care about overall center of mass or angular momentum.
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().value(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().weight(i), 0);
    EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().constraint_type(i),
              ConstraintType::Skip);
  }
}

// The torque outputs from ManipulatorJointSpaceController and
// systems::InverseDynamicsController should be the same for this simple case.
TEST_F(ManipulatorJointSpaceControllerTest,
       CompareWithBasicInverseDyanmics) {
  EXPECT_EQ(context_->get_time(), 0);
  VectorX<double> qp_output =
      output_->get_vector_data(qp_controller_output_index_)->CopyToVector();
  VectorX<double> vanilla_output =
      output_->get_vector_data(vanilla_controller_output_index_)
          ->CopyToVector();

  EXPECT_TRUE(drake::CompareMatrices(qp_output, vanilla_output, 1e-8,
                                     drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
