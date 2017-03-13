#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/Valkyrie/valkyrie_constants.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_value_source.h"

#include "gtest/gtest.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {
namespace {

// Makes a diagram of HumanoidPlanEvalSystem + QpControllerSystem. The
// controller is initialized to track a desired q (a nominal standing pose for
// valkyrie). Output is then evaluated with a measured state also set to the
// same desired p. The expected behavior is that QpInput from
// HumanoidPlanEvalSystem be zero desired accelerations, and the acceleration
// part in QpOutput from QpControllerSystem should be very close to the
// desired accelerations (all zeros) in QpInput.
// Since the inverse dynamics is set up as a minimization problem that has
// many objectives, the results is not going to match the desired input exactly
// unless set as hard constraints.
class HumanoidPlanEvalAndQpInverseDynamicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string kModelPath =
        drake::GetDrakePath() +
        "/examples/Valkyrie/urdf/urdf/"
        "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf";

    const std::string kAliasGroupsPath =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/valkyrie.alias_groups";

    const std::string kControlConfigPath =
        drake::GetDrakePath() +
        "/examples/QPInverseDynamicsForHumanoids/"
        "config/valkyrie.id_controller_config";

    auto robot = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        kModelPath, multibody::joints::kRollPitchYaw, robot.get());

    param_parsers::RigidBodyTreeAliasGroups<double> alias_groups(*robot);
    alias_groups.LoadFromFile(kAliasGroupsPath);

    // Desired state.
    VectorX<double> q =
        valkyrie::RPYValkyrieFixedPointState().head(valkyrie::kRPYValkyrieDof);
    VectorX<double> v =
        valkyrie::RPYValkyrieFixedPointState().tail(valkyrie::kRPYValkyrieDof);

    // Makes a diagram for testing.
    systems::DiagramBuilder<double> builder;
    const double kControlDt = 0.02;
    auto plan_eval = builder.AddSystem<HumanoidPlanEvalSystem>(
        *robot, kAliasGroupsPath, kControlConfigPath, kControlDt);
    auto controller = builder.AddSystem<QpControllerSystem>(*robot, kControlDt);

    // Estimated state source, also set to use the desired q and v.
    HumanoidStatus robot_status(*robot, alias_groups);
    robot_status.UpdateKinematics(0, q, v);
    auto state_source = builder.AddSystem<systems::ConstantValueSource<double>>(
        systems::AbstractValue::Make<HumanoidStatus>(robot_status));

    // State -> qp inverse dynamics.
    builder.Connect(state_source->get_output_port(0),
                    controller->get_input_port_humanoid_status());
    // State -> plan eval.
    builder.Connect(state_source->get_output_port(0),
                    plan_eval->get_input_port_humanoid_status());
    // plan eval -> qp inverse dynamics.
    builder.Connect(plan_eval->get_output_port_qp_input(),
                    controller->get_input_port_qp_input());

    // Expose the output ports for QpInput and QpOutput.
    qp_input_index_ =
        builder.ExportOutput(plan_eval->get_output_port_qp_input());
    qp_output_index_ =
        builder.ExportOutput(controller->get_output_port_qp_output());

    diagram_ = builder.Build();

    context_ = diagram_->CreateDefaultContext();
    context_->set_time(0);
    output_ = diagram_->AllocateOutput(*context_);

    // Initializes.
    auto plan_eval_context =
        diagram_->GetMutableSubsystemContext(context_.get(), plan_eval);
    plan_eval->Initialize(q, plan_eval_context->get_mutable_state());

    // Computes results.
    systems::UpdateActions<double> actions;
    diagram_->CalcNextUpdateTime(*context_, &actions);
    EXPECT_EQ(actions.events.size(), 1);

    std::unique_ptr<systems::State<double>> state = context_->CloneState();
    diagram_->CalcUnrestrictedUpdate(*context_, actions.events.front(),
                                     state.get());
    context_->get_mutable_state()->CopyFrom(*state);
    diagram_->CalcOutput(*context_, output_.get());
  }

  std::unique_ptr<systems::Diagram<double>> diagram_{nullptr};
  std::unique_ptr<systems::Context<double>> context_{nullptr};
  std::unique_ptr<systems::SystemOutput<double>> output_{nullptr};

  int qp_input_index_{};
  int qp_output_index_{};
};

// Since we are computing control for state = desired state, all acceleration
// related inputs should be zero.
TEST_F(HumanoidPlanEvalAndQpInverseDynamicsTest, InputShouldBeZero) {
  EXPECT_EQ(context_->get_time(), 0);
  const QpInput& qp_input =
      output_->get_data(qp_input_index_)->template GetValue<QpInput>();
  EXPECT_EQ(qp_input.desired_dof_motions().values().norm(), 0);
  for (auto const& body_motion_pair : qp_input.desired_body_motions()) {
    EXPECT_EQ(body_motion_pair.second.values().norm(), 0);
  }
  EXPECT_EQ(qp_input.desired_centroidal_momentum_dot().values().norm(), 0);
}

// Compares qp output's acceleration with qp input's. For hard constrained
// dof, the output should match the input. For soft constrained ones, since
// for this configuration of q, we know that zero acceleration can be
// achieved very well (robot standing still), we expect the output to match
// the input very closely. But this is generally not true, depending on
// specific weights, desired accelerations and various constraints.
TEST_F(HumanoidPlanEvalAndQpInverseDynamicsTest, DofAcceleration) {
  const QpInput& qp_input =
      output_->get_data(qp_input_index_)->template GetValue<QpInput>();
  const QpOutput& qp_output =
      output_->get_data(qp_output_index_)->template GetValue<QpOutput>();

  for (int i = 0; i < qp_input.desired_dof_motions().size(); ++i) {
    switch (qp_input.desired_dof_motions().constraint_type(i)) {
      case ConstraintType::Hard:
        EXPECT_NEAR(qp_input.desired_dof_motions().value(i), qp_output.vd()[i],
                    1e-12);
        break;

      case ConstraintType::Soft:
        EXPECT_NEAR(qp_input.desired_dof_motions().value(i), qp_output.vd()[i],
                    5e-4);
        break;

      default:
        break;
    }
  }
}

// We expect the solved contact points' accelerations to be very small.
TEST_F(HumanoidPlanEvalAndQpInverseDynamicsTest, ContactAcceleration) {
  const QpOutput& qp_output =
      output_->get_data(qp_output_index_)->template GetValue<QpOutput>();

  for (const auto& contact_pair : qp_output.resolved_contacts()) {
    EXPECT_TRUE(drake::CompareMatrices(contact_pair.second.body_acceleration(),
                                       Vector6<double>::Zero(), 1e-8,
                                       drake::MatrixCompareType::absolute));
  }
}

// We expect the solved bodies accelerations to track the desired pretty well.
TEST_F(HumanoidPlanEvalAndQpInverseDynamicsTest, BodyAcceleration) {
  const QpInput& qp_input =
      output_->get_data(qp_input_index_)->template GetValue<QpInput>();
  const QpOutput& qp_output =
      output_->get_data(qp_output_index_)->template GetValue<QpOutput>();

  for (const auto& body_motion_pair : qp_output.body_accelerations()) {
    EXPECT_TRUE(drake::CompareMatrices(
        body_motion_pair.second.accelerations(),
        qp_input.desired_body_motions().at(body_motion_pair.first).values(),
        5e-4, drake::MatrixCompareType::absolute));
  }
}

// We expect the solved centroidal momentum dot to track the desired pretty
// well.
TEST_F(HumanoidPlanEvalAndQpInverseDynamicsTest, CentroidalMomentum) {
  const QpInput& qp_input =
      output_->get_data(qp_input_index_)->template GetValue<QpInput>();
  const QpOutput& qp_output =
      output_->get_data(qp_output_index_)->template GetValue<QpOutput>();

  for (int i = 0; i < 6; i++) {
    switch (qp_input.desired_centroidal_momentum_dot().constraint_type(i)) {
      case ConstraintType::Hard:
        EXPECT_NEAR(qp_input.desired_centroidal_momentum_dot().value(i),
                    qp_output.centroidal_momentum_dot()[i], 1e-8);
        break;

      case ConstraintType::Soft:
        EXPECT_NEAR(qp_input.desired_centroidal_momentum_dot().value(i),
                    qp_output.centroidal_momentum_dot()[i], 1e-5);
        break;

      default:
        break;
    }
  }
}

}  // namespace
}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
