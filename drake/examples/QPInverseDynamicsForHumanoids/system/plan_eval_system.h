#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A simple PlanEval block that generates qp input for the qp inverse
 * dynamics controller.
 * The controller assume the robot is in double support, and the desired set
 * point is set by SetDesired.
 *
 * Input: HumanoidStatus
 * Output: QPInput
 */
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot) : robot_(robot) {
    input_port_index_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    output_port_index_qp_input_ =
        DeclareAbstractOutputPort(systems::kInheritedSampling).get_index();

    set_name("plan_eval");

    // TODO(siyuan.feng): Move these to some param / config file eventually.
    // Set up gains.
    int dim = robot_.get_num_positions();
    Kp_com_ = Eigen::Vector3d::Constant(40);
    Kd_com_ = Eigen::Vector3d::Constant(12);
    Kp_pelvis_ = Eigen::Vector6d::Constant(20);
    Kd_pelvis_ = Eigen::Vector6d::Constant(8);
    Kp_torso_ = Eigen::Vector6d::Constant(20);
    Kd_torso_ = Eigen::Vector6d::Constant(8);
    Kp_joints_ = Eigen::VectorXd::Constant(dim, 20);
    Kd_joints_ = Eigen::VectorXd::Constant(dim, 8);
    // Don't do feedback on pelvis pose.
    Kp_joints_.head<6>().setZero();
    Kd_joints_.head<6>().setZero();
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // Input:
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // Output:
    QPInput& result = output->GetMutableData(output_port_index_qp_input_)
                          ->GetMutableValue<QPInput>();

    // Update weights.
    for (const std::string& joint_name : robot_status->arm_joint_names()) {
      int idx = robot_status->name_to_position_index().at(joint_name);
      result.mutable_desired_joint_motions().mutable_weight(idx) = -1;
      result.mutable_desired_joint_motions().mutable_constraint_type(idx) =
          ConstraintType::Hard;
    }
    for (const std::string& joint_name : robot_status->neck_joint_names()) {
      int idx = robot_status->name_to_position_index().at(joint_name);
      result.mutable_desired_joint_motions().mutable_weight(idx) = -1;
      result.mutable_desired_joint_motions().mutable_constraint_type(idx) =
          ConstraintType::Hard;
    }

    // Update desired accelerations.
    result.mutable_desired_centroidal_momentum_dot()
        .mutable_values()
        .tail<3>() =
        (Kp_com_.array() * (desired_com_ - robot_status->com()).array() -
         Kd_com_.array() * robot_status->comd().array()).matrix() *
        robot_.getMass();

    result.mutable_desired_joint_motions().mutable_values() =
        joint_PDff_.ComputeTargetAcceleration(robot_status->position(),
                                              robot_status->velocity());
    result.mutable_desired_body_motions().at("pelvis").mutable_values() =
        pelvis_PDff_.ComputeTargetAcceleration(
            robot_status->pelvis().pose(), robot_status->pelvis().velocity());
    result.mutable_desired_body_motions().at("torso").mutable_values() =
        torso_PDff_.ComputeTargetAcceleration(robot_status->torso().pose(),
                                              robot_status->torso().velocity());
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<QPInput>(MakeExampleQPInput(robot_))));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    pelvis_PDff_ = CartesianSetpoint<double>(
        robot_status.pelvis().pose(), Eigen::Vector6d::Zero(),
        Eigen::Vector6d::Zero(), Kp_pelvis_, Kd_pelvis_);
    torso_PDff_ = CartesianSetpoint<double>(
        robot_status.torso().pose(), Eigen::Vector6d::Zero(),
        Eigen::Vector6d::Zero(), Kp_torso_, Kd_torso_);
    int dim = robot_status.position().size();
    joint_PDff_ = VectorSetpoint<double>(
        robot_status.position(), Eigen::VectorXd::Zero(dim),
        Eigen::VectorXd::Zero(dim), Kp_joints_, Kd_joints_);
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;

  int input_port_index_humanoid_status_;
  int output_port_index_qp_input_;

  // Gains and setpoints.
  VectorSetpoint<double> joint_PDff_;
  CartesianSetpoint<double> pelvis_PDff_;
  CartesianSetpoint<double> torso_PDff_;

  Eigen::Vector3d desired_com_;
  Eigen::Vector3d Kp_com_;
  Eigen::Vector3d Kd_com_;

  Eigen::Vector6d Kp_pelvis_;
  Eigen::Vector6d Kd_pelvis_;
  Eigen::Vector6d Kp_torso_;
  Eigen::Vector6d Kd_torso_;
  Eigen::VectorXd Kp_joints_;
  Eigen::VectorXd Kd_joints_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
