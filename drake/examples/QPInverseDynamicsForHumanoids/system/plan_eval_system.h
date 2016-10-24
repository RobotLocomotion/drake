#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_balancing_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  /**
   * A simple PlanEval block that generates qp input for the qp inverse
   * dynamics controller.
   * The controller assume the robot is in double support, and the desired set
   * point is set by SetupDesired.
   * Input: humanoid status
   * Output: qp input
   */
  explicit PlanEvalSystem(const RigidBodyTree& robot) : robot_(robot) {
    input_port_index_humanoid_status_ =
        DeclareAbstractInputPort(systems::kInheritedSampling).get_index();
    output_port_index_qp_input_ =
        DeclareAbstractOutputPort(systems::kInheritedSampling).get_index();

    set_name("plan_eval");

    // TODO(siyuan.feng@tri.gloabl): move these to some param / config file
    // eventually.
    // Setup gains.
    int dim = robot.get_num_positions();
    Kp_com_ = Eigen::Vector3d::Constant(40);
    Kd_com_ = Eigen::Vector3d::Constant(12);
    Kp_pelvis_ = Eigen::Vector6d::Constant(20);
    Kd_pelvis_ = Eigen::Vector6d::Constant(8);
    Kp_torso_ = Eigen::Vector6d::Constant(20);
    Kd_torso_ = Eigen::Vector6d::Constant(8);
    Kp_joints_ = Eigen::VectorXd::Constant(dim, 20);
    Kd_joints_ = Eigen::VectorXd::Constant(dim, 8);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // input: humanoid status
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // output: qp input
    QPInput& result = output->GetMutableData(output_port_index_qp_input_)
                         ->GetMutableValue<QPInput>();

    for (const std::string& joint_name : robot_status->arm_joint_names()) {
      int idx = robot_status->name_to_position_index().at(joint_name);
      result.mutable_desired_joint_motions().mutable_weights()[idx] = -1;
    }
    for (const std::string& joint_name : robot_status->neck_joint_names()) {
      int idx = robot_status->name_to_position_index().at(joint_name);
      result.mutable_desired_joint_motions().mutable_weights()[idx] = -1;
    }

    // Update desired accelerations.
    result.mutable_desired_comdd() =
        (Kp_com_.array() * (desired_com_ - robot_status->com()).array() -
         Kd_com_.array() * robot_status->comd().array()).matrix();

    result.mutable_desired_joint_motions().mutable_accelerations() = joint_PDff_.ComputeTargetAcceleration(robot_status->position(), robot_status->velocity());
    result.mutable_desired_body_motions().at("pelvis").mutable_accelerations() = pelvis_PDff_.ComputeTargetAcceleration(robot_status->pelvis().pose(), robot_status->pelvis().velocity());
    result.mutable_desired_body_motions().at("torso").mutable_accelerations() = torso_PDff_.ComputeTargetAcceleration(robot_status->torso().pose(), robot_status->torso().velocity());
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<QPInput>(MakeExampleQPInput(robot_))));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetupDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    pelvis_PDff_ = CartesianSetpoint(robot_status.pelvis().pose(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), Kp_pelvis_, Kd_pelvis_);
    torso_PDff_ = CartesianSetpoint(robot_status.torso().pose(), Eigen::Vector6d::Zero(), Eigen::Vector6d::Zero(), Kp_torso_, Kd_torso_);
    int dim = robot_status.position().size();
    joint_PDff_ = VectorSetpoint(robot_status.position(), Eigen::VectorXd::Zero(dim), Eigen::VectorXd::Zero(dim), Kp_joints_, Kd_joints_);
  }

  /**
   * @return the input port number that corresponds to: humanoid status.
   */
  inline const SystemPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return the output port number that corresponds to: qp input.
   */
  inline const SystemPortDescriptor<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree& robot_;

  int input_port_index_humanoid_status_;
  int output_port_index_qp_input_;

  VectorSetpoint joint_PDff_;
  CartesianSetpoint pelvis_PDff_;
  CartesianSetpoint torso_PDff_;

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
