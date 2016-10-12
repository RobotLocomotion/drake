#pragma once

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
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
    Kp_com_ = Eigen::Vector3d::Constant(40);
    Kd_com_ = Eigen::Vector3d::Constant(12);
    Kp_joints_ = Eigen::VectorXd::Constant(robot.get_num_positions(), 20);
    Kd_joints_ = Eigen::VectorXd::Constant(robot.get_num_velocities(), 8);

    desired_pelvis_.mutable_Kp() = Eigen::Vector6d::Constant(20);
    desired_pelvis_.mutable_Kd() = Eigen::Vector6d::Constant(8);
    desired_torso_.mutable_Kp() = Eigen::Vector6d::Constant(20);
    desired_torso_.mutable_Kd() = Eigen::Vector6d::Constant(8);

    // Setup weights.
    comdd_weight_ = 1e3;
    pelvisdd_weight_ = 1e1;
    torsodd_weight_ = 1e1;
    vd_weight_ = 1;
    basis_weight_ = 1e-6;
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    // input: humanoid status
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // output: qp input
    QPInput& result = output->GetMutableData(output_port_index_qp_input_)
                         ->GetMutableValue<QPInput>();
    result.mutable_desired_body_accelerations().clear();
    result.mutable_contact_info().clear();

    // Weights are set arbitrarily by the control designer, these typically
    // require tuning.
    // Setup tracking for center of mass acceleration.
    result.mutable_desired_comdd() =
        (Kp_com_.array() * (desired_com_ - robot_status->com()).array() -
         Kd_com_.array() * robot_status->comd().array()).matrix();
    result.mutable_w_com() = comdd_weight_;

    // Setup tracking for generalized accelerations.
    result.mutable_desired_vd() =
        (Kp_joints_.array() * (desired_q_ - robot_status->position()).array() -
         Kd_joints_.array() * robot_status->velocity().array()).matrix();
    result.mutable_w_vd() = vd_weight_;

    // Setup tracking for various body parts.
    DesiredBodyAcceleration pelvdd_d(*robot_status->robot().FindBody("pelvis"));
    pelvdd_d.mutable_weight() = pelvisdd_weight_;
    pelvdd_d.mutable_acceleration() = desired_pelvis_.ComputeTargetAcceleration(
        robot_status->pelvis().pose(), robot_status->pelvis().velocity());
    result.mutable_desired_body_accelerations().push_back(pelvdd_d);

    DesiredBodyAcceleration torsodd_d(*robot_status->robot().FindBody("torso"));
    torsodd_d.mutable_weight() = torsodd_weight_;
    torsodd_d.mutable_acceleration() = desired_torso_.ComputeTargetAcceleration(
        robot_status->torso().pose(), robot_status->torso().velocity());
    result.mutable_desired_body_accelerations().push_back(torsodd_d);

    // Set weight for the basis regularization term.
    result.mutable_w_basis_reg() = basis_weight_;

    // Make contact points for the left foot.
    ContactInformation left_foot_contact(
        *robot_status->robot().FindBody("leftFoot"), 4);
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(0.2, 0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(0.2, -0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(-0.05, -0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(-0.05, 0.05, -0.09));

    // Mirror the left foot.
    ContactInformation right_foot_contact(
        *robot_status->robot().FindBody("rightFoot"), 4);
    right_foot_contact.mutable_contact_points() =
        left_foot_contact.contact_points();

    result.mutable_contact_info().push_back(left_foot_contact);
    result.mutable_contact_info().push_back(right_foot_contact);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    QPInput qpinput(robot_);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<QPInput>(qpinput)));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetupDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    desired_q_ = robot_status.position();
    desired_pelvis_.mutable_desired_pose() = robot_status.pelvis().pose();
    desired_torso_.mutable_desired_pose() = robot_status.torso().pose();
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

  CartesianSetPoint desired_pelvis_;
  CartesianSetPoint desired_torso_;
  Eigen::Vector3d desired_com_;
  Eigen::VectorXd desired_q_;

  Eigen::Vector3d Kp_com_;
  Eigen::Vector3d Kd_com_;
  Eigen::VectorXd Kp_joints_;
  Eigen::VectorXd Kd_joints_;

  double comdd_weight_;
  double pelvisdd_weight_;
  double torsodd_weight_;
  double vd_weight_;
  double basis_weight_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
