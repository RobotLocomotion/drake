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
  explicit PlanEvalSystem(const RigidBodyTree& robot) : robot_(robot), qp_input_(MakeExampleQPInput(robot)) {
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

    result.mutable_desired_comdd() =
        (Kp_com_.array() * (desired_com_ - robot_status->com()).array() -
         Kd_com_.array() * robot_status->comd().array()).matrix();

    result.mutable_desired_joint_motions().mutable_setpoint() = qp_input_.desired_joint_motions().setpoint();

    std::list<DesiredBodyMotion>::const_iterator it = qp_input_.desired_body_motions().begin();
    for (DesiredBodyMotion& body_motion : result.mutable_desired_body_motions()) {
      body_motion.mutable_setpoint() = it->setpoint();
      it++;
    }

    /*
    result.mutable_desired_body_motions().clear();
    result.mutable_contact_info().clear();

    // Weights are set arbitrarily by the control designer, these typically
    // require tuning.
    // Setup tracking for center of mass acceleration.
    result.mutable_w_com() = comdd_weight_;

    // Setup tracking for generalized accelerations.
    int dim = robot_.get_num_velocities();

    result.mutable_desired_joint_motions().mutable_setpoint() = desired_joints_;
    result.mutable_desired_joint_motions().mutable_weights() = -vd_weight_ * Eigen::VectorXd::Constant(dim, 1);

    // Setup tracking for various body parts.
    DesiredBodyMotion pelvdd_d(*robot_status->robot().FindBody("pelvis"));
    pelvdd_d.mutable_weights() = pelvisdd_weight_ * Eigen::Vector6d::Constant(1);
    pelvdd_d.mutable_setpoint() = desired_pelvis_;
    result.mutable_desired_body_motions().push_back(pelvdd_d);

    DesiredBodyMotion torsodd_d(*robot_status->robot().FindBody("torso"));
    torsodd_d.mutable_weights() = torsodd_weight_ * Eigen::Vector6d::Constant(1);
    torsodd_d.mutable_weights().segment<3>(3).setZero();
    //torsodd_d.mutable_weights()[1] = -1;
    torsodd_d.mutable_setpoint() = desired_torso_;
    result.mutable_desired_body_motions().push_back(torsodd_d);

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
    */
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(
        std::unique_ptr<AbstractValue>(new Value<QPInput>(qp_input_)));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetupDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    TrackThis(robot_status, &qp_input_);
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

  QPInput qp_input_;

  Eigen::Vector3d desired_com_;
  Eigen::Vector3d Kp_com_;
  Eigen::Vector3d Kd_com_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
