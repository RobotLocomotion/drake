#pragma once

#include "drake/systems/framework/leaf_system.h"

#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"

namespace drake {
namespace systems {

class System2PlanEval : public LeafSystem<double> {
 public:
  explicit System2PlanEval(const RigidBodyTree &robot) : robot_(robot) {
    DeclareAbstractInputPort(kInheritedSampling);
    DeclareAbstractOutputPort(kInheritedSampling);

    set_name("plan_eval");

    // Setup gains
    Kp_com_ = Eigen::Vector3d::Constant(40);
    Kd_com_ = Eigen::Vector3d::Constant(12);
    Kp_joints_ = Eigen::VectorXd::Constant(robot.get_num_positions(), 20);
    Kd_joints_ = Eigen::VectorXd::Constant(robot.get_num_velocities(), 8);
   
    desired_pelvis_.mutable_Kp() = Eigen::Vector6d::Constant(20);
    desired_pelvis_.mutable_Kd() = Eigen::Vector6d::Constant(8);
    desired_torso_.mutable_Kp() = Eigen::Vector6d::Constant(20);
    desired_torso_.mutable_Kd() = Eigen::Vector6d::Constant(8);
  }

  void SetupDesired(const example::qp_inverse_dynamics::HumanoidStatus &rs) {
    desired_com_ = rs.com();
    desired_q_ = rs.position();
    desired_pelvis_.mutable_desired_pose() = rs.pelvis().pose();
    desired_torso_.mutable_desired_pose() = rs.torso().pose();
  }

  void EvalOutput(const Context<double> &context, SystemOutput<double>* output) const override {
    // input: humanoid status
    const example::qp_inverse_dynamics::HumanoidStatus *robot_status = EvalInputValue<example::qp_inverse_dynamics::HumanoidStatus>(context, 0);
  
    // output: qp input
    example::qp_inverse_dynamics::QPInput& input = output->GetMutableData(0)->GetMutableValue<example::qp_inverse_dynamics::QPInput>();
    
    input.mutable_desired_comdd() =
      (Kp_com_.array() * (desired_com_ - robot_status->com()).array() -
       Kd_com_.array() * robot_status->comd().array())
      .matrix();
    input.mutable_w_com() = 1e3;

    // Minimize acceleration in the generalized coordinates.
    input.mutable_desired_vd() =
      (Kp_joints_.array() * (desired_q_ - robot_status->position()).array() -
       Kd_joints_.array() * robot_status->velocity().array())
      .matrix();
    input.mutable_w_vd() = 1;

    // Setup tracking for various body parts.
    input.mutable_desired_body_accelerations().clear();
    
    example::qp_inverse_dynamics::DesiredBodyAcceleration pelvdd_d(*robot_status->robot().FindBody("pelvis"));
    pelvdd_d.mutable_weight() = 1e1;
    pelvdd_d.mutable_acceleration() = desired_pelvis_.ComputeTargetAcceleration(
        robot_status->pelvis().pose(), robot_status->pelvis().velocity());
    input.mutable_desired_body_accelerations().push_back(pelvdd_d);

    example::qp_inverse_dynamics::DesiredBodyAcceleration torsodd_d(*robot_status->robot().FindBody("torso"));
    torsodd_d.mutable_weight() = 1e1;
    torsodd_d.mutable_acceleration() = desired_torso_.ComputeTargetAcceleration(
        robot_status->torso().pose(), robot_status->torso().velocity());
    input.mutable_desired_body_accelerations().push_back(torsodd_d);

    // Weights are set arbitrarily by the control designer, these typically
    // require tuning.
    input.mutable_w_basis_reg() = 1e-6;

    // Make contact points.
    input.mutable_contact_info().clear();

    example::qp_inverse_dynamics::ContactInformation left_foot_contact(
        *robot_status->robot().FindBody("leftFoot"), 4);
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(0.2, 0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(0.2, -0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(-0.05, -0.05, -0.09));
    left_foot_contact.mutable_contact_points().push_back(
        Eigen::Vector3d(-0.05, 0.05, -0.09));

    example::qp_inverse_dynamics::ContactInformation right_foot_contact(
        *robot_status->robot().FindBody("rightFoot"), 4);
    right_foot_contact.mutable_contact_points() =
      left_foot_contact.contact_points();

    input.mutable_contact_info().push_back(left_foot_contact);
    input.mutable_contact_info().push_back(right_foot_contact); 
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(const Context<double>& context) const {
    std::unique_ptr<LeafSystemOutput<double>> output(new LeafSystemOutput<double>);
    example::qp_inverse_dynamics::QPInput qpinput(robot_);
    output->add_port(std::unique_ptr<AbstractValue>(new Value<example::qp_inverse_dynamics::QPInput>(qpinput)));
    return std::unique_ptr<SystemOutput<double>>(output.release()); 
  }

 private:
  example::qp_inverse_dynamics::CartesianSetPoint desired_pelvis_;
  example::qp_inverse_dynamics::CartesianSetPoint desired_torso_;
  Eigen::Vector3d desired_com_;
  Eigen::VectorXd desired_q_;

  Eigen::Vector3d Kp_com_;
  Eigen::Vector3d Kd_com_;
  Eigen::VectorXd Kp_joints_;
  Eigen::VectorXd Kd_joints_;

  const RigidBodyTree &robot_;
};

}
}
