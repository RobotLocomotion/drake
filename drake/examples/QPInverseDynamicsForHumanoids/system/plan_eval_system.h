#pragma once

#include <string>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/example_qp_input_for_valkyrie.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// TODO(siyuan.feng): Extend this class properly to support various different
// plans. This class currently only supports tracking a stationary fixed point.

/**
 * A simple PlanEval block that generates qp input for the qp inverse dynamics
 * controller.
 * The controller is set up to track a stationary fixed point assuming the
 * robot is in double support, and the desired set point is set by SetDesired.
 *
 * Input: HumanoidStatus
 * Output: QPInput
 */
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot) : robot_(robot) {
    input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
    output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

    set_name("plan_eval");

    // TODO(siyuan.feng): Move these to some param / config file eventually.
    // Set up gains.
    int dim = robot_.get_num_positions();
    Kp_com_ = Vector3<double>::Constant(40);
    Kd_com_ = Vector3<double>::Constant(12);
    Kp_pelvis_ = Vector6<double>::Constant(20);
    Kd_pelvis_ = Vector6<double>::Constant(8);
    Kp_torso_ = Vector6<double>::Constant(20);
    Kd_torso_ = Vector6<double>::Constant(8);
    Kp_joints_ = VectorX<double>::Constant(dim, 20);
    Kd_joints_ = VectorX<double>::Constant(dim, 8);
    // Don't do feedback on pelvis in the generalized coordinates.
    Kp_joints_.head<6>().setZero();
    Kd_joints_.head<6>().setZero();
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    // Input:
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // Output:
    lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
                             ->GetMutableValue<lcmt_qp_input>();

    Vector3<double> com_err = desired_com_ - robot_status->com();
    Vector3<double> comd_err = -robot_status->comd();

    // Update desired accelerations.
    QPInput qp_input = MakeExampleQPInput(*robot_status);
    qp_input.mutable_desired_centroidal_momentum_dot()
        .mutable_values().tail<3>() = robot_.getMass() *
        (Kp_com_.array() * com_err.array() +
         Kd_com_.array() * comd_err.array()).matrix();

    qp_input.mutable_desired_dof_motions().mutable_values() =
        joint_PDff_.ComputeTargetAcceleration(robot_status->position(),
                                              robot_status->velocity());
    qp_input.mutable_desired_body_motions().at("pelvis").mutable_values() =
        pelvis_PDff_.ComputeTargetAcceleration(
            robot_status->pelvis().pose(), robot_status->pelvis().velocity());
    qp_input.mutable_desired_body_motions().at("torso").mutable_values() =
        torso_PDff_.ComputeTargetAcceleration(robot_status->torso().pose(),
                                              robot_status->torso().velocity());

    // Encode and send.
    EncodeQPInput(qp_input, &msg);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    std::unique_ptr<LeafSystemOutput<double>> output(
        new LeafSystemOutput<double>);
    output->add_port(std::unique_ptr<AbstractValue>(
        new Value<lcmt_qp_input>(lcmt_qp_input())));
    return std::move(output);
  }

  /**
   * Set the set point for tracking.
   * @param robot_status, desired robot state
   */
  void SetDesired(const HumanoidStatus& robot_status) {
    desired_com_ = robot_status.com();
    pelvis_PDff_ = CartesianSetpoint<double>(
        robot_status.pelvis().pose(), Vector6<double>::Zero(),
        Vector6<double>::Zero(), Kp_pelvis_, Kd_pelvis_);
    torso_PDff_ = CartesianSetpoint<double>(
        robot_status.torso().pose(), Vector6<double>::Zero(),
        Vector6<double>::Zero(), Kp_torso_, Kd_torso_);
    int dim = robot_status.position().size();
    joint_PDff_ = VectorSetpoint<double>(
        robot_status.position(), VectorX<double>::Zero(dim),
        VectorX<double>::Zero(dim), Kp_joints_, Kd_joints_);
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

  Vector3<double> desired_com_;
  Vector3<double> Kp_com_;
  Vector3<double> Kd_com_;

  Vector6<double> Kp_pelvis_;
  Vector6<double> Kd_pelvis_;
  Vector6<double> Kp_torso_;
  Vector6<double> Kd_torso_;
  VectorX<double> Kp_joints_;
  VectorX<double> Kd_joints_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
