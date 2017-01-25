#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/examples/QPInverseDynamicsForHumanoids/control_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/lcm_utils.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/param_parsers/param_parser.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// TODO(siyuan.feng): Extend this class properly to support various different
// plans. This class currently only supports executing a fixed policy.

/**
 * A simple PlanEval block that generates qp input for the qp inverse dynamics
 * controller.
 * The controller moves the robot's pelvis height following a sine wave while
 * holding everything else stationary. It assumes the robot is in double
 * stance, and the stationary setpoint can be set by SetDesired.
 *
 * Input: HumanoidStatus
 * Output: lcmt_qp_input
 */
class PlanEvalSystem : public systems::LeafSystem<double> {
 public:
  explicit PlanEvalSystem(const RigidBodyTree<double>& robot)
      : robot_(robot), alias_groups_(robot) {
    input_port_index_humanoid_status_ = DeclareAbstractInputPort().get_index();
    output_port_index_qp_input_ = DeclareAbstractOutputPort().get_index();

    set_name("plan_eval");

    std::string alias_groups_config =
        drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
                                "config/alias_groups.yaml";
    std::string controller_config =
        drake::GetDrakePath() + "/examples/QPInverseDynamicsForHumanoids/"
                                "config/controller.yaml";
    // KinematicsProperty
    alias_groups_.LoadFromYAMLFile(YAML::LoadFile(alias_groups_config));

    // Controller config
    paramset_.LoadFromYAMLConfigFile(YAML::LoadFile(controller_config),
                                     alias_groups_);

    joint_PDff_ = VectorSetpoint<double>(robot_.get_num_velocities());

    // Load gains.
    Vector6<double> Kp, Kd;
    paramset_.LookupDesiredCentroidalMomentumDotGains(&Kp, &Kd);
    Kp_com_ = Kp.tail<3>();
    Kd_com_ = Kd.tail<3>();

    paramset_.LookupDesiredBodyMotionGains(
        *alias_groups_.get_body_group("pelvis").front(),
        &(pelvis_PDff_.mutable_Kp()), &(pelvis_PDff_.mutable_Kd()));
    paramset_.LookupDesiredBodyMotionGains(
        *alias_groups_.get_body_group("torso").front(),
        &(torso_PDff_.mutable_Kp()), &(torso_PDff_.mutable_Kd()));
    paramset_.LookupDesiredDoFMotionGains(&(joint_PDff_.mutable_Kp()),
                                          &(joint_PDff_.mutable_Kd()));
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    // Input:
    const HumanoidStatus* robot_status = EvalInputValue<HumanoidStatus>(
        context, input_port_index_humanoid_status_);

    // Output:
    lcmt_qp_input& msg = output->GetMutableData(output_port_index_qp_input_)
                             ->GetMutableValue<lcmt_qp_input>();

    // Update desired accelerations.
    QpInput qp_input(GetDofNames(robot_));
    qp_input.mutable_contact_information() =
        paramset_.MakeContactInformation("feet", alias_groups_);

    std::unordered_map<std::string, DesiredBodyMotion> motion_d =
        paramset_.MakeDesiredBodyMotion("pelvis", alias_groups_);
    qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                   motion_d.end());
    motion_d = paramset_.MakeDesiredBodyMotion("torso", alias_groups_);
    qp_input.mutable_desired_body_motions().insert(motion_d.begin(),
                                                   motion_d.end());

    qp_input.mutable_desired_dof_motions() = paramset_.MakeDesiredDofMotions();
    qp_input.mutable_w_basis_reg() =
        paramset_.get_basis_regularization_weight();

    qp_input.mutable_desired_centroidal_momentum_dot() =
        paramset_.MakeDesiredCentroidalMomentumDot();

    // Does acceleration feedback based on the plan.
    Vector3<double> desired_com = initial_com_;
    desired_com[2] += 0.1 * std::sin(2 * M_PI * robot_status->time());
    Vector3<double> com_err = desired_com - robot_status->com();
    Vector3<double> comd_err = -robot_status->comd();

    qp_input.mutable_desired_centroidal_momentum_dot()
        .mutable_values()
        .tail<3>() =
        robot_.getMass() *
        (Kp_com_.array() * com_err.array() + Kd_com_.array() * comd_err.array())
            .matrix();

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
    EncodeQpInput(qp_input, &msg);
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
   * @param q_d Desired generalized position.
   */
  void SetDesired(const VectorX<double>& q_d) {
    KinematicsCache<double> cache = robot_.doKinematics(q_d);

    initial_com_ = robot_.centerOfMass(cache);
    pelvis_PDff_.mutable_desired_pose() = robot_.relativeTransform(
        cache, 0,
        alias_groups_.get_body_group("pelvis").front()->get_body_index());
    torso_PDff_.mutable_desired_pose() = robot_.relativeTransform(
        cache, 0,
        alias_groups_.get_body_group("torso").front()->get_body_index());

    joint_PDff_.mutable_desired_position() = q_d;
  }

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const InputPortDescriptor<double>& get_input_port_humanoid_status()
      const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QpInput.
   */
  inline const OutputPortDescriptor<double>& get_output_port_qp_input() const {
    return get_output_port(output_port_index_qp_input_);
  }

 private:
  const RigidBodyTree<double>& robot_;
  param_parsers::RigidBodyTreeAliasGroups<double> alias_groups_;
  param_parsers::ParamSet paramset_;

  int input_port_index_humanoid_status_;
  int output_port_index_qp_input_;

  // Gains and setpoints.
  VectorSetpoint<double> joint_PDff_;
  CartesianSetpoint<double> pelvis_PDff_;
  CartesianSetpoint<double> torso_PDff_;

  Vector3<double> desired_com_;
  Vector3<double> initial_com_;
  Vector3<double> Kp_com_;
  Vector3<double> Kd_com_;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
