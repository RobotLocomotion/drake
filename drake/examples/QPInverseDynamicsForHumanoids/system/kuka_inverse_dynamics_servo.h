#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/kuka_servo_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/qp_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/robot_status_wrapper.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Builds a Diagram of a Kuka IIWA arm controlled by inverse dynamics to follow
 * a desired trajectory.
 */
class KukaInverseDynamicsServo : public systems::Diagram<double> {
 public:
  KukaInverseDynamicsServo(
      const std::string& model_path, const std::string& alias_group_path,
      const std::string& controller_config_path,
      std::shared_ptr<RigidBodyFrame<double>> world_offset = nullptr) {
    robot_for_control_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFile(
        model_path, multibody::joints::kFixed, world_offset,
        robot_for_control_.get());
    const RigidBodyTree<double>& robot = *robot_for_control_;

    this->set_name("KukaInverseDynamicsServo");

    systems::DiagramBuilder<double> builder;

    // converter from raw state to humanoid status
    RobotStatusWrapper* rs_wrapper = builder.AddSystem(
        std::make_unique<RobotStatusWrapper>(robot, alias_group_path));
    // converter from qp output to raw torque
    JointLevelControllerSystem* joint_level_controller =
        builder.AddSystem(std::make_unique<JointLevelControllerSystem>(robot));
    // generate qp_input from desired q and qd
    servo_ = builder.AddSystem(std::make_unique<KukaServoSystem>(
        robot, alias_group_path, controller_config_path, 0.002));
    // inverse dynamics
    QPControllerSystem* id_controller =
        builder.AddSystem(std::make_unique<QPControllerSystem>(robot, 0.002));

    // rs -> qp_input
    builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                    servo_->get_input_port_humanoid_status());

    // rs + qp_input -> qp_output
    builder.Connect(rs_wrapper->get_output_port_humanoid_status(),
                    id_controller->get_input_port_humanoid_status());
    builder.Connect(servo_->get_output_port_qp_input(),
                    id_controller->get_input_port_qp_input());

    // qp_output -> joint controller
    builder.Connect(id_controller->get_output_port_qp_output(),
                    joint_level_controller->get_input_port_qp_output());

    // expose arm state input.
    builder.ExportInput(rs_wrapper->get_input_port_state());

    // expose desired q qd input.
    builder.ExportInput(
        servo_->get_input_port_desired_state_and_acceleration());

    // expose arm torque output.
    builder.ExportOutput(joint_level_controller->get_output_port_torque());

    builder.BuildInto(this);
  }

  void Initialize(systems::Context<double>* context) {
    systems::Context<double>* servo_context =
        GetMutableSubsystemContext(context, servo_);
    systems::State<double>* servo_state = servo_context->get_mutable_state();
    servo_->Initialize(servo_state);
  }

  const RigidBodyTree<double>& get_robot_for_control() const {
    return *robot_for_control_;
  }

  inline const systems::InputPortDescriptor<double>&
  get_input_port_measured_state() const {
    return get_input_port(0);
  }

  inline const systems::InputPortDescriptor<double>&
  get_input_port_desired_state_and_acceleration() const {
    return get_input_port(1);
  }

  inline const systems::OutputPortDescriptor<double>& get_output_port_torque()
      const {
    return get_output_port(0);
  }

 private:
  // Maintains a model for the controller, this RBT is just for the IIWA arm.
  std::unique_ptr<RigidBodyTree<double>> robot_for_control_{nullptr};
  KukaServoSystem* servo_{nullptr};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
