#pragma once

#include <memory>
#include <string>
#include <utility>

#include "bot_core/atlas_command_t.hpp"
#include "bot_core/robot_state_t.hpp"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/examples/QPInverseDynamicsForHumanoids/system/atlas_joint_level_controller_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_plan_eval_system.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/humanoid_status_translator_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

using systems::controllers::qp_inverse_dynamics::QpInverseDynamicsSystem;

/**
 * A controller for humanoid balancing built on top of HumanoidPlanEvalSystem
 * and QpInverseDynamicsSystemSystem. This diagram does not have any input or output ports.
 * The state and plan inputs and control outputs are sent through LCM messages
 * directly.
 */
class ValkyrieController : public systems::Diagram<double> {
 public:
  ValkyrieController(const std::string& model_path,
                     const std::string& control_config_path,
                     const std::string& alias_group_path, lcm::DrakeLcm* lcm) {
    systems::DiagramBuilder<double> builder;

    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        model_path, multibody::joints::kRollPitchYaw, robot_.get());
    RobotStateMsgToHumanoidStatusSystem* msg_to_humanoid_status =
        builder.AddSystem(std::make_unique<RobotStateMsgToHumanoidStatusSystem>(
            robot_.get(), alias_group_path));
    msg_to_humanoid_status->set_name("msg_to_humanoid_status");

    const double kControlDt = 0.003;
    plan_eval_ = builder.AddSystem(std::make_unique<HumanoidPlanEvalSystem>(
        robot_.get(), alias_group_path, control_config_path, kControlDt));
    plan_eval_->set_name("plan_eval");

    QpInverseDynamicsSystem* qp_con = builder.AddSystem(
        std::make_unique<QpInverseDynamicsSystem>(robot_.get(), kControlDt));
    qp_con->set_name("qp_con");

    AtlasJointLevelControllerSystem* joint_con =
        builder.AddSystem<AtlasJointLevelControllerSystem>(*robot_);
    joint_con->set_name("joint_con");

    robot_state_subscriber_ = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<bot_core::robot_state_t>(
            "EST_ROBOT_STATE", lcm));
    robot_state_subscriber_->set_name("robot_state_subscriber");

    auto plan_subscriber = builder.AddSystem(
        systems::lcm::LcmSubscriberSystem::Make<robotlocomotion::robot_plan_t>(
            "VALKYRIE_MANIP_PLAN", lcm));
    plan_subscriber->set_name("plan_subscriber");

    auto atlas_command_publisher = builder.AddSystem(
        systems::lcm::LcmPublisherSystem::Make<bot_core::atlas_command_t>(
            "ROBOT_COMMAND", lcm));
    atlas_command_publisher->set_name("atlas_command_publisher");

    // lcm -> rs
    builder.Connect(robot_state_subscriber_->get_output_port(0),
                    msg_to_humanoid_status->get_input_port_robot_state_msg());
    // rs + plan -> qp_input
    builder.Connect(msg_to_humanoid_status->get_output_port_humanoid_status(),
                    plan_eval_->get_input_port_kinematic_state());
    builder.Connect(plan_subscriber->get_output_port(0),
                    plan_eval_->get_input_port_manip_plan_msg());
    // rs + qp_input -> qp_output
    builder.Connect(msg_to_humanoid_status->get_output_port_humanoid_status(),
                    qp_con->get_input_port_kinematic_state());
    builder.Connect(plan_eval_->get_output_port_qp_input(),
                    qp_con->get_input_port_qp_input());
    // qp_output -> atlas_command_t
    builder.Connect(qp_con->get_output_port_qp_output(),
                    joint_con->get_input_port_qp_output());
    // atlas_command_t -> lcm
    builder.Connect(joint_con->get_output_port_atlas_command(),
                    atlas_command_publisher->get_input_port(0));

    builder.BuildInto(this);
  }

  /**
   * Returns a const reference to the robot state message subscriber system.
   */
  const systems::lcm::LcmSubscriberSystem& get_state_msg_subscriber() const {
    return *robot_state_subscriber_;
  }

  /**
   * Returns a pointer to the plan eval block.
   */
  HumanoidPlanEvalSystem* get_mutable_plan_eval() { return plan_eval_; }

 private:
  systems::lcm::LcmSubscriberSystem* robot_state_subscriber_{nullptr};
  HumanoidPlanEvalSystem* plan_eval_{nullptr};
  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
};

}  // end namespace qp_inverse_dynamics
}  // end namespace examples
}  // end namespace drake
