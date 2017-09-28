#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/examples/humanoid_controller/humanoid_status.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace humanoid_controller {

/**
 * Translates a bot_core::robot_state_t message to HumanoidStatus.
 */
class RobotStateMsgToHumanoidStatusSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateMsgToHumanoidStatusSystem)

  /**
   * Constructor.
   * @param robot Pointer to a RigidBodyTree. The lifespan of @p robot
   * must be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used to construct
   * HumanoidStatus.
   */
  RobotStateMsgToHumanoidStatusSystem(const RigidBodyTree<double>* robot,
                                      const std::string& alias_group_path);
  /**
   * Returns input port for bot_core::robot_state_t.
   */
  const systems::InputPortDescriptor<double>& get_input_port() const {
    return System<double>::get_input_port(0);
  }

  /**
   * Returns the output port for HumanoidStatus.
   */
  const systems::OutputPort<double>& get_output_port() const {
    return System<double>::get_output_port(0);
  }

 private:
  // This is the calculator for the output port.
  void CalcHumanoidStatus(
      const systems::Context<double>& context,
      systems::controllers::qp_inverse_dynamics::RobotKinematicState<double>*
          output) const;

  const RigidBodyTree<double>& robot_;
  const manipulation::RobotStateLcmMessageTranslator translator_;
  std::unique_ptr<HumanoidStatus> default_output_;
};

}  // namespace humanoid_controller
}  // namespace examples
}  // namespace drake
