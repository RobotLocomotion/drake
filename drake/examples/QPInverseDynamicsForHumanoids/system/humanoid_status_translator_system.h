#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * An abstract base class for systems that translate various types of state
 * into HumanoidStatus.
 */
class HumanoidStatusTranslatorSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HumanoidStatusTranslatorSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree. The lifespan of @p robot must
   * be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used to construct
   * HumanoidStatus.
   */
  HumanoidStatusTranslatorSystem(const RigidBodyTree<double>& robot,
                                 const std::string& alias_group_path);

  /**
   * Returns the output port for HumanoidStatus.
   */
  const systems::OutputPort<double>&
  get_output_port_humanoid_status() const {
    return get_output_port(output_port_index_humanoid_status_);
  }

 protected:
  /**
   * Derived classes should use this method for the output port allocator.
   */
  HumanoidStatus MakeHumanoidStatus() const;

  /**
   * Derived classes should use this to record the output port index.
   */
  void set_output_port_index_humanoid_status(systems::OutputPortIndex index) {
    output_port_index_humanoid_status_ = index;
  }

  const RigidBodyTree<double>& get_robot() const { return robot_; }

 private:
  const RigidBodyTree<double>& robot_;
  const std::string alias_group_path_;

  int output_port_index_humanoid_status_{0};
};

/**
 * Translates a state vector to HumanoidStatus.
 */
class StateToHumanoidStatusSystem : public HumanoidStatusTranslatorSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateToHumanoidStatusSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree. The lifespan of @p robot must
   * be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used to construct
   * HumanoidStatus.
   */
  StateToHumanoidStatusSystem(const RigidBodyTree<double>& robot,
                              const std::string& alias_group_path);


  /**
   * Returns the input port for a state vector.
   */
  const systems::InputPortDescriptor<double>& get_input_port_state()
      const {
    return get_input_port(input_port_index_state_);
  }

 private:
  // This is the calculator for the output port.
  void CalcHumanoidStatus(const systems::Context<double>& context,
                          HumanoidStatus* output) const;

  int input_port_index_state_{0};
};

/**
 * Translates a bot_core::robot_state_t message to HumanoidStatus.
 */
class RobotStateMsgToHumanoidStatusSystem
    : public HumanoidStatusTranslatorSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotStateMsgToHumanoidStatusSystem)

  /**
   * Constructor.
   * @param robot Reference to a RigidBodyTree. The lifespan of @p robot
   * must be longer than this object.
   * @param alias_group_path Path to the alias groups file. Used to construct
   * HumanoidStatus.
   */
  RobotStateMsgToHumanoidStatusSystem(const RigidBodyTree<double>& robot,
                                      const std::string& alias_group_path);
  /**
   * Returns input port for bot_core::robot_state_t.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_robot_state_msg() const {
    return get_input_port(input_port_index_lcm_msg_);
  }

 private:
  // This is the calculator for the output port.
  void CalcHumanoidStatus(const systems::Context<double>& context,
                          HumanoidStatus* output) const;

  const manipulation::RobotStateLcmMessageTranslator translator_;
  int input_port_index_lcm_msg_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
