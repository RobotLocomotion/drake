#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/qp_controller.h"
#include "drake/lcmt_inverse_dynamics_debug_info.hpp"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * A discrete time system block for an inverse dynamics controller.
 */
class QpControllerSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QpControllerSystem)

  /**
   * Constructor for the inverse dynamics controller.
   * @param robot Reference to a RigidBodyTree. Its lifespan must be longer
   * than this object.
   * @param dt Control cycle period.
   */
  QpControllerSystem(const RigidBodyTree<double>& robot, double dt);

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
     const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& events,
     systems::State<double>* state) const override;

  /**
   * Returns the input port for HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * Returns the input port for QpInput.
   */
  inline const systems::InputPortDescriptor<double>& get_input_port_qp_input()
      const {
    return get_input_port(input_port_index_qp_input_);
  }

  /**
   * Returns the output port for QpOutput.
   */
  inline const systems::OutputPort<double>&
  get_output_port_qp_output() const {
    return get_output_port(output_port_index_qp_output_);
  }

  /**
   * Returns the output port for lcmt_inverse_dynamics_debug_info.
   */
  inline const systems::OutputPort<double>&
  get_output_port_debug_info() const {
    return get_output_port(output_port_index_debug_info_);
  }

  inline double get_control_dt() const { return control_dt_; }

 private:
  // Copies the QpOutput state variable to the output argument.
  void CopyOutQpOutput(const systems::Context<double>& context,
                       QpOutput* output) const;

  // Copies the debug info state variable to the output argument.
  void CopyOutDebugInfo(const systems::Context<double>& context,
                        lcmt_inverse_dynamics_debug_info* output) const;

  const RigidBodyTree<double>& robot_;
  const double control_dt_{};

  // TODO(siyuan.feng): This is a bad temporary hack to the const constraint for
  // CalcOutput. It is because qp controller needs to allocate mutable workspace
  // (MathematicalProgram, temporary matrices for doing math, etc),
  // and I want to avoid allocating these repeatedly.
  // This should be taken care of with the new system2 cache.
  mutable QPController qp_controller_;

  int input_port_index_humanoid_status_{0};
  int input_port_index_qp_input_{0};
  int output_port_index_qp_output_{0};
  int output_port_index_debug_info_{0};

  int abs_state_index_qp_output_{0};
  int abs_state_index_debug_info_{0};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
