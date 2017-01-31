#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/system/plan_eval_system.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

class KukaPlanEvalSystem : public PlanEvalSystem {
 public:
  KukaPlanEvalSystem(const RigidBodyTree<double>& robot,
                     const std::string& alias_groups_file_name,
                     const std::string& param_file_name)
      : PlanEvalSystem(robot, alias_groups_file_name, param_file_name) {
    set_name("kuka_plan_eval");
  }

  void DoCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
                                systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractState> AllocateAbstractState()
      const override;

  void SetDesiredTrajectory(const PiecewisePolynomialTrajectory& traj,
                            systems::State<double>* state);

  /**
   * @return Port for the input: HumanoidStatus.
   */
  inline const systems::InputPortDescriptor<double>&
  get_input_port_humanoid_status() const {
    return get_input_port(input_port_index_humanoid_status_);
  }

  /**
   * @return Port for the output: QPInput.
   */
  inline const systems::OutputPortDescriptor<double>& get_output_port_qp_input()
      const {
    return get_output_port(output_port_index_qp_input_);
  }
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
