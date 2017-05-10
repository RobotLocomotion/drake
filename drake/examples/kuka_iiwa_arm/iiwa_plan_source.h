#pragma once

#include <memory>
#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// This class implements a source of joint positions for an iiwa arm.
/// It has two input ports, one for robot_plan_t messages containing a
/// plan to follow, and another vector-valued port which expects the
/// current (q,v) state of the iiwa arm.
///
/// The system has two output ports, one with the current desired
/// state (q,v) of the iiwa arm and another for the accelerations.
///
/// If a plan is received with no knot points, the system will create
/// a plan which commands the arm to hold at the measured position.
class IiwaPlanSource : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IiwaPlanSource)

  IiwaPlanSource(const std::string& model_path,
                 double update_interval = kDefaultPlanUpdateInterval);
  ~IiwaPlanSource() override;

  const systems::InputPortDescriptor<double>& get_plan_input_port() const {
    return this->get_input_port(plan_input_port_);
  }

  const systems::InputPortDescriptor<double>& get_state_input_port() const {
    return this->get_input_port(state_input_port_);
  }

  const systems::OutputPortDescriptor<double>&
  get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPortDescriptor<double>&
  get_acceleration_output_port() const {
    return this->get_output_port(acceleration_output_port_);
  }

  /**
   * Makes a plan to hold at the measured joint configuration @p q0 starting at
   * @p plan_start_time. This function needs to be explicitly called before any
   * simulation. Otherwise this aborts in CalcOutput().
   */
  void Initialize(double plan_start_time, const VectorX<double>& q0,
                  systems::State<double>* state) const;

  const RigidBodyTree<double>& tree() { return tree_; }

 protected:
  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  std::unique_ptr<systems::AbstractValues> AllocateAbstractState()
      const override;

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
     const std::vector<const systems::UnrestrictedUpdateEvent<double>*>& events,
     systems::State<double>* state) const override;

 private:
  struct PlanData;

  void MakeFixedPlan(double plan_start_time, const VectorX<double>& q0,
                  systems::State<double>* state) const;

  static constexpr double kDefaultPlanUpdateInterval = 0.1;
  const int plan_input_port_{};
  const int state_input_port_{};
  const int state_output_port_{};
  const int acceleration_output_port_{};
  RigidBodyTree<double> tree_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
