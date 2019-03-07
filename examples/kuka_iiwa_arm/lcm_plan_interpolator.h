#pragma once

#include <string>

#include "drake/manipulation/planner/robot_plan_interpolator.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
class LcmPlanInterpolator : public systems::Diagram<double> {
 public:
  LcmPlanInterpolator(
      const std::string& model_path,
      manipulation::planner::InterpolatorType interpolator_type);

  const systems::InputPort<double>& get_input_port_iiwa_status()
      const {
    return get_input_port(input_port_iiwa_status_);
  }

  const systems::InputPort<double>& get_input_port_iiwa_plan() const {
    return get_input_port(input_port_iiwa_plan_);
  }

  const systems::OutputPort<double>& get_output_port_iiwa_command() const {
    return get_output_port(output_port_iiwa_command_);
  }

  /**
   * Makes a plan to hold at the measured joint configuration @p q0 starting at
   * @p plan_start_time. This function needs to be explicitly called before any
   * simulation. Otherwise this aborts in CalcOutput().
   */
  void Initialize(double plan_start_time, const VectorX<double>& q0,
                  systems::Context<double>* context) const;

  int num_joints() const { return num_joints_; }

 private:
  // Input ports.
  int input_port_iiwa_status_{-1};
  int input_port_iiwa_plan_{-1};

  // Output ports.
  int output_port_iiwa_command_{-1};

  manipulation::planner::RobotPlanInterpolator* robot_plan_interpolator_{};
  int num_joints_{};
};
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
