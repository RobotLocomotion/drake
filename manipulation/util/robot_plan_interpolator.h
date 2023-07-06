#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace util {

/// This enum specifies the type of interpolator to use in constructing
/// the piece-wise polynomial.
enum class InterpolatorType { ZeroOrderHold, FirstOrderHold, Pchip, Cubic };

/// This class implements a source of joint positions for a robot.
/// It has one input port for lcmt_robot_plan messages containing a
/// plan to follow.
///
/// The system has two output ports, one with the current desired
/// state (q,v) of the robot and another for the accelerations.
///
/// @system
/// name: RobotPlanInterpolator
/// input_ports:
/// - plan
/// output_ports:
/// - state
/// - acceleration
/// @endsystem
///
/// If a plan is received with no knot points, the system will create
/// a plan which commands the robot to hold at the measured position.
///
/// @ingroup manipulation_systems
class RobotPlanInterpolator : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotPlanInterpolator)

  RobotPlanInterpolator(const std::string& model_path,
                        const InterpolatorType = InterpolatorType::Cubic,
                        double update_interval = kDefaultPlanUpdateInterval);
  ~RobotPlanInterpolator() override;

  /// N.B. This input port is useless and may be left disconnected.
  const systems::InputPort<double>& get_plan_input_port() const {
    return this->get_input_port(plan_input_port_);
  }

  const systems::OutputPort<double>& get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>& get_acceleration_output_port() const {
    return this->get_output_port(acceleration_output_port_);
  }

  /**
   * Makes a plan to hold at the measured joint configuration @p q0 starting at
   * @p plan_start_time. This function needs to be explicitly called before any
   * simulation. Otherwise this aborts in CalcOutput().
   */
  void Initialize(double plan_start_time, const VectorX<double>& q0,
                  systems::State<double>* state) const;

  /**
   * Updates the plan if there is a new message on the input port.
   * Normally this is done automatically by a Simulator; here we invoke
   * the same periodic event handler that the Simulator would use.
   */
  void UpdatePlan(systems::Context<double>* context) const {
    UpdatePlanOnNewMessage(*context, &context->get_mutable_state());
  }

  const multibody::MultibodyPlant<double>& plant() { return plant_; }

 private:
  struct PlanData;

  // Updates plan when a new message arrives.
  systems::EventStatus UpdatePlanOnNewMessage(
      const systems::Context<double>& context,
      systems::State<double>* state) const;

  // Calculator method for the state output port.
  void OutputState(const systems::Context<double>& context,
                   systems::BasicVector<double>* output) const;

  // Calculator method for the acceleration output port.
  void OutputAccel(const systems::Context<double>& context,
                   systems::BasicVector<double>* output) const;

  void MakeFixedPlan(double plan_start_time, const VectorX<double>& q0,
                     systems::State<double>* state) const;

  static constexpr double kDefaultPlanUpdateInterval = 0.1;
  const int plan_input_port_{};
  int state_output_port_{-1};
  int acceleration_output_port_{-1};
  systems::AbstractStateIndex plan_index_;
  systems::AbstractStateIndex init_flag_index_;
  multibody::MultibodyPlant<double> plant_{0.0};
  const InterpolatorType interp_type_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
