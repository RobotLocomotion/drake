#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace planner {

/// This enum specifies the type of interpolator to use in constructing
/// the piece-wise polynomial.
enum class InterpolatorType {
  ZeroOrderHold,
  FirstOrderHold,
  Pchip,
  Cubic
};

/// This class implements a source of joint positions for a robot.
/// It has one input port for robot_plan_t messages containing a
/// plan to follow.
///
/// The system has two output ports, one with the current desired
/// state (q,v) of the robot and another for the accelerations.
///
/// @system{ RobotPlanInterpolator,
///   @input_port{plan},
///   @output_port{state}
///   @output_port{acceleration}
/// }
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

  const systems::OutputPort<double>&
  get_state_output_port() const {
    return this->get_output_port(state_output_port_);
  }

  const systems::OutputPort<double>&
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

  const multibody::MultibodyPlant<double>& plant() { return plant_; }

 protected:
  void SetDefaultState(const systems::Context<double>& context,
                       systems::State<double>* state) const override;

  void DoCalcUnrestrictedUpdate(const systems::Context<double>& context,
            const std::vector<const systems::UnrestrictedUpdateEvent<double>*>&,
            systems::State<double>* state) const override;

 private:
  struct PlanData;

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

}  // namespace planner
}  // namespace manipulation
}  // namespace drake
