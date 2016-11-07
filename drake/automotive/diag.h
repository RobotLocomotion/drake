#pragma once

#include <memory>

#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/automotive/linear_car.h"
//#include "drake/systems/framework/primitives/integrator.h"
#include "drake/automotive/idm_planner.h"
//#include "drake/automotive/idm_planner_four_inputs.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"

//#include "drake/systems/framework/primitives/integrator.h"
//#include "drake/systems/framework/primitives/adder.h"
//#include "drake/systems/framework/primitives/gain.h"
//#include "drake/systems/framework/primitives/pass_through.h"

namespace drake {
namespace automotive {

/// A PID controller system. Given an error signal `e` and its time derivative
/// `edot` the output of this sytem is
/// <pre>
///     y = Kp * e + Ki integ(e,dt) + Kd * edot
/// </pre>
/// where `integ(e,dt)` is the time integral of `e`.
/// A PID controller directly feedthroughs the error signal to the output when
/// the proportional constant is non-zero. It feeds through the rate of change
/// of the error signal when the derivative constant is non-zero.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template <typename T>
class Diag : public systems::Diagram<T> {
 public:
  /// Constructs a %Diag system where all of the gains are the same
  /// value.
  ///
  /// @param Kp the proportional constant.
  /// @param Ki the integral constant.
  /// @param Kd the derivative constant.
  /// @param size number of elements in the signal to be processed.
  Diag(const T& Kp, const T& Ki, const T& Kd,
       const T& v_0, const T& a_agent,
       int size);

  /// Constructs a %Diag system where each gain can have a different
  /// value.
  ///
  /// @param Kp the vector of proportional gain constants.
  /// @param Ki the vector of integral gain constants.
  /// @param Kd the vector of derivative gain constants.
  Diag(const VectorX<T>& Kp, const VectorX<T>& Ki,
       const VectorX<T>& Kd,
       const T& v_0, const T& a_agent
       );

  ~Diag() override {}


  // System<T> overrides
  /// A PID controller directly feedthroughs the error signal to the output when
  /// the proportional constant is non-zero. It feeds through the rate of change
  /// of the error signal when the derivative constant is non-zero.
  bool has_any_direct_feedthrough() const override;

  /// Sets @p context to a default state in which the integral of the error
  /// signal is zero.
  void SetDefaultState(systems::Context<T>* context) const;

  /// Sets the integral of the %PidController to @p value.
  /// @p value must be a column vector of the appropriate size.
  //void set_integral_value(systems::Context<T>* context,
  //                        const Eigen::Ref<const VectorX<T>>& value) const;

 private:
  //systems::Adder<T>* adder_ = nullptr;
  //systems::Integrator<T>* integrator_ = nullptr;
  //systems::PassThrough<T>* pass_through_ = nullptr;
  //systems::Gain<T>* proportional_gain_ = nullptr;
  //systems::Gain<T>* integral_gain_ = nullptr;
  //systems::Gain<T>* derivative_gain_ = nullptr;

  LinearCar<T>* ego_car_ = nullptr;
  LinearCar<T>* agent_car_ = nullptr;
  //systems::Integrator<T>* velocity_ego_car_integrator_ = nullptr;
  //systems::Integrator<T>* position_ego_car_integrator_ = nullptr;
  //systems::Integrator<T>* velocity_agent_car_integrator_ = nullptr;
  //systems::Integrator<T>* position_agent_car_integrator_ = nullptr;
  //systems::Integrator<T>* agent_car_ = nullptr;
  //IdmPlannerFourInputs<T>* planner_ = nullptr;
  IdmPlanner<T>* planner_ = nullptr;
  systems::ConstantVectorSource<T>* value_ = nullptr;
  //systems::ConstantVectorSource<T>* value_ego_ = nullptr;
  //systems::Gain<T>* another_gain_ = nullptr;
};

}  // namespace systems
}  // namespace drake
