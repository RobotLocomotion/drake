#pragma once

#include <memory>

#include "drake/common/symbolic.h"
#include "drake/examples/pendulum/gen/pendulum_input.h"
#include "drake/examples/pendulum/gen/pendulum_params.h"
#include "drake/examples/pendulum/gen/pendulum_state.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// A model of a simple pendulum
/// @f[ ml^2 \ddot\theta + b\dot\theta + mgl\sin\theta = u @f]
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
template <typename T>
class PendulumPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PendulumPlant);

  PendulumPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PendulumPlant(const PendulumPlant<U>&);

  ~PendulumPlant() override;

  /// Returns the input port to the externally applied force.
  const systems::InputPortDescriptor<T>& get_input_port() const;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_output_port() const;

  /// Calculates the kinetic + potential energy.
  T CalcTotalEnergy(const systems::Context<T>& context) const;

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  T get_tau(const systems::Context<T>& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(0);
  }

  static const PendulumState<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const PendulumState<T>&>(cstate.get_vector());
  }

  static const PendulumState<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static PendulumState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<PendulumState<T>&>(cstate->get_mutable_vector());
  }

  static PendulumState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  static PendulumState<T>* get_mutable_output(
      systems::SystemOutput<T>* output) {
    return dynamic_cast<PendulumState<T>*>(output->GetMutableVectorData(0));
  }

  const PendulumParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<PendulumParams>(context, 0);
  }

 private:
  // This is the calculator method for the state output port.
  void CopyStateOut(const systems::Context<T>& context,
                    PendulumState<T>* output) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
