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
  PendulumPlant();

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit PendulumPlant(const PendulumPlant<U>&);

  ~PendulumPlant() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /// Returns the input port to the externally applied force.
  const systems::InputPortDescriptor<T>& get_tau_port() const;

  /// Returns the port to output state.
  const systems::OutputPort<T>& get_output_port() const;

  void set_theta(MyContext* context, const T& theta) const {
    get_mutable_state(context)->set_theta(theta);
  }

  void set_thetadot(MyContext* context, const T& thetadot) const {
    get_mutable_state(context)->set_thetadot(thetadot);
  }

  explicit PendulumPlant(const PendulumPlant& other) = delete;
  PendulumPlant& operator=(const PendulumPlant& other) = delete;
  explicit PendulumPlant(PendulumPlant&& other) = delete;
  PendulumPlant& operator=(PendulumPlant&& other) = delete;

  T CalcTotalEnergy(const MyContext& context) const;

 private:
  // This is the calculator method for the state output port.
  void CopyStateOut(const MyContext& context,
                    PendulumState<T>* output) const;

  void DoCalcTimeDerivatives(const MyContext& context,
                             MyContinuousState* derivatives) const override;

  T get_tau(const MyContext& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(0);
  }

  static const PendulumState<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const PendulumState<T>&>(cstate.get_vector());
  }

  static PendulumState<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<PendulumState<T>*>(cstate->get_mutable_vector());
  }

  static PendulumState<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<PendulumState<T>*>(
        output->GetMutableVectorData(0));
  }

  static const PendulumState<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static PendulumState<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
