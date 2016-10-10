#pragma once

#include <memory>

#include "drake/common/drake_export.h"
#include "drake/examples/Pendulum/gen/pendulum_state_vector.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace pendulum {

/// A model of a one-dimensional pendulum system, similar to the one
/// described in Chapter 2 of Russ Tedrake. Underactuated Robotics:
/// Algorithms for Walking, Running, Swimming, Flying, and
/// Manipulation (Course Notes for MIT 6.832). Downloaded on
/// 2016-09-30 from
/// http://underactuated.csail.mit.edu/underactuated.html?chapter=2
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class PendulumSystem : public systems::LeafSystem<T> {
 public:
  PendulumSystem();
  ~PendulumSystem() override;

  using MyContext = systems::Context<T>;
  using MyContinuousState = systems::ContinuousState<T>;
  using MyOutput = systems::SystemOutput<T>;

  /// The input force to this system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Returns the input port to the externally applied force.
  const systems::SystemPortDescriptor<T>& get_tau_port() const;

  /// Returns the port to output state.
  const systems::SystemPortDescriptor<T>& get_output_port() const;

  void set_theta(MyContext* context, const T& theta) const {
    get_mutable_state(context)->set_theta(theta);
  }

  void set_thetadot(MyContext* context, const T& thetadot) const {
    get_mutable_state(context)->set_thetadot(thetadot);
  }

  void EvalOutput(const MyContext& context, MyOutput* output) const override;

  void EvalTimeDerivatives(const MyContext& context,
                           MyContinuousState* derivatives) const override;

  /// Pendulum mass in kg
  T m() const { return m_; }
  /// Pendulum length in meters
  T l() const { return l_; }
  /// Damping torque in kg m^2 / s
  T b() const { return b_; }
  /// Gravity in m/s^2
  T g() const { return g_; }

  explicit PendulumSystem(const PendulumSystem& other) = delete;
  PendulumSystem& operator=(const PendulumSystem& other) = delete;
  explicit PendulumSystem(PendulumSystem&& other) = delete;
  PendulumSystem& operator=(PendulumSystem&& other) = delete;

 protected:
  // LeafSystem<T> override.
  std::unique_ptr<MyContinuousState>
  AllocateContinuousState() const override;

  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;

 private:
  T get_tau(const MyContext& context) const {
    return this->EvalVectorInput(context, 0)->GetAtIndex(0);
  }

  static const PendulumStateVector<T>& get_state(
      const MyContinuousState& cstate) {
    return dynamic_cast<const PendulumStateVector<T>&>(cstate.get_state());
  }

  static PendulumStateVector<T>* get_mutable_state(
      MyContinuousState* cstate) {
    return dynamic_cast<PendulumStateVector<T>*>(cstate->get_mutable_state());
  }

  static PendulumStateVector<T>* get_mutable_output(MyOutput* output) {
    return dynamic_cast<PendulumStateVector<T>*>(
        output->GetMutableVectorData(0));
  }

  static const PendulumStateVector<T>& get_state(const MyContext& context) {
    return get_state(*context.get_continuous_state());
  }

  static PendulumStateVector<T>* get_mutable_state(MyContext* context) {
    return get_mutable_state(context->get_mutable_continuous_state());
  }

  const double m_{1.0};   // kg
  const double l_{.5};    // m
  const double b_{0.1};   // kg m^2 /s
  const double lc_{.5};   // m
  const double I_{.25};   // m*l^2; % kg*m^2
  const double g_{9.81};  // m/s^2
};

}  // namespace pendulum
}  // namespace examples
}  // namespace drake
