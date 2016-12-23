#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace estimators {

/// A simple state observer for a dynamical system of the form:
/// @f[\dot{x} = f(x,u) @f]
/// @f[y = g(x,u) @f]
/// the observer dynamics takes the form
/// @f[\dot{\hat{x}} = f(\hat{x},u) + L(y - g(\hat{x},u)) @f]
/// where @f$\hat{x}@f$ is the estimated state of the original system.
///
/// The output of the observer system is @f$\hat{x}@f$.
///
/// @ingroup estimator_systems
template <typename T>
class LuenbergerObserver : public systems::LeafSystem<T> {
 public:
  /// Constructs the oberver.
  ///
  /// @param observed_system  The forward model for the observer.  Currently,
  /// this system must have a maximum of one input port and exactly one output
  /// port.
  /// @param observed_system_context Required because it may contain parameters
  /// which we need to evaluate the system.
  /// @param observer_gain A m-by-n matrix where m is the number of state
  /// variables in @p observed_system, and n is the dimension of the output port
  /// of @p observed_system.
  ///
  /// Note: Takes ownership of the unique_ptrs to the observed_system and the
  /// observed_system_context.
  LuenbergerObserver(
      std::unique_ptr<systems::System<T>> observed_system,
      std::unique_ptr<systems::Context<T>> observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& observer_gain);

  /// Non-copyable.
  LuenbergerObserver(const LuenbergerObserver<T>&) = delete;
  LuenbergerObserver& operator=(const LuenbergerObserver<T>&) = delete;

  /// This system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Provides access to the observer gain.
  const Eigen::MatrixXd& observer_gain() { return observer_gain_; }

  /// Provides access via the short-hand name, L, too.
  const Eigen::MatrixXd& L() { return observer_gain_; }

 private:
  // Advance the state estimate using forward dynamics and the observer gains.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Outputs the estimated state.
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  const std::unique_ptr<systems::System<T>> observed_system_;
  const Eigen::MatrixXd observer_gain_;  // Gain matrix (often called "L").

  // A (mutable) context is needed to efficiently call the observed system's
  // dynamics and output methods.  Does not add any undeclared state.  This
  // simply avoids the need to allocate a new context on every function
  // evaluation.
  const std::unique_ptr<systems::Context<T>> observed_system_context_;
  const std::unique_ptr<systems::SystemOutput<T>> observed_system_output_;
  const std::unique_ptr<systems::ContinuousState<T>>
  observed_system_derivatives_;
};

}  // namespace estimators
}  // namespace systems
}  // namespace drake
