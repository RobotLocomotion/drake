#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/linear_system.h"

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
  /// @param observed_system  The forward model for the observer.
  /// @param observed_system_context Required because it may contain parameters
  /// which we need to evaluate the system.
  /// @param observer_gain A m-by-n matrix where m is the number of state
  /// variables in the observed_system, and n is the number of outputs of the
  /// observed_system.
  ///
  /// Note: Takes ownership of the unique_ptrs to the observed_system and the
  /// observed_system_context (use std::move when calling this method).
  LuenbergerObserver(
      std::unique_ptr<systems::System<T>> observed_system,
      std::unique_ptr<systems::Context<T>> observed_system_context,
      const Eigen::Ref<const Eigen::MatrixXd>& observer_gain);

  /// Non-copyable.
  LuenbergerObserver(const LuenbergerObserver<T>&) = delete;
  LuenbergerObserver& operator=(const LuenbergerObserver<T>&) = delete;

  /// This system is not direct feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Advance the state estimate using forward dynamics and the observer gains.
  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  /// Outputs the estimated state.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  /// Access to the observer gain
  const Eigen::MatrixXd& observer_gain() { return observer_gain_; }

  /// Provide access via the short-hand name, too.
  const Eigen::MatrixXd& L() { return observer_gain_; }

 private:
  const std::unique_ptr<systems::System<T>> observed_system_;
  const Eigen::MatrixXd observer_gain_;  // Gain matrix (often called "L").

  // A (mutable) context is needed to efficiently call the observed system's
  // dynamics and output methods.  Does not add any undeclared state.  This
  // simply avoids the need to allocate a new context on every function
  // evaluation.
  const std::unique_ptr<systems::Context<T>> observed_system_context_;
  const std::unique_ptr<systems::SystemOutput<T>> observed_system_output_;
};

}  // namespace estimators
}  // namespace systems
}  // namespace drake
