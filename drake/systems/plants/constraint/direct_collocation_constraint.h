#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {

/// Provides a base implementation and interface for a dynamic
/// constraint (which is intended to be used with trajectory
/// optimization, but is not specific to that purpose). This
/// implementation deliberately knows nothing about the underlying
/// system representation.
///
/// Each evaluation of the constraint considers a pair of state
/// vectors + input vectors along with an accompanying timestep.
class DRAKE_EXPORT DirectCollocationConstraint :
      public solvers::Constraint {
 public:
  /// The format of the input to the eval() function is defined by @p
  /// num_states and @p num_inputs.  The length of the vector will be
  /// (1 + num_states * 2 + num_inputs * 2), with the format:
  ///
  /// (length)
  /// 1: timestep
  /// num_states: state 0
  /// num_states: state 1
  /// num_inputs: input 0
  /// num_inputs: input 1
  DirectCollocationConstraint(int num_states, int num_inputs);
  virtual ~DirectCollocationConstraint();

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override;
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override;

  explicit DirectCollocationConstraint(
      const DirectCollocationConstraint& other) = delete;
  DirectCollocationConstraint& operator=(
      const DirectCollocationConstraint& other) = delete;
  explicit DirectCollocationConstraint(
      DirectCollocationConstraint&& other) = delete;
  DirectCollocationConstraint& operator=(
      DirectCollocationConstraint&& other) = delete;

 protected:
  virtual void dynamics(const TaylorVecXd& state,
                        const TaylorVecXd& input,
                        TaylorVecXd* xdot) const = 0;

 private:
  int num_states_;
  int num_inputs_;
};

/// Implements a dynamic constraint which uses the continuous dynamics
/// of a system.
class DRAKE_EXPORT System2DirectCollocationConstraint
    : public DirectCollocationConstraint {
 public:
  /// Create a direct colocation constraint for a system.  Systems
  /// must have a single input port and a single output port, match
  /// the template requirement of AutoDiffXd, and implement
  /// EvalTimeDerivatives.
  explicit System2DirectCollocationConstraint(
      std::unique_ptr<System<AutoDiffXd>> system);
  ~System2DirectCollocationConstraint() override;

  explicit System2DirectCollocationConstraint(
      const System2DirectCollocationConstraint& other) = delete;
  System2DirectCollocationConstraint& operator=(
      const System2DirectCollocationConstraint& other) = delete;
  explicit System2DirectCollocationConstraint(
      System2DirectCollocationConstraint&& other) = delete;
  System2DirectCollocationConstraint& operator=(
      System2DirectCollocationConstraint&& other) = delete;

 private:
  void dynamics(const TaylorVecXd& state,
                const TaylorVecXd& input,
                TaylorVecXd* xdot) const override;

  std::unique_ptr<System<AutoDiffXd>> system_;
  std::unique_ptr<Context<AutoDiffXd>> context_;
  FreestandingInputPort* input_port_{nullptr};
  std::unique_ptr<ContinuousState<AutoDiffXd>> derivatives_;
};

}  // namespace systems
}  // namespace drake
