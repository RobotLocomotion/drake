#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_copyable.h"
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
class DirectCollocationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirectCollocationConstraint)

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

 protected:
  virtual void dynamics(const TaylorVecXd& state, const TaylorVecXd& input,
                        TaylorVecXd* xdot) const = 0;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd> &x,
              Eigen::VectorXd &y) const override;
  void DoEval(const Eigen::Ref<const TaylorVecXd> &x,
              TaylorVecXd &y) const override;


 private:
  int num_states_;
  int num_inputs_;
};

/// Implements a dynamic constraint which uses the continuous dynamics
/// of a system.
class SystemDirectCollocationConstraint : public DirectCollocationConstraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemDirectCollocationConstraint)

  /// Creates a direct collocation constraint for a system.
  /// @param system A dynamical system to be used in the dynamic constraints.
  ///  This system must implement DoToAutoDiffXd.  Note that the optimization
  ///  will "clone" this system for it's internal use; changes to system
  ///  after calling this method will NOT impact the trajectory optimization.
  ///  Currently, this system must have exactly one input port.
  /// @param context Required to describe any parameters of the system.  The
  ///  values of the state in this context do not have any effect.  This
  ///  context will also be "cloned" by the optimization; changes to the context
  ///  after calling this method will NOT impact the trajectory optimization.
  SystemDirectCollocationConstraint(const systems::System<double>& system,
                                     const systems::Context<double>& context);
  ~SystemDirectCollocationConstraint() override;

 private:
  void dynamics(const TaylorVecXd& state, const TaylorVecXd& input,
                TaylorVecXd* xdot) const override;

  std::unique_ptr<System<AutoDiffXd>> system_;
  std::unique_ptr<Context<AutoDiffXd>> context_;
  FreestandingInputPortValue* input_port_value_{nullptr};
  std::unique_ptr<ContinuousState<AutoDiffXd>> derivatives_;
};

}  // namespace systems
}  // namespace drake
