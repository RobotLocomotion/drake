#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/drakeDynamicConstraint_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/System.h"

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
class DRAKEDYNAMICCONSTRAINT_EXPORT DynamicConstraint :
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
  DynamicConstraint(int num_states, int num_inputs);
  virtual ~DynamicConstraint();

  void Eval(const Eigen::Ref<const Eigen::VectorXd>& x,
            Eigen::VectorXd& y) const override;
  void Eval(const Eigen::Ref<const TaylorVecXd>& x,
            TaylorVecXd& y) const override;

 protected:
  virtual void dynamics(const TaylorVecXd& state,
                        const TaylorVecXd& input,
                        TaylorVecXd* xdot) const = 0;

 private:
  int num_states_;
  int num_inputs_;
};

/// Implements a dynamic constraint which uses the dynamics function
/// of a system.
template <typename System>
class SystemDynamicConstraint : public DynamicConstraint {
 public:
  // TODO(sam.creasey) Should this be a const bare ptr?
  explicit SystemDynamicConstraint(std::shared_ptr<System> system)
      : DynamicConstraint(drake::getNumStates(*system),
                          drake::getNumInputs(*system)),
        system_(system) {}

 private:
  void dynamics(const TaylorVecXd& state,
                const TaylorVecXd& input,
                TaylorVecXd* xdot) const override {
    typename System::template StateVector<TaylorVarXd> x = state;
    typename System::template InputVector<TaylorVarXd> u = input;
    TaylorVarXd t(1);
    t = 0;
    *xdot = toEigen(system_->dynamics(t, x, u));
  }

  std::shared_ptr<System> system_;
};

}  // systems
}  // drake
