#pragma once

#include <memory>

#include "drake/solvers/constraint.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace systems {
/** Given the decision variable values x (as in
 * SystemConstraintWrapper.Eval(x, &y)), update part of the context with the
 * value of x.
 */
template <typename T>
using UpdateContextFromDecisionVariables = std::function<void(
    const System<T>&, const Eigen::Ref<const VectorX<T>>&, Context<T>*)>;

/**
 * This wrapper class wraps a SystemConstraint object to the format of
 * solvers::Constraint.
 * The constraint is
 * lower <= SystemConstraint.Calc(selector(x)) <= upper
 * where lower/upper are the lower and upper bounds of the SystemConstraint
 * object. When the lower and upper are equal, this represents an equality
 * constraint.
 */
class SystemConstraintWrapper : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemConstraintWrapper)

  /**
   * Wraps a single SystemConstraint of the given system into a
   * solvers::Constraint.
   * @param system_double The System whose SystemConstraint is converted to
   * solvers::Constraint.
   * @param system_autodiff This system should be converted from system_double
   * by converting the scalar type. If this is pointer is null, then the
   * AutoDiffXd version of the system will be created internally inside this
   * wrapper class.
   * @param index The index of the SystemConstraint in @p system_double (and
   * also @p system_autodiff).
   * @param context The value stored in this context will be used in
   * SystemConstraintWrapper::Eval. If @p selector_double (and @p
   * selector_autodiff) doesn't update everything in the context (such as state,
   * input, params, etc), then the un-updated part in the context will keep its
   * value to those stored in @p context.
   * @param selector_double Maps x in SystemConstraintWrapper::Eval(x, &y) to a
   * context. The context is then used in SystemConstraint.Calc(context).
   * @param selector_autodiff Same as @p selector_double, but works for autodiff
   * type.
   * @param x_size The number of variables bound with this constraint. Namely,
   * the size of x in SystemConstraintWrapper.Eval(x, &y).
   */
  SystemConstraintWrapper(
      const System<double>* const system_double,
      const System<AutoDiffXd>* const system_autodiff,
      SystemConstraintIndex index, const Context<double>& context,
      UpdateContextFromDecisionVariables<double> selector_double,
      UpdateContextFromDecisionVariables<AutoDiffXd> selector_autodiff,
      int x_size);

  ~SystemConstraintWrapper() override {}

  /** Get the AutoDiffXd type System stored in this constraint.*/
  const System<AutoDiffXd>* system_autodiff() const;

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  const System<double>* const system_double_;
  std::unique_ptr<const System<AutoDiffXd>> owned_system_autodiff_;
  const System<AutoDiffXd>* system_autodiff_;
  const SystemConstraintIndex index_;
  // These contexts are created by system_double_ and system_autodiff_
  // respectively. Their value are initialized to the same as @p context in the
  // constructor. They hold the values inside context that are not part of @p x
  // in the Eval function.
  std::unique_ptr<Context<double>> context_double_;
  std::unique_ptr<Context<AutoDiffXd>> context_autodiff_;
  UpdateContextFromDecisionVariables<double> selector_double_;
  UpdateContextFromDecisionVariables<AutoDiffXd> selector_autodiff_;
};
}  // namespace systems
}  // namespace drake
