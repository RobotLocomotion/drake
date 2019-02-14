#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/solvers/binding.h"
#include "drake/systems/optimization/system_constraint_wrapper.h"

namespace drake {
namespace systems {
/**
 * This class is a factory class to generate SystemConstraintWrapper. Namely
 * this class helps to convert a SystemConstraint to a solvers::Constraint.
 * Internally this class will convert a System<double> to System<AutoDiffXd>
 * (and System<symbolic::Expression> if possible), and store these systems (of
 * different scalar types) inside this class. Using this class with a system
 * that cannot be converted to System<AutoDiffXd> will cause a runtime error.
 */
class SystemConstraintAdapter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SystemConstraintAdapter)

  explicit SystemConstraintAdapter(const System<double>* system);

  /**
   * This method creates a solvers::Constraint from a SystemConstraint.
   * The newly created constraint represents
   * lower <=
   * system_constraint.Calc(UpdateContextFromDecisionVariablesGeneric(x)) <=
   * upper, where lower and upper are obtained from
   * SystemConstraint::lower_bound() and SystemConstraint::upper_bound().
   * @param index The index of the constraint stored inside @p system in the
   * class constructor.
   * @param context SystemConstraint::Calc function requires a context as the
   * input. On the other hand, the generated constraint might be imposed on a
   * partial subset of variables (state, time, input and parameters) inside the
   * context. Hence we use @p UpdateContextFromDecisionVariablesGeneric to
   * select the decision variables inside @p context. The unselected variables
   * will remain to its values stored in @p context.
   */
  template <typename UpdateContextFromDecisionVariablesGenericFunction>
  std::shared_ptr<SystemConstraintWrapper> Create(
      SystemConstraintIndex index, const Context<double>& context,
      UpdateContextFromDecisionVariablesGenericFunction updater,
      int x_size) const {
    return std::make_shared<SystemConstraintWrapper>(
        system_double_, system_autodiff_.get(), index, context,
        std::forward<UpdateContextFromDecisionVariablesFunction<double>>(
            updater),
        std::forward<UpdateContextFromDecisionVariablesFunction<AutoDiffXd>>(
            updater),
        x_size);
  }

  // TODO(hongkai.dai): support parsing second-order-cone or quadratic
  // constraint.
  /**
   * Given a SystemConstraint and the Context to evaluate this SystemConstraint,
   * parse the constraint in the symbolic forms. Currently we support parsing
   * the following forms:
   *
   *  1. bounding box ( lower <= x <= upper )
   *  2. linear equality ( aᵀx = b )
   *  3. linear inequality ( lower <= aᵀx <= upper )
   *
   * If the SystemConstraint cannot be parsed to the forms above, then return
   * false, and clear @p constraints. Otherwise return true.
   * @param index The index of the constraint in the System object.
   * @param context The context used to evaluate the SystemConstraint.
   * @retval constraints If the SystemConstraint can be parsed to the constraint
   * in the above forms, then constraints.value()[i] is the i'th row of the
   * SystemConstraint evaluation result; if the SystemConstraint cannot be
   * parsed in the above forms (either due to the System is not instantiated
   * with symbolic::Expression, or the constraint is not linear), then
   * constraints.has_value() = false.
   */
  optional<std::vector<solvers::Binding<solvers::Constraint>>>
  MaybeCreateConstraintSymbolically(
      SystemConstraintIndex index,
      const Context<symbolic::Expression>& context) const;

  /**
   * Returns the symbolic system. Throws a runtime error if the system cannot be
   * instantiated with symbolic::Expression.
   */
  const System<symbolic::Expression>& system_symbolic() const;

 private:
  const System<double>* const system_double_;
  const std::unique_ptr<System<AutoDiffXd>> system_autodiff_;
  const std::unique_ptr<System<symbolic::Expression>> system_symbolic_;
};
}  // namespace systems
}  // namespace drake
