#pragma once

#include "drake/systems/optimization/system_constraint_wrapper.h"

namespace drake {
namespace systems {
namespace internal {
struct UpdateContextForSymbolicSystemConstraint {
 public:
  UpdateContextForSymbolicSystemConstraint(
      const Context<symbolic::Expression>* context);

  void operator()(const System<double>& system,
                  const Eigen::Ref<const VectorX<double>>& x,
                  Context<double>* context) const;

  void operator()(const System<AutoDiffXd>& system,
                  const Eigen::Ref<const AutoDiffVecXd>& x,
                  Context<AutoDiffXd>* context) const;

  const VectorX<symbolic::Variable>& bound_variables() const {
    return bound_variables_;
  }

  bool successfully_constructed() const { return successfully_constructed_; }

 private:
  bool AddSymbolicVariables(
      const symbolic::Expression& expr,
      std::function<void(Context<double>* context, double val)> updater_double,
      std::function<void(Context<AutoDiffXd>* context, const AutoDiffXd& val)>
          updater_autodiff);

  const Context<symbolic::Expression>* const context_;
  VectorX<symbolic::Variable> bound_variables_;
  // map a bounded variable to its index in bound_variables_;
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index_;
  std::vector<UpdateContextFromDecisionVariablesFunction<double>>
      updaters_double_;
  std::vector<UpdateContextFromDecisionVariablesFunction<AutoDiffXd>>
      updaters_autodiff_;
  bool successfully_constructed_;
};
}  // namespace internal
}  // namespace systems
}  // namespace drake
