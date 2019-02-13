#include "drake/systems/optimization/system_constraint_adapter.h"
#include "drake/systems/optimization/system_constraint_adapter_internal.h"

#include "drake/solvers/create_constraint.h"

namespace drake {
namespace systems {
SystemConstraintAdapter::SystemConstraintAdapter(
    const System<double>* const system)
    : system_double_{system},
      system_autodiff_{system_double_->ToAutoDiffXd()},
      // TODO(hongkai.dai): use system_symbolic_ to parse the constraint in the
      // symbolic form.
      system_symbolic_{system_double_->ToSymbolicMaybe()} {
  DRAKE_DEMAND(system);
}

optional<std::vector<solvers::Binding<solvers::Constraint>>>
SystemConstraintAdapter::MaybeCreateConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {
  if (!system_symbolic_) {
    return {};
  }
  const SystemConstraint<symbolic::Expression>& system_constraint =
      system_symbolic_->get_constraint(index);
  VectorX<symbolic::Expression> constraint_val(system_constraint.size());
  // Evaluate the constraint as symbolic expressions.
  system_constraint.Calc(context, &constraint_val);
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  constraints.reserve(constraint_val.size());
  // Parse the symbolic expression to constraint.
  // If the symbolic expression have some special structures (for example, being
  // linear), then we can create a
  // LinearConstraint/LinearEqualityConstraint/BoundingBoxConstraint.

  bool is_success = true;
  for (int i = 0; i < constraint_val.size(); ++i) {
    std::unique_ptr<solvers::Binding<solvers::Constraint>> linear_constraint =
        solvers::internal::MaybeParseLinearConstraint(
            constraint_val(i), system_constraint.bounds().lower()(i),
            system_constraint.bounds().upper()(i));
    if (linear_constraint) {
      constraints.push_back(*linear_constraint);
    } else {
      // TODO(hongkai.dai): parse second order cone constraint.
      is_success = false;
      break;
    }
  }
  if (!is_success) {
    return {};
  }
  return constraints;
}

namespace internal {
UpdateContextForSymbolicSystemConstraint::
    UpdateContextForSymbolicSystemConstraint(
        const Context<symbolic::Expression>* context)
    : context_{context} {
  // continuous state.
  for (int i = 0; i < context_->get_continuous_state_vector().size(); ++i) {
    AddSymbolicVariables(
        context_->get_continuous_state_vector().GetAtIndex(i),
        [i](Context<double>* m_context, double val) {
          m_context->get_mutable_continuous_state_vector().GetAtIndex(i) = val;
        },
        [i](Context<AutoDiffXd>* m_context, const AutoDiffXd& val) {
          m_context->get_mutable_continuous_state_vector().GetAtIndex(i) = val;
        });
  }
  // discrete state.
  for (int i = 0; i < context_->get_num_discrete_state_groups(); ++i) {
    for (int j = 0; j < context_->get_discrete_state(i).size(); ++j) {
      AddSymbolicVariables(
          context_->get_discrete_state(i).GetAtIndex(j),
          [i, j](Context<double>* m_context, double val) {
            m_context->get_mutable_discrete_state(i).GetAtIndex(j) = val;
          },
          [i, j](Context<AutoDiffXd>* m_context, const AutoDiffXd& val) {
            m_context->get_mutable_discrete_state(i).GetAtIndex(j) = val;
          });
    }
  }
  // time
  AddSymbolicVariables(
      context_->get_time(),
      [](Context<double>* m_context, double val) { m_context->set_time(val); },
      [](Context<AutoDiffXd>* m_context, const AutoDiffXd& val) {
        m_context->set_time(val);
      });
  // numeric parameters.
  for (int i = 0; i < context_->num_numeric_parameter_groups(); ++i) {
    for (int j = 0; j < context_->get_numeric_parameter(i).size(); ++j) {
      AddSymbolicVariables(
          context_->get_numeric_parameter(i).GetAtIndex(j),
          [i, j](Context<double>* m_context, double val) {
            m_context->get_mutable_numeric_parameter(i).SetAtIndex(j, val);
          },
          [i, j](Context<AutoDiffXd>* m_context, const AutoDiffXd& val) {
            m_context->get_mutable_numeric_parameter(i).SetAtIndex(j, val);
          });
    }
  }
}

void UpdateContextForSymbolicSystemConstraint::operator()(
    const System<double>& system, const Eigen::Ref<const VectorX<double>>& x,
    Context<double>* context) const {
  for (int i = 0; i < static_cast<int>(updaters_double_.size()); ++i) {
    updaters_double_[i](system, x, context);
  }
}

void UpdateContextForSymbolicSystemConstraint::operator()(
    const System<AutoDiffXd>& system, const Eigen::Ref<const AutoDiffVecXd>& x,
    Context<AutoDiffXd>* context) const {
  for (int i = 0; i < static_cast<int>(updaters_autodiff_.size()); ++i) {
    updaters_autodiff_[i](system, x, context);
  }
}

void UpdateContextForSymbolicSystemConstraint::AddSymbolicVariables(
    const symbolic::Expression& expr,
    std::function<void(Context<double>* context, double val)> updater_double,
    std::function<void(Context<AutoDiffXd>* context, const AutoDiffXd& val)>
        updater_autodiff) {
  if (symbolic::is_constant(expr)) {
    const double constant_val = symbolic::get_constant_value(expr);
    updaters_double_.push_back([updater_double, constant_val](
        const System<double>&, const Eigen::Ref<const VectorX<double>>&,
        Context<double>* context) { updater_double(context, constant_val); });
    updaters_autodiff_.push_back([updater_autodiff, constant_val](
        const System<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>&,
        Context<AutoDiffXd>* context) {
      updater_autodiff(context, AutoDiffXd(constant_val));
    });
  } else if (symbolic::is_variable(expr)) {
    const symbolic::Variable& var = symbolic::get_variable(expr);
    auto it = map_var_to_index_.find(var.get_id());
    int var_index{-1};
    if (it == map_var_to_index_.end()) {
      map_var_to_index_.emplace_hint(it, var.get_id(), bound_variables_.rows());
      var_index = bound_variables_.rows();
      bound_variables_.conservativeResize(bound_variables_.rows() + 1);
      bound_variables_(bound_variables_.rows() - 1) = var;
    } else {
      var_index = it->second;
    }
    updaters_double_.push_back([updater_double, var_index](
        const System<double>&, const Eigen::Ref<const VectorX<double>>& x,
        Context<double>* context) { updater_double(context, x(var_index)); });
    updaters_autodiff_.push_back([updater_autodiff, var_index](
        const System<AutoDiffXd>&, const Eigen::Ref<const AutoDiffVecXd>& x,
        Context<AutoDiffXd>* context) {
      updater_autodiff(context, x(var_index));
    });
  } else {
    throw std::invalid_argument(
        "SystemConstraintAdapter: only support Context<symbolic::Expression> "
        "with its expression being either a constant or a "
        "symbolic::Variable.");
  }
}
}  // namespace internal
/*
optional<solvers::Binding<solvers::Constraint>>
SystemConstraintAdapter::MaybeCreateGenericConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {}*/

const System<symbolic::Expression>& SystemConstraintAdapter::system_symbolic()
    const {
  if (system_symbolic_) {
    return *system_symbolic_;
  } else {
    throw std::runtime_error(
        "SystemConstraintAdapter: the system cannot be instantiated with"
        "symbolic::Expression.");
  }
}

}  // namespace systems
}  // namespace drake
