#include "drake/systems/optimization/system_constraint_adapter.h"

#include <unordered_map>

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

struct ContextContinuousStateIndex {
  ContextContinuousStateIndex(int m_state_index, int m_var_index)
      : state_index(m_state_index), var_index(m_var_index) {}
  int state_index;
  int var_index;
};

struct ContextDiscreteStateIndex {
  ContextDiscreteStateIndex(int m_group_index, int m_state_index,
                            int m_var_index)
      : group_index(m_group_index),
        state_index(m_state_index),
        var_index(m_var_index) {}
  int group_index;
  int state_index;
  int var_index;
};

struct ContextNumericParameterIndex {
  ContextNumericParameterIndex(int m_group_index, int m_parameter_index,
                               int m_var_index)
      : group_index(m_group_index),
        parameter_index(m_parameter_index),
        var_index(m_var_index) {}
  int group_index;
  int parameter_index;
  int var_index;
};

namespace {
struct UpdateContextForSymbolicSystemConstraint {
 public:
  UpdateContextForSymbolicSystemConstraint(
      const std::vector<ContextContinuousStateIndex>& continuous_state_indices,
      const std::vector<ContextDiscreteStateIndex>& discrete_state_indices,
      const std::vector<ContextNumericParameterIndex>&
          numeric_parameter_indices,
      const optional<int>& time_var_index)
      : continuous_state_indices_{continuous_state_indices},
        discrete_state_indices_{discrete_state_indices},
        numeric_parameter_indices_{numeric_parameter_indices},
        time_var_index_{time_var_index} {}

  template <typename T>
  void operator()(const System<T>&, const Eigen::Ref<const VectorX<T>>& x,
                  Context<T>* context) const {
    // Time.
    if (time_var_index_.has_value()) {
      context->set_time(x(time_var_index_.value()));
    }
    // Continuous state.
    for (const auto& continuous_state_index : continuous_state_indices_) {
      context->get_mutable_continuous_state_vector().GetAtIndex(
          continuous_state_index.state_index) =
          x(continuous_state_index.var_index);
    }
    // Discrete state.
    for (const auto& discrete_state_index : discrete_state_indices_) {
      context->get_mutable_discrete_state(discrete_state_index.group_index)
          .GetAtIndex(discrete_state_index.state_index) =
          x(discrete_state_index.var_index);
    }
    // Numeric parameter.
    for (const auto& numeric_parameter_index : numeric_parameter_indices_) {
      context
          ->get_mutable_numeric_parameter(numeric_parameter_index.group_index)
          .GetAtIndex(numeric_parameter_index.parameter_index) =
          x(numeric_parameter_index.var_index);
    }
  }

 private:
  std::vector<ContextContinuousStateIndex> continuous_state_indices_;
  std::vector<ContextDiscreteStateIndex> discrete_state_indices_;
  std::vector<ContextNumericParameterIndex> numeric_parameter_indices_;
  optional<int> time_var_index_;
};

// Parse the expression to either @p constant_val or the variable. Append that
// variable to @p map-var_to_index and bound_variables. Return true if the
// expression is either a constant or a symbolic variable; otherwise returns
// false.
bool ParseSymbolicVariableOrConstant(
    const symbolic::Expression& expr,
    std::unordered_map<symbolic::Variable::Id, int>* map_var_to_index,
    VectorX<symbolic::Variable>* bound_variables, optional<int>* variable_index,
    optional<double>* constant_val) {
  if (symbolic::is_constant(expr)) {
    *constant_val = symbolic::get_constant_value(expr);
    variable_index->reset();
    return true;
  } else if (symbolic::is_variable(expr)) {
    const symbolic::Variable& var = symbolic::get_variable(expr);
    auto it = map_var_to_index->find(var.get_id());
    if (it == map_var_to_index->end()) {
      map_var_to_index->emplace_hint(it, var.get_id(), bound_variables->rows());
      bound_variables->conservativeResize(bound_variables->rows() + 1);
      (*bound_variables)(bound_variables->rows() - 1) = var;
      *variable_index = bound_variables->rows() - 1;
    } else {
      *variable_index = it->second;
    }
    constant_val->reset();
    return true;
  } else {
    return false;
  }
}
}  // namespace

optional<solvers::Binding<solvers::Constraint>>
SystemConstraintAdapter::MaybeCreateGenericConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {
  // TODO(hongkai.dai): First find out all the symbolic variables that could
  // appear in the system constraint. If system_symbolic_ is not nullptr, then
  // we can evaluate the system constraint using @p context. Otherwise, all the
  // variables shown up in @p context would be the decision variables.
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  VectorX<symbolic::Variable> bound_variables;
  // context_fixed stores the constant values in @p context
  auto context_fixed = system_double_->CreateDefaultContext();

  std::vector<ContextContinuousStateIndex> continuous_state_indices;
  std::vector<ContextDiscreteStateIndex> discrete_state_indices;
  std::vector<ContextNumericParameterIndex> numeric_parameter_indices;
  optional<int> time_var_index{};
  optional<double> constant_val{};
  optional<int> variable_index{};
  // Time
  bool success = ParseSymbolicVariableOrConstant(
      context.get_time(), &map_var_to_index, &bound_variables, &variable_index,
      &constant_val);
  if (!success) {
    return {};
  }
  if (constant_val.has_value()) {
    context_fixed->set_time(constant_val.value());
  } else {
    time_var_index = variable_index.value();
  }
  // Continuous state.
  for (int i = 0; i < context.get_continuous_state_vector().size(); ++i) {
    success = ParseSymbolicVariableOrConstant(
        context.get_continuous_state_vector().GetAtIndex(i), &map_var_to_index,
        &bound_variables, &variable_index, &constant_val);
    if (!success) {
      return {};
    }
    if (variable_index.has_value()) {
      continuous_state_indices.emplace_back(i, *variable_index);
    } else if (constant_val.has_value()) {
      context_fixed->get_mutable_continuous_state_vector().GetAtIndex(i) =
          constant_val.value();
    }
  }
  // Discrete state.
  for (int i = 0; i < context.get_num_discrete_state_groups(); ++i) {
    for (int j = 0; j < context.get_discrete_state(i).size(); ++j) {
      success = ParseSymbolicVariableOrConstant(
          context.get_discrete_state(i).GetAtIndex(j), &map_var_to_index,
          &bound_variables, &variable_index, &constant_val);
      if (!success) {
        return {};
      }
      if (variable_index.has_value()) {
        discrete_state_indices.emplace_back(i, j, *variable_index);
      } else if (constant_val.has_value()) {
        context_fixed->get_mutable_discrete_state(i).GetAtIndex(j) =
            constant_val.value();
      }
    }
  }
  // numeric parameters.
  for (int i = 0; i < context.num_numeric_parameter_groups(); ++i) {
    for (int j = 0; j < context.get_numeric_parameter(i).size(); ++j) {
      success = ParseSymbolicVariableOrConstant(
          context.get_numeric_parameter(i).GetAtIndex(j), &map_var_to_index,
          &bound_variables, &variable_index, &constant_val);
      if (!success) {
        return {};
      }
      if (variable_index.has_value()) {
        numeric_parameter_indices.emplace_back(i, j, *variable_index);
      } else if (constant_val.has_value()) {
        context_fixed->get_mutable_numeric_parameter(i).GetAtIndex(j) =
            constant_val.value();
      }
    }
  }

  // abstract state
  if (context.get_num_abstract_states() != 0) {
    throw std::invalid_argument(
        "SystemConstraintAdapter: cannot handle system with abstract state "
        "using symbolic Context, try SystemConstraintAdapter::Create() "
        "instead.");
  }
  // abstract parameter
  if (context.num_abstract_parameters() != 0) {
    throw std::invalid_argument(
        "SystemConstraintAdapter: cannot handle system with abstract "
        "paramter using symbolic Context, try "
        "SystemConstraintAdapter::Create() instead.");
  }

  UpdateContextForSymbolicSystemConstraint updater(
      continuous_state_indices, discrete_state_indices,
      numeric_parameter_indices, time_var_index);

  return solvers::Binding<solvers::Constraint>(
      this->Create(index, *context_fixed, updater, map_var_to_index.size()),
      bound_variables);
}

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
