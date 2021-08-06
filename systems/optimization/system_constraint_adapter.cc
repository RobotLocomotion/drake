#include "drake/systems/optimization/system_constraint_adapter.h"

#include <unordered_map>
#include <utility>

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
  DRAKE_DEMAND(system != nullptr);
}

std::optional<std::vector<solvers::Binding<solvers::Constraint>>>
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

namespace {
enum class ContextElementKind {
  kContinuous,
  kDiscrete,
  kParam,
};

struct ContextIndex {
  ContextIndex(ContextElementKind kind_in, int group_index_in,
               int element_offset_in, int var_index_in)
      : kind{kind_in},
        group_index{group_index_in},
        element_offset{element_offset_in},
        var_index{var_index_in} {}
  ContextElementKind kind;
  int group_index{};
  int element_offset{};
  int var_index{};
};

/**
 * A functor to update the Context<double> or Context<AutoDiffXd> from decision
 * variables. This functor will be used as
 * UpdateContextFromDecisionVariableFunction.
 */
class UpdateContextForSymbolicSystemConstraint {
 public:
  UpdateContextForSymbolicSystemConstraint(
      std::vector<ContextIndex> context_indices,
      const std::optional<int>& time_var_index)
      : context_indices_(std::move(context_indices)),
        time_var_index_{time_var_index} {}

  template <typename T>
  void operator()(const System<T>&, const Eigen::Ref<const VectorX<T>>& x,
                  Context<T>* context) const {
    // Time.
    if (time_var_index_.has_value()) {
      context->SetTime(x(time_var_index_.value()));
    }
    for (const auto& item : context_indices_) {
      switch (item.kind) {
        case ContextElementKind::kContinuous: {
          // Continuous state.
          context->get_mutable_continuous_state_vector().GetAtIndex(
              item.element_offset) = x(item.var_index);
          break;
        }
        case ContextElementKind::kDiscrete: {
          // Discrete state.
          context->get_mutable_discrete_state(item.group_index)
              .GetAtIndex(item.element_offset) = x(item.var_index);
          break;
        }
        case ContextElementKind::kParam: {
          // Numeric parameter.
          context->get_mutable_numeric_parameter(item.group_index)
              .GetAtIndex(item.element_offset) = x(item.var_index);
          break;
        }
      }
    }
  }

 private:
  std::vector<ContextIndex> context_indices_;
  std::optional<int> time_var_index_;
};

// Parse the expression to either @p constant_val or the variable. Append that
// variable to @p map_var_to_index and bound_variables. Return true if the
// expression is either a constant or a symbolic variable; otherwise returns
// false.
bool ParseSymbolicVariableOrConstant(
    const symbolic::Expression& expr,
    std::unordered_map<symbolic::Variable::Id, int>* map_var_to_index,
    VectorX<symbolic::Variable>* bound_variables,
    std::optional<int>* variable_index, std::optional<double>* constant_val) {
  variable_index->reset();
  constant_val->reset();
  if (symbolic::is_constant(expr)) {
    *constant_val = symbolic::get_constant_value(expr);
    return true;
  } else if (symbolic::is_variable(expr)) {
    const symbolic::Variable& var = symbolic::get_variable(expr);
    auto it = map_var_to_index->find(var.get_id());
    if (it == map_var_to_index->end()) {
      const int new_index = bound_variables->rows();
      map_var_to_index->emplace_hint(it, var.get_id(), new_index);
      bound_variables->conservativeResize(bound_variables->rows() + 1);
      (*bound_variables)(new_index) = var;
      *variable_index = new_index;
    } else {
      *variable_index = it->second;
    }
    return true;
  } else {
    return false;
  }
}
}  // namespace

std::optional<solvers::Binding<solvers::Constraint>>
SystemConstraintAdapter::MaybeCreateGenericConstraintSymbolically(
    SystemConstraintIndex index,
    const Context<symbolic::Expression>& context) const {
  // TODO(hongkai.dai): First find out all the symbolic variables that could
  // appear in the system constraint. If system_symbolic_ is not nullptr, then
  // we can evaluate the system constraint using @p context. Otherwise, all the
  // variables shown up in @p context would be the decision variables. Picking
  // the variables in SystemConstraint.Calc(context) result would give us a
  // more sparse constraint (with less number of bounded variables).
  std::unordered_map<symbolic::Variable::Id, int> map_var_to_index;
  VectorX<symbolic::Variable> bound_variables;
  // context_fixed stores the constant values in @p context
  auto context_fixed = system_double_->CreateDefaultContext();
  context_fixed->SetAccuracy(context.get_accuracy());

  std::vector<ContextIndex> context_indices;
  std::optional<int> time_var_index{};
  std::optional<double> constant_val{};
  std::optional<int> variable_index{};
  // Time
  bool success = ParseSymbolicVariableOrConstant(
      context.get_time(), &map_var_to_index, &bound_variables, &variable_index,
      &constant_val);
  if (!success) {
    return {};
  }
  if (constant_val.has_value()) {
    context_fixed->SetTime(constant_val.value());
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
      context_indices.emplace_back(ContextElementKind::kContinuous, 0, i,
                                   *variable_index);
    } else {
      DRAKE_ASSERT(constant_val.has_value());
      context_fixed->get_mutable_continuous_state_vector().GetAtIndex(i) =
          constant_val.value();
    }
  }
  // Discrete state.
  for (int i = 0; i < context.num_discrete_state_groups(); ++i) {
    for (int j = 0; j < context.get_discrete_state(i).size(); ++j) {
      success = ParseSymbolicVariableOrConstant(
          context.get_discrete_state(i).GetAtIndex(j), &map_var_to_index,
          &bound_variables, &variable_index, &constant_val);
      if (!success) {
        return {};
      }
      if (variable_index.has_value()) {
        context_indices.emplace_back(ContextElementKind::kDiscrete, i, j,
                                     *variable_index);
      } else {
        DRAKE_ASSERT(constant_val.has_value());
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
        context_indices.emplace_back(ContextElementKind::kParam, i, j,
                                     *variable_index);
      } else {
        DRAKE_ASSERT(constant_val.has_value());
        context_fixed->get_mutable_numeric_parameter(i).GetAtIndex(j) =
            constant_val.value();
      }
    }
  }

  // input ports
  // TODO(hongkai.dai): parse fixed values in the input ports, when we can copy
  // the fixed input port value from Context<double> to Context<AutoDiffXd>.
  for (int i = 0; i < context.num_input_ports(); ++i) {
    if (context.MaybeGetFixedInputPortValue(i) != nullptr) {
      throw std::runtime_error(
          "SystemConstraintAdapter doesn't support system with fixed input "
          "ports yet.");
    }
  }

  // abstract state
  if (context.num_abstract_states() != 0) {
    throw std::invalid_argument(
        "SystemConstraintAdapter: cannot handle system with abstract state "
        "using symbolic Context, try SystemConstraintAdapter::Create() "
        "instead.");
  }
  // abstract parameter
  if (context.num_abstract_parameters() != 0) {
    throw std::invalid_argument(
        "SystemConstraintAdapter: cannot handle system with abstract "
        "parameter using symbolic Context, try "
        "SystemConstraintAdapter::Create() instead.");
  }

  UpdateContextForSymbolicSystemConstraint updater(context_indices,
                                                   time_var_index);

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
