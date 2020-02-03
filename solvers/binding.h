#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
/**
 * A binding on constraint type C is a mapping of the decision
 * variables onto the inputs of C.  This allows the constraint to operate
 * on a vector made up of different elements of the decision variables.
 */
template <typename C>
class Binding {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Binding)

  Binding(const std::shared_ptr<C>& c,
          const Eigen::Ref<const VectorXDecisionVariable>& v)
      : evaluator_(c), vars_(v) {
    DRAKE_DEMAND(c->num_vars() == v.rows() || c->num_vars() == Eigen::Dynamic);
  }

  /**
   * Concatenates each VectorDecisionVariable object in @p v into a single
   * column vector, binds this column vector of decision variables with
   * the constraint @p c.
   */
  Binding(const std::shared_ptr<C>& c, const VariableRefList& v)
      : evaluator_(c) {
    vars_ = ConcatenateVariableRefList(v);
    DRAKE_DEMAND(c->num_vars() == vars_.rows() ||
                 c->num_vars() == Eigen::Dynamic);
  }

  template <typename U>
  Binding(const Binding<U>& b,
          typename std::enable_if<std::is_convertible<
              std::shared_ptr<U>, std::shared_ptr<C>>::value>::type* = nullptr)
      : Binding(b.evaluator(), b.variables()) {}

  const std::shared_ptr<C>& evaluator() const { return evaluator_; }

  const VectorXDecisionVariable& variables() const { return vars_; }

  /**
   * Returns true iff the given @p var is included in this Binding. */
  bool ContainsVariable(const symbolic::Variable& var) const {
    for (int i = 0; i < vars_.rows(); ++i) {
      if (vars_(i).equal_to(var)) {
        return true;
      }
    }
    return false;
  }

  size_t GetNumElements() const {
    // TODO(ggould-tri) assumes that no index appears more than once in the
    // view, which is nowhere asserted (but seems assumed elsewhere).
    return vars_.size();
  }

 private:
  std::shared_ptr<C> evaluator_;
  VectorXDecisionVariable vars_;
};

/**
 * Get a description of the binding.
 */
template <typename C>
std::string to_string(const Binding<C>& binding) {
  std::string name = binding.evaluator()->get_description() + " on";
  for (int i = 0; i < binding.variables().rows(); ++i) {
    name += " " + binding.variables()(i).get_name();
  }
  name += "\n";
  return name;
}

template <typename C>
std::ostream& operator<<(std::ostream& os, const Binding<C>& binding) {
  return os << to_string(binding);
}

/**
 * Specialized template function to get the description of a linear cost
 * binding.
 */
template <>
std::string to_string<LinearCost>(
    const Binding<LinearCost>& linear_cost_binding);

/**
 * Specialized template function to get the description of a quadratic cost
 * binding.
 */
template <>
std::string to_string<QuadraticCost>(
    const Binding<QuadraticCost>& quadratic_cost_binding);

/**
 * Specialized template function to get the description of a bounding box
 * constraint binding.
 */
template <>
std::string to_string<BoundingBoxConstraint>(
    const Binding<BoundingBoxConstraint>& bounding_box_binding);

/**
 * Specialized template function to get the description of a linear equality
 * constraint binding.
 */
template <>
std::string to_string<LinearEqualityConstraint>(
    const Binding<LinearEqualityConstraint>& linear_eq_binding);

/**
 * Specialized template function to get the description of a linear constraint
 * binding.
 */
template <>
std::string to_string<LinearConstraint>(
    const Binding<LinearConstraint>& linear_eq_binding);

namespace internal {

/*
 * Create binding, inferring the type from the provided pointer.
 * @tparam C Cost or Constraint type to be bound.
 * @note Since this forwards arguments, this will not be usable with
 * `std::intializer_list`.
 */
template <typename C, typename... Args>
Binding<C> CreateBinding(const std::shared_ptr<C>& c, Args&&... args) {
  return Binding<C>(c, std::forward<Args>(args)...);
}

template <typename To, typename From>
Binding<To> BindingDynamicCast(const Binding<From>& binding) {
  auto constraint = std::dynamic_pointer_cast<To>(binding.evaluator());
  DRAKE_DEMAND(constraint != nullptr);
  return Binding<To>(constraint, binding.variables());
}

}  // namespace internal

}  // namespace solvers
}  // namespace drake
