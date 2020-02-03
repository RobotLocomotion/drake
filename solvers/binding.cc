#include "drake/solvers/binding.h"

namespace drake {
namespace solvers {
std::string CostBindingToString(const Binding<Cost>& cost_binding) {
  VectorX<symbolic::Expression> e;
  cost_binding.evaluator()->Eval(cost_binding.variables(), &e);
  return e[0].to_string();
}

template <>
std::string to_string<LinearCost>(
    const Binding<LinearCost>& linear_cost_binding) {
  return CostBindingToString(linear_cost_binding);
}

template <>
std::string to_string<QuadraticCost>(
    const Binding<QuadraticCost>& quadratic_cost_binding) {
  return CostBindingToString(quadratic_cost_binding);
}

template <>
std::string to_string<BoundingBoxConstraint>(
    const Binding<BoundingBoxConstraint>& bounding_box_binding) {
  std::string str{};
  for (int i = 0; i < bounding_box_binding.variables().rows(); ++i) {
    str += std::to_string(bounding_box_binding.evaluator()->lower_bound()(i)) +
           "<=" + bounding_box_binding.variables()(i).get_name() + "<=" +
           std::to_string(bounding_box_binding.evaluator()->upper_bound()(i)) +
           "\n";
  }
  return str;
}

template <>
std::string to_string<LinearEqualityConstraint>(
    const Binding<LinearEqualityConstraint>& linear_eq_binding) {
  VectorX<symbolic::Expression> Ax(linear_eq_binding.GetNumElements());
  linear_eq_binding.evaluator()->Eval(linear_eq_binding.variables(), &Ax);
  std::string str{};
  for (int i = 0; i < Ax.rows(); ++i) {
    str += Ax(i).to_string() + " == " +
           std::to_string(linear_eq_binding.evaluator()->upper_bound()(i)) +
           "\n";
  }
  return str;
}

template <>
std::string to_string<LinearConstraint>(
    const Binding<LinearConstraint>& linear_binding) {
  VectorX<symbolic::Expression> Ax(linear_binding.GetNumElements());
  linear_binding.evaluator()->Eval(linear_binding.variables(), &Ax);
  std::string str{};
  for (int i = 0; i < Ax.rows(); ++i) {
    str += std::to_string(linear_binding.evaluator()->lower_bound()(i)) +
           " <= " + Ax(i).to_string() + " <= " +
           std::to_string(linear_binding.evaluator()->upper_bound()(i)) + "\n";
  }
  return str;
}
}  // namespace solvers
}  // namespace drake
