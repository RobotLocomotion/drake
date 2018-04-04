#include "drake/solvers/integer_optimization_util.h"

#include "drake/solvers/binding.h"
#include "drake/solvers/create_constraint.h"

namespace drake {
namespace solvers {
Binding<LinearConstraint> CreateLogicalAndConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_and_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_and_b2 >= b1 + b2 - 1 &&
                                    b1_and_b2 <= b1 &&
                                    b1_and_b2 <= b2 &&
                                    0 <= b1_and_b2 &&
                                    b1_and_b2 <= 1));
      // clang-format on
}

Binding<LinearConstraint> CreateLogicalOrConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_or_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_or_b2 <= b1 + b2 &&
                                    b1_or_b2 >= b1 &&
                                    b1_or_b2 >= b2 &&
                                    0 <= b1_or_b2 &&
                                    b1_or_b2 <= 1));
      // clang-format on
}

Binding<LinearConstraint> CreateLogicalXorConstraint(
    const symbolic::Expression& b1, const symbolic::Expression& b2,
    const symbolic::Expression& b1_xor_b2) {
  return internal::BindingDynamicCast<LinearConstraint>(
      // clang-format off
      internal::ParseConstraint(b1_xor_b2 <= b1 + b2 &&
                                    b1_xor_b2 >= b1 - b2 &&
                                    b1_xor_b2 >= b2 - b1 &&
                                    b1_xor_b2 <= 2 - b1 - b2 &&
                                    0 <= b1_xor_b2 &&
                                    b1_xor_b2 <= 1));
      // clang-format on
}

Binding<LinearConstraint> CreateBinaryCodeMatchConstraint(
    const VectorX<symbolic::Expression>& code,
    const Eigen::Ref<const Eigen::VectorXi>& expected,
    const symbolic::Expression& match) {
  DRAKE_ASSERT(code.rows() == expected.rows());
  // If the elementwise and of match_and = 1, then code = expected.
  VectorX<symbolic::Expression> match_and(code.rows());
  symbolic::Formula f = match >= 0 && match <= 1;
  for (int i = 0; i < code.rows(); ++i) {
    // match_and(i) = 1 iff code(i) == expected(i)
    if (expected(i) == 1) {
      match_and(i) = code(i);
    } else if (expected(i) == 0) {
      match_and(i) = 1 - code(i);
    } else {
      throw std::logic_error("expected should only contain either 0 or 1.");
    }
    f = f && match <= match_and(i);
  }
  f = f && match >= match_and.sum() - (code.rows() - 1);
  return internal::BindingDynamicCast<LinearConstraint>(
      internal::ParseConstraint(f));
}
}  // namespace solvers
}  // namespace drake
