#pragma once

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
  Binding(const std::shared_ptr<C>& c, const Eigen::Ref<const DecisionVariableVectorX>& v)
      : constraint_(c), vars_(v) {}

  /**
   * Concatenates each DecisionVariableVector object in @p v into a single
   * column vector, binds this column vector of decision variables with
   * the constraint @p c.
   */
  Binding(const std::shared_ptr<C>& c, const VariableListRef& v) :
      constraint_(c) {
    vars_ = ConcatenateVariableListRef(v);
  }

  template <typename U>
  Binding(
      const Binding<U>& b,
      typename std::enable_if<std::is_convertible<
          std::shared_ptr<U>, std::shared_ptr<C>>::value>::type* = nullptr)
  : Binding(b.constraint(), b.variables()) {}

  const std::shared_ptr<C>& constraint() const { return constraint_; }

  const DecisionVariableVectorX& variables() const { return vars_; }

  /**
   * Returns true iff the given @p var is included in this Binding.*/
  bool ContainsVariable(const symbolic::Variable& var) const {
    for (int i = 0; i < vars_.rows(); ++i) {
      if (vars_(i) == var) {
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
  std::shared_ptr<C> constraint_;
  DecisionVariableVectorX vars_;
};
}  // namespace solvers
}  // namespace drake
