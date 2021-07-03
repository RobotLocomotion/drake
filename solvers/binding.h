#pragma once

#include <cstdint>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/evaluator_base.h"

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
          typename std::enable_if_t<std::is_convertible_v<
              std::shared_ptr<U>, std::shared_ptr<C>>>* = nullptr)
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

  /**
   * Returns the number of variables associated with this evaluator.
   */
  size_t GetNumElements() const {
    // TODO(ggould-tri) assumes that no index appears more than once in the
    // view, which is nowhere asserted (but seems assumed elsewhere).
    return vars_.size();
  }

  /**
   * Returns string representation of Binding.
   */
  std::string to_string() const {
    std::ostringstream os;
    os << *this;
    return os.str();
  }

  /**
   * Compare two bindings based on their evaluator pointers and the bound
   * variables.
   */
  bool operator==(const Binding<C>& other) const {
    if (this->evaluator().get() != other.evaluator().get()) {
      return false;
    }
    if (vars_.rows() != other.vars_.rows()) {
      return false;
    }
    for (int i = 0; i < vars_.rows(); ++i) {
      if (!vars_(i).equal_to(other.vars_(i))) {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const Binding<C>& other) const {
    return !((*this) == (other));
  }

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const Binding<C>& item) noexcept {
    using drake::hash_append;
    const EvaluatorBase* const base = item.evaluator().get();
    hash_append(hasher, reinterpret_cast<std::uintptr_t>(base));
    // We follow the pattern in
    // http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n3980.html#hash_append_vector
    // to append the hash for a std::vector, first to append all its elements,
    // and then append the vector size.
    for (int i = 0; i < item.variables().rows(); ++i) {
      hash_append(hasher, item.variables()(i));
    }
    hash_append(hasher, item.variables().rows());
  }

 private:
  std::shared_ptr<C> evaluator_;
  VectorXDecisionVariable vars_;
};

/**
 * Print out the Binding.
 */
template <typename C>
std::ostream& operator<<(std::ostream& os, const Binding<C>& binding) {
  return binding.evaluator()->Display(os, binding.variables());
}

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

namespace std {
template <typename C>
struct hash<drake::solvers::Binding<C>> : public drake::DefaultHash {};
}  // namespace std
