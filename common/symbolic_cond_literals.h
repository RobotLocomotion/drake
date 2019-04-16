#pragma once

#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/symbolic.h"

// TODOs:
// Overview / tutorial.
// Stop using std::function entirely.
// Short-circuit even T=Formula when there are no free vars, don't use vector.
// Add lazy_assign overload for &eigen_matrix, not just &scalar.
// Silence the linter on `extra space in 'lazy_if ('`.
// Silence the linter on `using namespace ..._literals`.

namespace drake {
namespace symbolic {
namespace internal {

using StdFunc = std::function<void(void)>;

struct PredFuncList : public std::vector<std::pair<Formula, StdFunc>> {
  PredFuncList(const Formula& pred, StdFunc&& func) {
    emplace_back(pred, func);
  }
  PredFuncList operator||(PredFuncList&& other) {
    DRAKE_DEMAND(other.size() == 1);
    emplace_back(std::move(other.front()));
    return *this;
  }
};
struct ImmediateIf {
  template <typename Lambda>
  bool operator^(Lambda&& func) {
    if (pred) { func(); }
    return pred;
  }
  bool pred{};
};
struct LazyIf {
  PredFuncList operator^(StdFunc&& func) {
    return PredFuncList(pred, std::move(func));
  }
  Formula pred;
};
template <typename Lambda>
struct Else {
  Lambda func;
  operator bool() {
    func();
    return true;
  }
  friend PredFuncList operator||(PredFuncList&& other, Else&& self) {
    other.emplace_back(Formula::True(), StdFunc(self.func));
    return other;
  }
};
struct ElseKeyword {
  template <typename Lambda>
  Else<Lambda> operator^(Lambda func) const { return Else<Lambda>{std::move(func)}; }
};
struct AssignCapture {
  AssignCapture(const std::initializer_list<Expression*>& mutables) {
    for (Expression* ptr : mutables) {
      mutable_refs.emplace_back(std::ref(*ptr));
    }
  }
  void operator=(PredFuncList&& clauses) {
    const int num_clauses = clauses.size();
    const int num_mutables = mutable_refs.size();
    // Save the originals.
    std::vector<Expression> e_orig;
    for (int i = 0; i < num_mutables; ++i) { e_orig.push_back(mutable_refs[i]); }
    // Evaluate each clause.
    std::vector<std::vector<Expression>> then_values;
    for (int j = 0; j < num_clauses; ++j) {
      // Invoke the consequent and save its outcome.
      clauses[j].second();
      then_values.emplace_back(mutable_refs.begin(), mutable_refs.end());
      // Restore the originals.
      for (int i = 0; i < num_mutables; ++i) { mutable_refs[i].get() = e_orig[i]; }
    }
    // Write back the final phi values (reverse order, so that first one wins).
    for (int j = num_clauses - 1; j >= 0; --j) {
      for (int i = 0; i < num_mutables; ++i) {
        const Formula& pred = clauses[j].first;
        mutable_refs[i].get() = if_then_else(pred, then_values[j][i], mutable_refs[i]);
      }
    }
  }
  std::vector<std::reference_wrapper<Expression>> mutable_refs;
};

template <typename T>
struct DummyAssignCapture {
  DummyAssignCapture(const std::initializer_list<T*>& mutables) {}
  void operator=(bool) {}
};

template <typename T, typename... Args>
auto lazy_assign(T* arg, Args... args) {
  using ResultType = std::conditional_t<
      scalar_predicate<T>::is_bool,
      internal::DummyAssignCapture<T>,
      internal::AssignCapture>;
  return ResultType{arg, args...};
}

ImmediateIf lazy_if(bool pred) {
  return ImmediateIf{pred};
}

LazyIf lazy_if(const Formula& pred) {
  return LazyIf{pred};
}

ImmediateIf lazy_elif(bool pred) {
  return ImmediateIf{pred};
}

LazyIf lazy_elif(const Formula& pred) {
  return LazyIf{pred};
}

}  // namespace internal
namespace branching_literals {

using internal::lazy_assign;
using internal::lazy_if;
using internal::lazy_elif;
const internal::ElseKeyword lazy_else;

}  // namespace cond_literals
}  // namespace symbolic
}  // namespace drake
