#pragma once

#include <type_traits>
#include <utility>
#include <vector>

#include "drake/common/symbolic.h"

using std::vector;

// TODOs:
// Overview / tutorial.
// Stop using std::function entirely.
// Short-circuit even T=Formula when there are no free vars, don't use vector.
// Add mutating_only overload for &eigen_matrix, not just &scalar.
// Silence the linter on `extra space in '""_if ('`.
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
  bool pred;
};
struct LazyIf {
  PredFuncList operator^(StdFunc&& func) {
    return PredFuncList(pred, std::move(func));
  }
  Formula pred;
};
struct MakerOfIf {
  ImmediateIf operator()(bool pred) { return ImmediateIf{pred}; }
  LazyIf operator()(const Formula& pred) { return LazyIf{pred}; }
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
  Else<Lambda> operator^(Lambda func) { return Else<Lambda>{std::move(func)}; }
};
struct MutatingOnly {
  MutatingOnly(const std::initializer_list<Expression*>& mutables) {
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
struct DummyMutatingOnly {
  void operator=(bool) {}
};
struct MakerOfMutatingOnly {
  template <typename T, typename... Args>
  DummyMutatingOnly operator()(T*, Args...) {
    return DummyMutatingOnly{};
  }
  template <typename... Args>
  MutatingOnly operator()(Expression* arg, Args... args) {
    return MutatingOnly{arg, args...};
  }
};

}  // namespace internal
namespace cond_literals {

inline internal::MakerOfMutatingOnly operator ""_mutating_only(const char*, std::size_t) { return {}; }
inline internal::MakerOfIf operator ""_if (const char*, std::size_t) { return {}; }
inline internal::MakerOfIf operator ""_elif (const char*, std::size_t) { return {}; }
inline internal::ElseKeyword operator ""_else (const char*, std::size_t) { return {}; }

}  // namespace cond_literals
}  // namespace symbolic
}  // namespace drake
