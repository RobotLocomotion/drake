#include <vector>
#include <utility>

#include "drake/common/symbolic.h"

using std::vector;

namespace drake {
namespace symbolic {
namespace internal {

using Func = std::function<void(void)>;

struct PredFuncList : public std::vector<std::pair<Formula, Func*>> {
  PredFuncList(const Formula& pred, Func&& func) {
    emplace_back(pred, &func);
  }
  PredFuncList operator||(PredFuncList&& other) {
    DRAKE_DEMAND(other.size() == 1);
    emplace_back(other.front());
    return std::move(*this);
  }  
};
struct ImmediateIf {
  bool operator^(Func&& func) {
    if (pred) { func(); }
    return pred;
  }
  bool pred;
};
struct LazyIf {
  PredFuncList operator^(Func&& func) {
    return PredFuncList(pred, std::move(func));
  }
  Formula pred;
};
struct MakerOfIf {
  ImmediateIf operator()(bool pred) { return ImmediateIf{pred}; }
  LazyIf operator()(const Formula& pred) { return LazyIf{pred}; }
};
struct Else {
  Func* func{};
  operator bool() {
    (*func)();
    return true;
  }
  friend PredFuncList operator||(PredFuncList&& other, const Else& self) {
    other.emplace_back(Formula::True(), self.func);
    return std::move(other);
  }
};
struct ElseKeyword {
  Else operator^(Func&& func) { return Else{&func}; }
};
template <typename T>
struct MutatingOnly {
  MutatingOnly(const std::initializer_list<T*>& mutables) {
    for (T* ptr : mutables) {
      mutable_refs.emplace_back(std::ref(*ptr));
    }
  }
  void operator=(bool) {}
  void operator=(PredFuncList&& clauses) {
    const int num_clauses = clauses.size();
    const int num_mutables = mutable_refs.size();
    // Save the originals.
    std::vector<T> e_orig;
    for (int i = 0; i < num_mutables; ++i) { e_orig.push_back(mutable_refs[i]); }
    // Evaluate each clause.
    std::vector<std::vector<Expression>> then_values;
    for (int j = 0; j < num_clauses; ++j) {
      // Invoke the consequent and save its outcome.
      (*clauses[j].second)();
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
  std::vector<std::reference_wrapper<T>> mutable_refs;
};
struct MakerOfMutatingOnly {
  template <typename T, typename... Args>
  MutatingOnly<T> operator()(T* arg, Args... args) {
    return MutatingOnly<T>{arg, args...};
  }
};

}  // namespace internal
namespace cond_literals {

internal::MakerOfMutatingOnly operator ""_mutating_only(const char*, std::size_t) { return {}; }
internal::MakerOfIf operator ""_if (const char*, std::size_t) { return {}; }
internal::MakerOfIf operator ""_elif (const char*, std::size_t) { return {}; }
internal::ElseKeyword operator ""_else (const char*, std::size_t) { return {}; }

}  // namespace cond_literals
}  // namespace symbolic
}  // namespace drake
