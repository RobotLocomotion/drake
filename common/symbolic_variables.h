#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <ostream>
#include <set>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/** Represents a set of variables.
 *
 * This class is based on std::set<Variable>. The intent is to add things that
 * we need including set-union (Variables::insert, operator+, operator+=),
 * set-minus (Variables::erase, operator-, operator-=), and subset/superset
 * checking functions (Variables::IsSubsetOf, Variables::IsSupersetOf,
 * Variables::IsStrictSubsetOf, Variables::IsStrictSupersetOf).
 */
class Variables {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Variables)

  typedef typename std::set<Variable>::size_type size_type;
  typedef typename std::set<Variable>::iterator iterator;
  typedef typename std::set<Variable>::const_iterator const_iterator;
  typedef typename std::set<Variable>::reverse_iterator reverse_iterator;
  typedef typename std::set<Variable>::const_reverse_iterator
      const_reverse_iterator;

  /** Default constructor. */
  Variables() = default;

  /** List constructor. */
  Variables(std::initializer_list<Variable> init);

  /** Constructs from an Eigen vector of variables. */
  explicit Variables(const Eigen::Ref<const VectorX<Variable>>& vec);

  /** Returns the number of elements. */
  [[nodiscard]] size_type size() const { return vars_.size(); }

  /** Checks if this set is empty or not. */
  [[nodiscard]] bool empty() const { return vars_.empty(); }

  /** Returns string representation of Variables. */
  [[nodiscard]] std::string to_string() const;

  /** Implements the @ref hash_append concept. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const Variables& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.vars_);
  }

  /** Returns an iterator to the beginning. */
  iterator begin() { return vars_.begin(); }
  /** Returns an iterator to the end. */
  iterator end() { return vars_.end(); }
  /** Returns an iterator to the beginning. */
  [[nodiscard]] const_iterator begin() const { return vars_.cbegin(); }
  /** Returns an iterator to the end. */
  [[nodiscard]] const_iterator end() const { return vars_.cend(); }
  /** Returns a const iterator to the beginning. */
  [[nodiscard]] const_iterator cbegin() const { return vars_.cbegin(); }
  /** Returns a const iterator to the end. */
  [[nodiscard]] const_iterator cend() const { return vars_.cend(); }
  /** Returns a reverse iterator to the beginning. */
  reverse_iterator rbegin() { return vars_.rbegin(); }
  /** Returns a reverse iterator to the end. */
  reverse_iterator rend() { return vars_.rend(); }
  /** Returns a reverse iterator to the beginning. */
  [[nodiscard]] const_reverse_iterator rbegin() const {
    return vars_.crbegin();
  }
  /** Returns a reverse iterator to the end. */
  [[nodiscard]] const_reverse_iterator rend() const { return vars_.crend(); }
  /** Returns a const reverse-iterator to the beginning. */
  [[nodiscard]] const_reverse_iterator crbegin() const {
    return vars_.crbegin();
  }
  /** Returns a const reverse-iterator to the end. */
  [[nodiscard]] const_reverse_iterator crend() const { return vars_.crend(); }

  /** Inserts a variable @p var into a set. */
  void insert(const Variable& var) { vars_.insert(var); }
  /** Inserts variables in [@p first, @p last) into a set. */
  template <class InputIt>
  void insert(InputIt first, InputIt last) {
    vars_.insert(first, last);
  }
  /** Inserts variables in @p vars into a set. */
  void insert(const Variables& vars) { vars_.insert(vars.begin(), vars.end()); }

  /** Erases @p key from a set. Return number of erased elements (0 or 1). */
  size_type erase(const Variable& key) { return vars_.erase(key); }

  /** Erases variables in @p vars from a set. Return number of erased
      elements ([0, vars.size()]). */
  size_type erase(const Variables& vars);

  /** Finds element with specific key. */
  iterator find(const Variable& key) { return vars_.find(key); }
  [[nodiscard]] const_iterator find(const Variable& key) const {
    return vars_.find(key);
  }

  /** Return true if @p key is included in the Variables. */
  [[nodiscard]] bool include(const Variable& key) const {
    return find(key) != end();
  }

  /** Return true if @p vars is a subset of the Variables. */
  [[nodiscard]] bool IsSubsetOf(const Variables& vars) const;
  /** Return true if @p vars is a superset of the Variables. */
  [[nodiscard]] bool IsSupersetOf(const Variables& vars) const;
  /** Return true if @p vars is a strict subset of the Variables. */
  [[nodiscard]] bool IsStrictSubsetOf(const Variables& vars) const;
  /** Return true if @p vars is a strict superset of the Variables. */
  [[nodiscard]] bool IsStrictSupersetOf(const Variables& vars) const;

  friend bool operator==(const Variables& vars1, const Variables& vars2);

  friend bool operator<(const Variables& vars1, const Variables& vars2);

  friend std::ostream& operator<<(std::ostream&, const Variables& vars);

  friend Variables intersect(const Variables& vars1, const Variables& vars2);

 private:
  /* Constructs from std::set<Variable>. */
  explicit Variables(std::set<Variable> vars);

  std::set<Variable> vars_;
};

/** Updates @p var1 with the result of set-union(@p var1, @p var2). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator+=(Variables& vars1, const Variables& vars2);
/** Updates @p vars with the result of set-union(@p vars, { @p var }). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator+=(Variables& vars, const Variable& var);
/** Returns set-union of @p var1 and @p var2. */
Variables operator+(Variables vars1, const Variables& vars2);
/** Returns set-union of @p vars and {@p var}. */
Variables operator+(Variables vars, const Variable& var);
/** Returns set-union of {@p var} and @p vars. */
Variables operator+(const Variable& var, Variables vars);

/** Updates @p var1 with the result of set-minus(@p var1, @p var2). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator-=(Variables& vars1, const Variables& vars2);
/** Updates @p vars with the result of set-minus(@p vars, {@p var}). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
Variables operator-=(Variables& vars, const Variable& var);
/** Returns set-minus(@p var1, @p vars2). */
Variables operator-(Variables vars1, const Variables& vars2);
/** Returns set-minus(@p vars, { @p var }). */
Variables operator-(Variables vars, const Variable& var);

/** Returns the intersection of @p vars1 and @p vars2.
 *
 * This function has a time complexity of `O(N₁ + N₂)` where `N₁` and `N₂` are
 * the size of @p vars1 and @p vars2 respectively.
 */
Variables intersect(const Variables& vars1, const Variables& vars2);

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Variables>. */
template <>
struct hash<drake::symbolic::Variables> : public drake::DefaultHash {};
}  // namespace std
