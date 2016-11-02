#pragma once

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <ostream>
#include <set>
#include <string>

#include "drake/common/drake_export.h"
#include "drake/common/symbolic_variable.h"

namespace drake {

namespace symbolic {

/** Represents a set of symbolic variables.
 *
 * This class is based on std::set<Variable>. The intent is to add things that
 * we need including set-union (Variables::insert, operator+, operator+=),
 * set-minus (Variables::erase, operator-, operator-=), and subset/superset
 * checking functions (Variables::IsSubsetOf, Variables::IsSupersetOf,
 * Variables::IsStrictSubsetOf, Variables::IsStrictSupersetOf).
 */

class DRAKE_EXPORT Variables {
 public:
  typedef typename drake::symbolic::Variable key_type;
  typedef typename drake::symbolic::Variable value_type;
  typedef typename std::set<key_type> set;
  typedef typename set::size_type size_type;
  typedef typename set::iterator iterator;
  typedef typename set::const_iterator const_iterator;
  typedef typename set::reverse_iterator reverse_iterator;
  typedef typename set::const_reverse_iterator const_reverse_iterator;

  /** Default constructor. */
  Variables() = default;

  /** Move-construct a set from an rvalue. */
  Variables(Variables&& e) = default;

  /** Copy-construct a set from an lvalue. */
  Variables(const Variables& e) = default;

  /** Move-assign a set from an rvalue. */
  Variables& operator=(Variables&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Variables& operator=(const Variables& e) = default;

  /** List constructor. */
  Variables(std::initializer_list<value_type> init);

  /** Returns hash value. */
  size_t get_hash() const;

  /** Returns the number of elements. */
  size_type size() const { return vars_.size(); }

  /** Returns string representation of Variables. */
  std::string to_string() const;

  /** Returns an iterator to the beginning. */
  iterator begin() { return vars_.begin(); }
  /** Returns an iterator to the end. */
  iterator end() { return vars_.end(); }
  /** Returns an iterator to the beginning. */
  const_iterator begin() const { return vars_.cbegin(); }
  /** Returns an iterator to the end. */
  const_iterator end() const { return vars_.cend(); }
  /** Returns a const iterator to the beginning. */
  const_iterator cbegin() const { return vars_.cbegin(); }
  /** Returns a const iterator to the end. */
  const_iterator cend() const { return vars_.cend(); }
  /** Returns a reverse iterator to the beginning. */
  reverse_iterator rbegin() { return vars_.rbegin(); }
  /** Returns a reverse iterator to the end. */
  reverse_iterator rend() { return vars_.rend(); }
  /** Returns a reverse iterator to the beginning. */
  const_reverse_iterator rbegin() const { return vars_.crbegin(); }
  /** Returns a reverse iterator to the end. */
  const_reverse_iterator rend() const { return vars_.crend(); }
  /** Returns a const reverse-iterator to the beginning. */
  const_reverse_iterator crbegin() const { return vars_.crbegin(); }
  /** Returns a const reverse-iterator to the end. */
  const_reverse_iterator crend() const { return vars_.crend(); }

  /** Inserts a variable \p var into a set. */
  void insert(const value_type& var) { vars_.insert(var); }
  /** Inserts variables in [\p first, \p last) into a set. */
  template <class InputIt>
  void insert(InputIt first, InputIt last) {
    vars_.insert(first, last);
  }
  /** Inserts variables in \p vars into a set. */
  void insert(const Variables& vars) { vars_.insert(vars.begin(), vars.end()); }

  /** Erases \p key from a set. Return number of erased elements (0 or 1). */
  size_type erase(const key_type& key) { return vars_.erase(key); }

  /** Erases variables in \p vars from a set. Return number of erased
      elements ([0, vars.size()]). */
  size_type erase(const Variables& vars);

  /** Finds element with specific key. */
  iterator find(const key_type& key) { return vars_.find(key); }
  const_iterator find(const key_type& key) const { return vars_.find(key); }

  /** Return true if \p key is included in the Variables. */
  bool include(const key_type& key) const { return find(key) != end(); }

  /** Return true if \p vars is a subset of the Variables. */
  bool IsSubsetOf(const Variables& vars) const;
  /** Return true if \p vars is a superset of the Variables. */
  bool IsSupersetOf(const Variables& vars) const;
  /** Return true if \p vars is a strict subset of the Variables. */
  bool IsStrictSubsetOf(const Variables& vars) const;
  /** Return true if \p vars is a strict superset of the Variables. */
  bool IsStrictSupersetOf(const Variables& vars) const;

  friend DRAKE_EXPORT bool operator==(const Variables& vars1,
                                      const Variables& vars2);

  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
                                               const Variables& vars);

 private:
  set vars_;
};

/** Updates \p var1 with the result of set-union(\p var1, \p var2). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Variables operator+=(Variables& vars1, const Variables& vars2);
/** Updates \p vars with the result of set-union(\p vars, { \p var }). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Variables operator+=(Variables& vars, const Variable& var);
/** Returns set-union of \p var1 and \p var2. */
DRAKE_EXPORT Variables operator+(Variables vars1, const Variables& vars2);
/** Returns set-union of \p vars and {\p var}. */
DRAKE_EXPORT Variables operator+(Variables vars, const Variable& var);
/** Returns set-union of {\p var} and \p vars. */
DRAKE_EXPORT Variables operator+(const Variable& var, Variables vars);

/** Updates \p var1 with the result of set-minus(\p var1, \p var2). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Variables operator-=(Variables& vars1, const Variables& vars2);
/** Updates \p vars with the result of set-minus(\p vars, {\p var}). */
// NOLINTNEXTLINE(runtime/references) per C++ standard signature.
DRAKE_EXPORT Variables operator-=(Variables& vars, const Variable& var);
/** Returns set-minus(\p var1, \p vars2). */
DRAKE_EXPORT Variables operator-(Variables vars1, const Variables& vars2);
/** Returns set-minus(\p vars, { \p var }). */
DRAKE_EXPORT Variables operator-(Variables vars, const Variable& var);

}  // namespace symbolic
}  // namespace drake

namespace std {
/* Provides std::hash<drake::symbolic::Variables>. */
template <>
struct hash<drake::symbolic::Variables> {
  size_t operator()(const drake::symbolic::Variables& vars) const {
    return vars.get_hash();
  }
};
}  // namespace std
