#pragma once

#include <cstddef>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <set>
#include <string>

#include "drake/common/symbolic_variable.h"
#include "drake/drakeCommon_export.h"

namespace drake {

namespace symbolic {

/** \brief Represent a set of symbolic variables.
 *
 * On top of std::set<Variable>, this class provides set-union
 * (Variables::insert, operator+, operator+=), set-minus (Variables::erase,
 * operator-, operator-=), and subset/superset checking functions
 * (Variables::IsSubsetOf, Variables::IsSupersetOf, Variables::IsStrictSubsetOf,
 * Variables::IsStrictSupersetOf).
 */

class DRAKECOMMON_EXPORT Variables {
 public:
  typedef typename drake::symbolic::Variable key_type;
  typedef typename drake::symbolic::Variable value_type;
  typedef typename std::set<key_type> set;

 private:
  set vars_;

 public:
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
  Variables(Variables const& e) = default;

  /** Move-assign a set from an rvalue. */
  Variables& operator=(Variables&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Variables& operator=(Variables const& e) = default;

  /** List constructor. */
  Variables(std::initializer_list<value_type> init);

  size_t get_hash() const;

  size_type size() const { return vars_.size(); }

  iterator begin() { return vars_.begin(); }
  iterator end() { return vars_.end(); }
  const_iterator begin() const { return vars_.cbegin(); }
  const_iterator end() const { return vars_.cend(); }
  const_iterator cbegin() const { return vars_.cbegin(); }
  const_iterator cend() const { return vars_.cend(); }
  reverse_iterator rbegin() { return vars_.rbegin(); }
  reverse_iterator rend() { return vars_.rend(); }
  const_reverse_iterator rbegin() const { return vars_.crbegin(); }
  const_reverse_iterator rend() const { return vars_.crend(); }
  const_reverse_iterator crbegin() const { return vars_.crbegin(); }
  const_reverse_iterator crend() const { return vars_.crend(); }

  /** Insert a variable \p var into a set */
  void insert(value_type const& var) { vars_.insert(var); }
  /** Insert variables in [\p first, \p last) into a set */
  template <class InputIt>
  void insert(InputIt first, InputIt last) {
    vars_.insert(first, last);
  }
  /** Insert variables in \p vars into a set */
  void insert(Variables const& vars) { vars_.insert(vars.begin(), vars.end()); }

  /** Erase \p key from a set. Return number of erased elements (0 or 1). */
  size_type erase(key_type const& key) { return vars_.erase(key); }

  /** Erase variables in \p vars from a set. Return number of erased
      elements ([0, vars.size()]) */
  size_type erase(Variables const& vars);

  iterator find(key_type const& key) { return vars_.find(key); }
  const_iterator find(key_type const& key) const { return vars_.find(key); }
  bool include(key_type const& key) const { return find(key) != end(); }

  bool IsSubsetOf(Variables const& vars) const;
  bool IsSupersetOf(Variables const& vars) const;
  bool IsStrictSubsetOf(Variables const& vars) const;
  bool IsStrictSupersetOf(Variables const& vars) const;

  friend DRAKECOMMON_EXPORT bool operator==(Variables const& vars1,
                                            Variables const& vars2);

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream&,
                                                     Variables const& vars);
};

/** Update \p var1 with the result of set-union(\p var1, \p var2). */
DRAKECOMMON_EXPORT Variables operator+=(Variables& vars1,
                                        Variables const& vars2);
/** Update \p vars with the result of set-union(\p vars, { \p var }). */
DRAKECOMMON_EXPORT Variables operator+=(Variables& vars, Variable const& var);
/** Return set-union of \p var1 and \p var2 */
DRAKECOMMON_EXPORT Variables operator+(Variables vars1, Variables const& vars2);
/** Return set-union of \p vars and {\p var} */
DRAKECOMMON_EXPORT Variables operator+(Variables vars, Variable const& var);
/** Return set-union of {\p var} and \p vars */
DRAKECOMMON_EXPORT Variables operator+(Variable const& var, Variables vars);

/** Update \p var1 with the result of set-minus(\p var1, \p var2). */
DRAKECOMMON_EXPORT Variables operator-=(Variables& vars1,
                                        Variables const& vars2);
/** Update \p vars with the result of set-minus(\p vars, {\p var}). */
DRAKECOMMON_EXPORT Variables operator-=(Variables& vars, Variable const& var);
/** Return set-minus(\p var1, \p vars2). */
DRAKECOMMON_EXPORT Variables operator-(Variables vars1, Variables const& vars2);
/** Return set-minus(\p vars, { \p var }). */
DRAKECOMMON_EXPORT Variables operator-(Variables vars, Variable const& var);

}  // namespace symbolic
}  // namespace drake

/** Provide std::hash<drake::symbolic::Variable>. */
namespace std {
template <>
struct hash<drake::symbolic::Variables> {
  size_t operator()(drake::symbolic::Variables const& vars) const {
    return vars.get_hash();
  }
};

/** Provide std::to_string for drake::symbolic::Variables. */
DRAKECOMMON_EXPORT std::string to_string(
    drake::symbolic::Variables const& vars);
}  // namespace std
