#pragma once

#include <initializer_list>
#include <iostream>
#include <unordered_map>

#include "drake/common/symbolic_variable.h"
#include "drake/drakeCommon_export.h"

namespace drake {

namespace symbolic {

/** \brief Represent a symbolic form of an environment (mapping from a variable
 * to a value).
 */
class DRAKECOMMON_EXPORT Environment {
 public:
  typedef typename drake::symbolic::Variable key_type;
  typedef double mapped_type;
  typedef typename std::unordered_map<key_type, mapped_type> map;

 private:
  map map_;

 public:
  typedef typename map::value_type
      value_type; /** std::pair<key_type, mapped_type> */
  typedef typename map::iterator iterator;
  typedef typename map::const_iterator const_iterator;

  /** Default constructor. */
  Environment() = default;

  /** Move-construct a set from an rvalue. */
  Environment(Environment&& e) = default;

  /** Copy-construct a set from an lvalue. */
  Environment(Environment const& e) = default;

  /** Move-assign a set from an rvalue. */
  Environment& operator=(Environment&& e) = default;

  /** Copy-assign a set from an lvalue. */
  Environment& operator=(Environment const& e) = default;

  /** List constructor. */
  Environment(std::initializer_list<value_type> init);

  iterator begin() { return map_.begin(); }
  iterator end() { return map_.end(); }
  const_iterator begin() const { return map_.cbegin(); }
  const_iterator end() const { return map_.cend(); }
  const_iterator cbegin() const { return map_.cbegin(); }
  const_iterator cend() const { return map_.cend(); }

  void insert(key_type const & key, mapped_type const & elem);

  bool empty() const { return map_.empty(); }
  size_t size() const { return map_.size(); }

  iterator find(key_type const& key) { return map_.find(key); }
  const_iterator find(key_type const& key) const { return map_.find(key); }

  friend DRAKECOMMON_EXPORT std::ostream& operator<<(std::ostream& os,
                                                     Environment const& env);
};
}  // namespace symbolic
}  // namespace drake

namespace std {
/** Provide std::to_string for drake::symbolic::Environment. */
DRAKECOMMON_EXPORT std::string to_string(
    drake::symbolic::Environment const& env);
}  // namespace std
