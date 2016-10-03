#pragma once

#include <cstddef>
#include <functional>
#include <set>
#include <string>

#include "drake/drakeCommon_export.h"

namespace drake {
namespace symbolic {

/** \brief Represent a symbolic variable. */
class DRAKECOMMON_EXPORT Variable {
 private:
  size_t id_;        /** Unique identifier. */
  std::string name_; /** Name of variable */
  /// Next variable ID which will be assigned to a new variable.
  static size_t next_id_;

 public:
  /** Default constructor. */
  Variable() = default;

  /** Construct a variable with a string . */
  explicit Variable(std::string const& name);

  /** Move-construct a set from an rvalue. */
  Variable(Variable&& f) = default;

  /** Copy-construct a set from an lvalue. */
  Variable(Variable const& f) = default;

  /** Move-assign a set from an rvalue. */
  Variable& operator=(Variable&& f) = default;

  /** Copy-assign a set from an lvalue. */
  Variable& operator=(Variable const& f) = default;

  size_t get_id() const;
  size_t get_hash() const { return std::hash<size_t>{}(id_); }
  std::string get_name() const;

  DRAKECOMMON_EXPORT friend std::ostream& operator<<(std::ostream& os,
                                                     Variable const& var);
};

/// Compare two variables based on their ID values
DRAKECOMMON_EXPORT bool operator<(Variable const& lhs, Variable const& rhs);

/// Check equality
DRAKECOMMON_EXPORT bool operator==(Variable const& lhs, Variable const& rhs);

}  // namespace symbolic
}  // namespace drake

/** Provide std::hash<drake::symbolic::Variable>. */
namespace std {
template <>
struct hash<drake::symbolic::Variable> {
  size_t operator()(drake::symbolic::Variable const& v) const {
    return v.get_hash();
  }
};

/** Provide std::to_string for drake::symbolic::Variable. */
DRAKECOMMON_EXPORT std::string to_string(drake::symbolic::Variable const& v);
}  // namespace std
