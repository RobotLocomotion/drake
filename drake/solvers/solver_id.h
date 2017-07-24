#pragma once

#include <functional>
#include <ostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/reinit_after_move.h"

namespace drake {
namespace solvers {

/// Identifies a MathematicalProgramSolverInterface implementation.
///
/// A moved-from instance is guaranteed to be empty and will not compare equal
/// to any non-empty ID.
class SolverId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverId)
  ~SolverId() = default;

  /// Constructs a specific, known solver type.  Internally, a hidden
  /// identifier is allocated and assigned to this instance; all instances that
  /// share an identifier (including copies of this instance) are considered
  /// equal.  The solver names are not enforced to be unique, though we
  /// recommend that they remain so in practice.
  explicit SolverId(std::string name);

  const std::string& name() const { return name_; }

 private:
  friend bool operator==(const SolverId&, const SolverId&);
  friend bool operator!=(const SolverId&, const SolverId&);
  friend struct std::less<SolverId>;

  reinit_after_move<int> id_;
  std::string name_;
};

bool operator==(const SolverId&, const SolverId&);
bool operator!=(const SolverId&, const SolverId&);
std::ostream& operator<<(std::ostream&, const SolverId&);

}  // namespace solvers
}  // namespace drake

namespace std {
/* Provides std::less<drake::solvers::SolverId>. */
template <>
struct less<drake::solvers::SolverId> {
  bool operator()(const drake::solvers::SolverId& lhs,
                  const drake::solvers::SolverId& rhs) const {
    return lhs.id_ < rhs.id_;
  }
};
}  // namespace std
