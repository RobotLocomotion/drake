#pragma once

#include <ostream>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/reinit_after_move.h"

namespace drake {
namespace solvers {

/// Identifies a MathematicalProgramSolverInterface implementation.
///
/// A moved-from instance is guaranteed to be identical to a
/// default-constructed instance.
class SolverId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverId)
  ~SolverId() = default;

  /// Constructs a default "unknown" solver type.  All unknown instances are
  /// considered equal.
  SolverId();

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

  reinit_after_move<int> id_;
  std::string name_;
};

bool operator==(const SolverId&, const SolverId&);
bool operator!=(const SolverId&, const SolverId&);
std::ostream& operator<<(std::ostream&, const SolverId&);

}  // namespace solvers
}  // namespace drake
