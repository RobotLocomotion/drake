#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_type.h"

namespace drake {
namespace solvers {

/// Converts between SolverType and SolverId.  This class only exists for
/// backwards compatiblity, and should not be used in new code.
class SolverTypeConverter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverTypeConverter);
  SolverTypeConverter() = delete;
  ~SolverTypeConverter() = delete;

  /// Converts the given type to its matching ID.
  static SolverId TypeToId(SolverType);

  /// Converts the given ID to its matching type, iff the type matches one of
  /// SolverType's known values.
  static std::optional<SolverType> IdToType(SolverId);
};

}  // namespace solvers
}  // namespace drake
