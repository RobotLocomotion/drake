#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_type.h"

namespace drake {
namespace solvers {

/// Converts between SolverType and SolverId.
class SolverTypeConverter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverTypeConverter);
  SolverTypeConverter() = delete;
  ~SolverTypeConverter() = delete;

  /// Converts the given type to its matching ID.
  static SolverId TypeToId(SolverType);

  /// Converts the given ID to its matching type, iff the type matches one of
  /// SolverType's known values.
  static optional<SolverType> IdToType(SolverId);
};

}  // namespace solvers
}  // namespace drake
