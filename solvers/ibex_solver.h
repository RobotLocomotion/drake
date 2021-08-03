#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

class IbexSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IbexSolver)

  IbexSolver();
  ~IbexSolver() final;

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  //@}

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;

  // void DoOptimize() const;

  // void DoCheckSatisfiability() const;
};

}  // namespace solvers
}  // namespace drake
