#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/// The Clarabel solver details after calling the Solve() function. The user can
/// call MathematicalProgramResult::get_solver_details<ClarabelSolver>() to
/// obtain the details.
struct ClarabelSolverDetails {
  /// The solve time inside Clarabel in seconds.
  double solve_time{};
  /// Number of iterations in Clarabel.
  int iterations{};
  /// The status from Clarabel.
  std::string status{};
};

/// An interface to wrap Clarabel https://github.com/oxfordcontrol/Clarabel.cpp
class ClarabelSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ClarabelSolver);

  /// Type of details stored in MathematicalProgramResult.
  using Details = ClarabelSolverDetails;

  ClarabelSolver();
  ~ClarabelSolver() final;

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve2(const MathematicalProgram&, const Eigen::VectorXd&,
                internal::SpecificOptions*,
                MathematicalProgramResult*) const final;
};
}  // namespace solvers
}  // namespace drake
