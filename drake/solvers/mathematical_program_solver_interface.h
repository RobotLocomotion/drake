#pragma once
#include <string>

namespace drake {
namespace solvers {
class MathematicalProgram;

enum SolutionSummary {
  kSolutionFound = 0,
  kInvalidInput = -1,
  kInfeasibleConstraints = -2,
  kUnknownError = -3
};

/// This is the returned type of MathematicalProgramSolverInterface::Solve().
/// It contains the information generated by each solver, when solving the
/// optimization problem.
class MathematicalProgramSolverResult {
 public:
  explicit MathematicalProgramSolverResult(SolutionSummary summary)
      : summary_(summary) {}

  virtual ~MathematicalProgramSolverResult() = default;

  /** MathematicalProgramSolverResult is not copyable. */
  MathematicalProgramSolverResult(const MathematicalProgramSolverResult& rhs) =
      delete;

  /** MathematicalProgramSolverResult is not assignable. */
  MathematicalProgramSolverResult& operator=(
      const MathematicalProgramSolverResult& rhs) = delete;

  /** MathematicalProgramSolverResult is not movable. */
  MathematicalProgramSolverResult(MathematicalProgramSolverResult&& rhs) =
      delete;

  /** MathematicalProgramSolverResult is not move-assignable.*/
  MathematicalProgramSolverResult& operator=(
      MathematicalProgramSolverResult&& rhs) = delete;

  /** Getter for summary. */
  SolutionSummary summary() const { return summary_; }

 private:
  const SolutionSummary summary_;
};

/// Interface used by implementations of individual solvers.
class MathematicalProgramSolverInterface {
 public:
  virtual ~MathematicalProgramSolverInterface() = default;

  /// Returns true iff this solver was enabled at compile-time.
  bool available() const { return available_impl(); }

  /// Returns the name of the solver.
  std::string SolverName() const { return SolverName_impl(); }

  /// Sets values for the decision variables on the given MathematicalProgram
  /// @p prog, or:
  ///  * If no solver is available, throws std::runtime_error
  ///  * If the solver returns an error, returns a nonzero SolutionSummary.
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  std::unique_ptr<MathematicalProgramSolverResult> Solve(
      MathematicalProgram* const prog) const {
    return std::unique_ptr<MathematicalProgramSolverResult>(Solve_impl(prog));
  }

 protected:
  virtual bool available_impl() const = 0;

  virtual std::string SolverName_impl() const = 0;

  /** This function should be handled with caution, since it returns a raw
   * pointer, pointing to the newly allocated memory on the heap. It is then
   * called by the public Solve function, that wraps this raw pointer as a
   * unique pointer.
   * @return The raw pointer to the newly constructed result.
   */
  virtual MathematicalProgramSolverResult* Solve_impl(
      MathematicalProgram* const prog) const = 0;
};
}  // namespace solvers
}  // namespace drake
