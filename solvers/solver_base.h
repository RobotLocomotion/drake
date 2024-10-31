#pragma once

#include <functional>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_interface.h"
#include "drake/solvers/solver_options.h"
#include "drake/solvers/specific_options.h"

namespace drake {
namespace solvers {

/** Abstract base class used by implementations of individual solvers. */
class SolverBase : public SolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SolverBase);

  ~SolverBase() override;

  /** Like SolverInterface::Solve(), but the result is a return value instead of
  an output argument. */
  MathematicalProgramResult Solve(
      const MathematicalProgram& prog,
      const std::optional<Eigen::VectorXd>& initial_guess = std::nullopt,
      const std::optional<SolverOptions>& solver_options = std::nullopt) const;

  // Implement the SolverInterface methods.
  void Solve(const MathematicalProgram&, const std::optional<Eigen::VectorXd>&,
             const std::optional<SolverOptions>&,
             MathematicalProgramResult*) const override;
  bool available() const override;
  bool enabled() const override;
  SolverId solver_id() const final { return solver_id_; }
  bool AreProgramAttributesSatisfied(const MathematicalProgram&) const override;
  std::string ExplainUnsatisfiedProgramAttributes(
      const MathematicalProgram&) const override;

 protected:
  /** Constructs a SolverBase with the given default implementations of the
  solver_id(), available(), enabled(), AreProgramAttributesSatisfied(), and
  ExplainUnsatisfiedProgramAttributes() methods. Typically, the subclass will
  simply pass the address of its static method, e.g., `&available`, for these
  functors. Any of the functors can be nullptr, in which case the subclass must
  override the matching virtual method instead, except for `explain_unsatisfied`
  which already has a default implementation. */
  SolverBase(const SolverId& id, std::function<bool()> available,
             std::function<bool()> enabled,
             std::function<bool(const MathematicalProgram&)> are_satisfied,
             std::function<std::string(const MathematicalProgram&)>
                 explain_unsatisfied = nullptr);

  /** Hook for subclasses to implement Solve. Prior to the SolverBase's call to
  this method, the solver's availability and capabilities vs the program
  attributes have already been checked, and the result's set_solver_id() and
  set_decision_variable_index() have already been set. The options and initial
  guess are already merged, i.e., the DoSolve implementation should ignore
  prog's solver options and prog's initial guess. */
  virtual void DoSolve(const MathematicalProgram& prog,
                       const Eigen::VectorXd& initial_guess,
                       const SolverOptions& merged_options,
                       MathematicalProgramResult* result) const;

  /** (Internal use only) Like DoSolve() but using SpecificOptions instead of
  SolverOptions. The "2" here means "version 2", to help disabiguate various
  DoSolve overloads.

  By default, DoSolve() will delegate to DoSolve2() and DoSolve2() will throw.
  Subclasses should override exactly one of the various DoSolver methods.
  Solvers wrapped inside Drake should override DoSolve2(); solvers wrapped in
  downstream projects must override DoSolve() since this function is marked as
  internal use only. */
  virtual void DoSolve2(const MathematicalProgram& prog,
                        const Eigen::VectorXd& initial_guess,
                        internal::SpecificOptions* options,
                        MathematicalProgramResult* result) const;

 private:
  SolverId solver_id_;
  std::function<bool()> default_available_;
  std::function<bool()> default_enabled_;
  std::function<bool(const MathematicalProgram&)> default_are_satisfied_;
  std::function<std::string(const MathematicalProgram&)>
      default_explain_unsatisfied_;
};

}  // namespace solvers
}  // namespace drake
