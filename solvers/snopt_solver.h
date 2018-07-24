#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {

class SnoptSolver : public MathematicalProgramSolverInterface  {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SnoptSolver)

  SnoptSolver() = default;
  ~SnoptSolver() override = default;

  // This solver is implemented in various pieces depending on if
  // SNOPT was available during compilation.
  bool available() const override;

  SolutionResult Solve(MathematicalProgram& prog) const override;

  /**
   * Calls SNOPT to solve the optimization program, and returns the snopt status
   * together with the value of the decision variables at solver termination.
   * @param prog[in] Stores the optimization program.
   * @param x_init[in] The initial value for all the decision variables.
   * @param snopt_options_string[in] The string type options for SNOPT. The
   * supported options for SNOPT is listed in section 7 "optional parameters"
   * of SNOPT7 user's guide.
   * @param snopt_options_int[in] The integer type options for SNOPT.
   * @param snopt_options_double[in] The double type options for SNOPT.
   * @param snopt_status[out] The status of SNOPT solver at the exit. Please
   * refer to section 8.6 Exit conditions of User's Guide for SNOPT version 7:
   * Software for Large-Scale Nonlinear Programming for the meaning of the
   * status. 
   * @param objective[out] The value of the objective function at solver
   * termination, variable_values contain
   * @param x_val[out] The value of the decision variables at solver
   * termination. Note that the number of rows in x_val should be
   * prog.num_vars().
   * @note This function DOES NOT change the initial guess stored inside
   * @p prog. One usage of DoSolve() function is calling several SNOPT solvers
   * in multiple threads, with different initial guesses and/or paramters. Check
   * out the test "MultiThreadTest" in solvers/test/snopt_solver_test for an
   * example.
   */
  void  DoSolve(
      const MathematicalProgram& prog,
      const Eigen::Ref<const Eigen::VectorXd>& x_init,
      const std::map<std::string, std::string>& snopt_options_string,
      const std::map<std::string, int>& snopt_options_int,
      const std::map<std::string, double>& snopt_options_double,
      int* snopt_status,
      double* objective,
      EigenPtr<Eigen::VectorXd> x_val) const;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();
};

}  // namespace solvers
}  // namespace drake
