#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {
/**
 * The Mosek solver details after calling Solve() function. The user can call
 * MathematicalProgramResult::get_solver_details<MosekSolver>() to obtain the
 * details.
 */
struct MosekSolverDetails {
  /// The mosek optimization time. Please refer to MSK_DINF_OPTIMIZER_TIME in
  /// https://docs.mosek.com/9.2/capi/constants.html?highlight=msk_dinf_optimizer_time
  double optimizer_time{};
  /// The response code returned from mosek solver. Check
  /// https://docs.mosek.com/9.2/capi/response-codes.html for the meaning on the
  /// response code.
  int rescode{};
  /// The solution status after solving the problem. Check
  /// https://docs.mosek.com/9.2/capi/accessing-solution.html and
  /// https://docs.mosek.com/9.2/capi/constants.html#mosek.solsta for the
  /// meaning on the solution status.
  int solution_status{};
};

/**
 * An implementation of SolverInterface for the commercially-licensed MOSEK
 * solver (https://www.mosek.com/).
 *
 * The default build of Drake is not configured to use MOSEK, so therefore
 * SolverInterface::available() will return false. You must compile Drake
 * from source in order to link against MOSEK. For details, refer to the
 * documentation at https://drake.mit.edu/bazel.html#mosek.
 *
 * The MOSEKLM_LICENSE_FILE environment variable controls whether or not
 * SolverInterface::enabled() returns true.  Iff it is set to any non-empty
 * value, then the solver is enabled; otherwise, the solver is not enabled.
 *
 * @note Mosek only cares about the initial guess of integer variables. The
 * initial guess of continuous variables are not passed to MOSEK. If all the
 * integer variables are set to some integer values, then MOSEK will be forced
 * to compute the remaining continuous variable values as the initial guess.
 * (Mosek might change the values of the integer/binary variables in the
 * subsequent iterations.) If the specified integer solution is infeasible or
 * incomplete, MOSEK will simply ignore it. For more details, check
 * https://docs.mosek.com/9.2/capi/tutorial-mio-shared.html?highlight=initial
 *
 * Mosek supports many solver parameters. You can refer to the full list of
 * parameters in
 * https://docs.mosek.com/9.2/capi/param-groups.html#doc-param-groups. On top of
 * these parameters, we also provide the following additional parameters
 * 1. "writedata", set to a file name so that Mosek solver will write the
 *    optimization model to this file. check
 *    https://docs.mosek.com/9.2/capi/solver-io.html#saving-a-problem-to-a-file
 *    for more details. The supported file extensions are listed in
 *    https://docs.mosek.com/9.2/capi/supported-file-formats.html#doc-shared-file-formats.
 *    Set this parameter to "" if you don't want to write to a file. Default is
 *    not to write to a file.
 */
class MosekSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = MosekSolverDetails;

  MosekSolver();
  ~MosekSolver() final;

  /**
   * This type contains a valid MOSEK license environment, and is only to be
   * used from AcquireLicense().
   */
  class License;

  /**
   * This acquires a MOSEK license environment shared among all MosekSolver
   * instances; the environment will stay valid as long as at least one
   * shared_ptr returned by this function is alive.
   * Call this ONLY if you must use different MathematicalProgram
   * instances at different instances in time, and repeatedly acquiring the
   * license is costly (e.g., requires contacting a license server).
   * @return A shared pointer to a license environment that will stay valid
   * as long as any shared_ptr returned by this function is alive. If MOSEK is
   * not available in your build, this will return a null (empty) shared_ptr.
   * @throws std::exception if MOSEK is available but a license cannot be
   * obtained.
   */
  static std::shared_ptr<License> AcquireLicense();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  /// Returns true iff the environment variable MOSEKLM_LICENSE_FILE has been
  /// set to a non-empty value.
  static bool is_enabled();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
  static std::string UnsatisfiedProgramAttributes(const MathematicalProgram&);
  //@}

  // A using-declaration adds these methods into our class's Doxygen.
  using SolverBase::Solve;

 private:
  void DoSolve(const MathematicalProgram&, const Eigen::VectorXd&,
               const SolverOptions&, MathematicalProgramResult*) const final;

  // Note that this is mutable to allow latching the allocation of mosek_env_
  // during the first call of Solve() (which avoids grabbing a Mosek license
  // before we know that we actually want one).
  mutable std::shared_ptr<License> license_;
};

}  // namespace solvers
}  // namespace drake
