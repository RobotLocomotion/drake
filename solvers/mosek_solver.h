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
  /// https://docs.mosek.com/9.0/capi/constants.html?highlight=msk_dinf_optimizer_time
  double optimizer_time{};
  /// The response code returned from mosek solver. Check
  /// https://docs.mosek.com/9.0/capi/response-codes.html for the meaning on the
  /// response code.
  int rescode{};
  /// The solution status after solving the problem. Check
  /// https://docs.mosek.com/9.0/capi/accessing-solution.html and
  /// https://docs.mosek.com/9.0/capi/constants.html#mosek.solsta for the
  /// meaning on the solution status.
  int solution_status{};
};

/**
 * @note Mosek only cares about the initial guess of integer variables. The
 * initial guess of continuous variables are not passed to MOSEK. If all the
 * integer variables are set to some integer values, then MOSEK will be forced
 * to compute the remaining continuous variable values as the initial guess.
 * (Mosek might change the values of the integer/binary variables in the
 * subsequent iterations.) If the specified integer solution is infeasible or
 * incomplete, MOSEK will simply ignore it. For more details, check
 * https://docs.mosek.com/9.0/capi/tutorial-mio-shared.html?highlight=initial
 */
class MosekSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  /// Type of details stored in MathematicalProgramResult.
  using Details = MosekSolverDetails;

  MosekSolver();
  ~MosekSolver() final;

  /**
   * Control stream logging. Refer to
   * https://docs.mosek.com/9.0/capi/solver-io.html for more details.
   * @param flag Set to true if the user want to turn on stream logging.
   * @param log_file If the user wants to output the logging to a file, then
   * set @p log_file to the name of that file. If the user wants to output the
   * logging to the console, then set log_file to empty string.
   */
  void set_stream_logging(bool flag, const std::string& log_file) {
    stream_logging_ = flag;
    log_file_ = log_file;
  }

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
   * @throws std::runtime_error if MOSEK is available but a license cannot be
   * obtained.
   */
  static std::shared_ptr<License> AcquireLicense();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  static bool ProgramAttributesSatisfied(const MathematicalProgram&);
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
  // Set to true if the user wants the solver to produce output to the console
  // or a log file. Default to false, such that the solver runs silently.
  // Check out https://docs.mosek.com/9.0/capi/solver-io.html for more info.
  bool stream_logging_{false};
  // set @p log_file to the name of that file. If the user wants to output the
  // logging to the console, then set log_file to empty string. Default to an
  // empty string.
  std::string log_file_{};
};

}  // namespace solvers
}  // namespace drake
