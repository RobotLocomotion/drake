#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/mathematical_program_solver_interface.h"

namespace drake {
namespace solvers {
struct MosekSolverDetails {
  /// The mosek optimization time. Please refer to MSK_DINF_OPTIMIZER_TIME in
  /// https://docs.mosek.com/8.1/capi/constants.html?highlight=msk_dinf_optimizer_time
  double optimizer_time{};
  /// The response code returned from mosek solver. Check
  /// https://docs.mosek.com/8.1/capi/response-codes.html for the meaning on the
  /// response code.
  int rescode{};
  /// The solution status after solving the problem. Check
  /// https://docs.mosek.com/8.1/capi/accessing-solution.html and
  /// https://docs.mosek.com/8.1/capi/constants.html#mosek.solsta for the
  /// meaning on the solution status.
  int solution_status{};
};

class MosekSolver : public MathematicalProgramSolverInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MosekSolver)

  MosekSolver() = default;
  ~MosekSolver() override = default;

  /**
   * Defined true if Mosek was included during compilation, false otherwise.
   */
  bool available() const override { return is_available(); };

  static bool is_available();

  void Solve(const MathematicalProgram& prog,
             const optional<Eigen::VectorXd>& initial_guess,
             const optional<SolverOptions>& solver_options,
             MathematicalProgramResult* result) const override;

  // Todo(hongkai.dai@tri.global): deprecate Solve with a non-const
  // MathematicalProgram.
  SolutionResult Solve(MathematicalProgram& prog) const override;

  SolverId solver_id() const override;

  /// @return same as MathematicalProgramSolverInterface::solver_id()
  static SolverId id();

  bool AreProgramAttributesSatisfied(
      const MathematicalProgram& prog) const override;

  static bool ProgramAttributesSatisfied(const MathematicalProgram& prog);

  /**
   * Control stream logging. Refer to
   * https://docs.mosek.com/8.1/capi/solver-io.html for more details.
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

 private:
  // Note that this is mutable to allow latching the allocation of mosek_env_
  // during the first call of Solve() (which avoids grabbing a Mosek license
  // before we know that we actually want one).
  mutable std::shared_ptr<License> license_;
  // Set to true if the user wants the solver to produce output to the console
  // or a log file. Default to false, such that the solver runs silently.
  // Check out https://docs.mosek.com/8.1/capi/solver-io.html for more info.
  bool stream_logging_{false};
  // set @p log_file to the name of that file. If the user wants to output the
  // logging to the console, then set log_file to empty string. Default to an
  // empty string.
  std::string log_file_{};
};

}  // namespace solvers
}  // namespace drake
