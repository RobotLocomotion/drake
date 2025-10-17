#pragma once

#include <functional>
#include <memory>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/solvers/decision_variable.h"
#include "drake/solvers/solver_base.h"

namespace drake {
namespace solvers {

/// The Gurobi solver details after calling Solve() function. The user can call
/// MathematicalProgramResult::get_solver_details<GurobiSolver>() to obtain the
/// details.
struct GurobiSolverDetails {
  /// The gurobi optimization time. Please refer to
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/attributes/model.html#attrruntime
  double optimizer_time{};

  /// The error message returned from Gurobi call. Please refer to
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/errors.html
  int error_code{};

  /// The status code when the optimize call has returned. Please refer to
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/statuscodes.html
  int optimization_status{};

  /// The best known bound on the optimal objective. This is used in mixed
  /// integer optimization. Please refer to
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/attributes/model.html#objbound
  double objective_bound{NAN};
};

/// An implementation of SolverInterface for the commercially-licensed Gurobi
/// solver (https://www.gurobi.com/).
///
/// The default build of Drake is not configured to use Gurobi, so therefore
/// SolverInterface::available() will return false. You must compile Drake
/// from source in order to link against Gurobi. For details, refer to the
/// documentation at https://drake.mit.edu/bazel.html#proprietary-solvers.
///
/// The GRB_LICENSE_FILE environment variable controls whether or not
/// SolverInterface::enabled() returns true.  If it is set to any non-empty
/// value, then the solver is enabled; otherwise, the solver is not enabled.
///
/// Gurobi solver supports options/parameters listed in
/// https://docs.gurobi.com/projects/optimizer/en/12.0/concepts/parameters.html.
/// On top of these options, we provide the following additional options
/// 1. "GRBwrite", set to a file name so that Gurobi solver will write the
///    optimization model to this file, check
///    https://www.docs.gurobi.com/projects/optimizer/en/12.0/reference/python/model.html#Write
///    for more details, such as all supported file extensions. Set this option
///    to "" if you don't want to write to file. Default is not to write to a
///    file.
/// 2. "GRBcomputeIIS", set to 1 to compute an Irreducible Inconsistent
///    Subsystem (IIS) when the problem is infeasible. Refer to
///    https://docs.gurobi.com/projects/optimizer/en/12.0/reference/python/model.html#Model.computeIIS
///    for more details. Often this method is called together with
///    setting GRBwrite to "FILENAME.ilp" to write IIS to a file with extension
///    "ilp". Default is not to compute IIS.
///
/// GurobiSolver supports parallelization during Solve().
/// If both the "Threads" integer solver option and
/// CommonSolverOption::kMaxThreads have been set by the user, then the value in
/// "Threads" will be used as the number of threads.
///
/// If neither the "Threads" integer solver option nor
/// CommonSolverOption::kMaxThreads has been set by the user, then
/// %GurobiSolver uses the environment variable GUROBI_NUM_THREADS (if set) as a
/// default value for "Threads".
///
/// If none of "Threads",
/// CommonSolverOption::kMaxThreads, or GUROBI_NUM_THREADS are set, then
/// Drake's default maximum parallelism will be used.
class GurobiSolver final : public SolverBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GurobiSolver);

  /// Type of details stored in MathematicalProgramResult.
  using Details = GurobiSolverDetails;

  GurobiSolver();
  ~GurobiSolver() final;

  /// Contains info returned to a user function that handles
  /// a Node or Solution callback.
  /// @see MipNodeCallbackFunction
  /// @see MipSolCallbackFunction
  struct SolveStatusInfo {
    /// Runtime as of this callback.
    double reported_runtime{};
    /// Objective of current solution.
    double current_objective{};
    /// Objective of best solution yet.
    double best_objective{};
    /// Best known objective lower bound.
    double best_bound{};
    /// Number of nodes explored so far.
    int explored_node_count{};
    /// Number of feasible sols found so far.
    int feasible_solutions_count{};
  };

  /// Users can supply a callback to be called when the Gurobi solver
  /// finds an intermediate solution node, which may not be feasible.
  /// See Gurobi reference manual for more detail on callbacks:
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/callbacks.html
  /// The user may supply a partial solution in the VectorXd and
  /// VectorXDecisionVariable arguments that will be passed to Gurobi
  /// as a candidate feasible solution.
  /// See gurobi_solver_test.cc for an example of using std::bind
  /// to create a callback of this signature, while allowing
  /// additional data to be passed through.
  /// @param MathematicalProgram& The optimization wrapper, whose
  /// current variable values (accessible via
  /// MathematicalProgram::GetSolution) will be set to the intermediate
  /// solution values.
  /// @param SolveStatusInfo& Intermediate solution status information values
  /// queried from Gurobi.
  /// @param VectorXd* User may assign this to be the values of the variable
  /// assignments.
  /// @param VectorXDecisionVariable* User may assign this to be the decision
  /// variables being assigned. Must have the same number of elements as
  /// the VectorXd assignment.
  typedef std::function<void(const MathematicalProgram&,
                             const SolveStatusInfo& callback_info,
                             Eigen::VectorXd*, VectorXDecisionVariable*)>
      MipNodeCallbackFunction;

  /// Registers a callback to be called at intermediate solutions
  /// during the solve.
  /// @param callback User callback function.
  void AddMipNodeCallback(const MipNodeCallbackFunction& callback) {
    mip_node_callback_ = callback;
  }

  /// Users can supply a callback to be called when the Gurobi solver
  /// finds a feasible solution.
  /// See Gurobi reference manual for more detail on callbacks:
  /// https://docs.gurobi.com/projects/optimizer/en/12.0/reference/numericcodes/callbacks.html
  /// See gurobi_solver_test.cc for an example of using std::bind
  /// to create a callback of this signature, while allowing
  /// additional data to be passed through.
  /// @param MathematicalProgram& The optimization wrapper, whose
  /// current variable values (accessible via
  /// MathematicalProgram::GetSolution) will be set to the intermediate
  /// solution values.
  /// @param SolveStatusInfo& Intermediate solution status information values
  /// queried from Gurobi.
  /// @param void* Arbitrary data supplied during callback registration.
  typedef std::function<void(const MathematicalProgram&,
                             const SolveStatusInfo& callback_info)>
      MipSolCallbackFunction;

  /// Registers a callback to be called at feasible solutions
  /// during the solve.
  /// @param callback User callback function.
  void AddMipSolCallback(const MipSolCallbackFunction& callback) {
    mip_sol_callback_ = callback;
  }

  /**
   * This type contains a valid Gurobi license environment, and is only to be
   * used from AcquireLicense().
   */
  class License;

  /**
   * This acquires a Gurobi license environment shared among all GurobiSolver
   * instances. The environment will stay valid as long as at least one
   * shared_ptr returned by this function is alive. GurobiSolver calls this
   * method on each Solve().
   *
   * If the license file contains the string `HOSTID`, then we treat this as
   * confirmation that the license is attached to the local host, and maintain
   * an internal copy of the shared_ptr for the lifetime of the process.
   * Otherwise the default behavior is to only hold the license while at least
   * one GurobiSolver instance is alive.
   *
   * Call this method directly and maintain the shared_ptr ONLY if you must use
   * different MathematicalProgram instances at different instances in time,
   * and repeatedly acquiring the license is costly (e.g., requires contacting
   * a license server).
   * @return A shared pointer to a license environment that will stay valid as
   * long as any shared_ptr returned by this function is alive. If Gurobi is
   * not available in your build, this will return a null (empty) shared_ptr.
   * @throws std::exception if Gurobi is available but a license cannot be
   * obtained.
   */
  static std::shared_ptr<License> AcquireLicense();

  /// @name Static versions of the instance methods with similar names.
  //@{
  static SolverId id();
  static bool is_available();
  /// Returns true iff the environment variable GRB_LICENSE_FILE has been set
  /// to a non-empty value.
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

  // Note that this is mutable to allow latching the allocation of env_
  // during the first call of Solve() (which avoids grabbing a Gurobi license
  // before we know that we actually want one).
  mutable std::shared_ptr<License> license_;
  // Callbacks and generic user data to pass through,
  // or NULL if no callback has been supplied.
  MipNodeCallbackFunction mip_node_callback_;
  MipSolCallbackFunction mip_sol_callback_;
};

}  // namespace solvers
}  // namespace drake
