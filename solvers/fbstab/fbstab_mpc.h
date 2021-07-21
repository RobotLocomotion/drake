#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_feasibility.h"
#include "drake/solvers/fbstab/components/mpc_residual.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"
#include "drake/solvers/fbstab/components/riccati_linear_solver.h"
#include "drake/solvers/fbstab/fbstab_algorithm.h"

namespace drake {
namespace solvers {
namespace fbstab {

/** Convenience typedef for the templated version of the algorithm.*/
using FBstabAlgoMpc = FBstabAlgorithm<MpcVariable, MpcResidual, MpcData,
                                      RiccatiLinearSolver, MpcFeasibility>;

/**
 * FBstabMpc implements the Proximally Stabilized Semismooth Method for
 * solving the following quadratic programming problem (1):
 *
 *     min.  \sum_{i=0}^N 1/2 [x(i)]' * [Q(i) S(i)'] [x(i)] + [q(i)]'*[x(i)]
 *                            [u(i)]    [S(i) R(i) ] [u(i)]   [r(i)]  [u(i)]
 *
 *     s.t.  x(i+1) = A(i)*x(i) + B(i) u(i) + c(i), i = 0 ... N-1
 *           x(0) = x0
 *           E(i)*x(i) + L(i)*u(i) + d(i) <= 0,     i = 0 ... N
 *
 * Where
 *        [ Q(i),S(i)']
 *        [ S(i),R(i) ]
 *
 * is positive semidefinite for all i \in [0,N].
 * See also (29) in https://arxiv.org/pdf/1901.04046.pdf.
 *
 * The problem is of size (N,nx,nu,nc) where:
 * - N > 0 is the horizon length
 * - nx > 0 is the number of states
 * - nu > 0 is the number of control inputs
 * - nc > 0 is the number of constraints per stage
 *
 * This is a specialization of the general form (2),
 *
 *     min.  1/2 z'Hz + f'z
 *
 *     s.t.  Gz =  h
 *           Az <= b
 *
 * which has dimensions nz = (nx + nu) * (N + 1), nl = nx * (N + 1),
 * and nv = nc * (N + 1).
 *
 * Aside from convexity there are no assumptions made about the problem.
 * This method can detect unboundedness/infeasibility
 * and exploit arbitrary initial guesses.
 */
class FBstabMpc {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FBstabMpc);
  /**
   * Structure to hold the problem data.
   * See the class documentation or (29) in https://arxiv.org/pdf/1901.04046.pdf
   * for more details.
   */
  struct QPData {
    ///  N + 1 vector of nx x nx matrices
    const std::vector<Eigen::MatrixXd>* Q = nullptr;
    /// N + 1 vector of nu x nu matrices
    const std::vector<Eigen::MatrixXd>* R = nullptr;
    /// N + 1 vector of nu x nx matrices
    const std::vector<Eigen::MatrixXd>* S = nullptr;
    /// N + 1 vector of nx x 1 vectors
    const std::vector<Eigen::VectorXd>* q = nullptr;
    /// N + 1 vector of nu x 1 vectors
    const std::vector<Eigen::VectorXd>* r = nullptr;
    /// N vector of nx x nx matrices
    const std::vector<Eigen::MatrixXd>* A = nullptr;
    /// N  vector of nx x nu matrices
    const std::vector<Eigen::MatrixXd>* B = nullptr;
    /// N vector of nx vectors
    const std::vector<Eigen::VectorXd>* c = nullptr;
    /// N + 1 vector of nc x nx matrices
    const std::vector<Eigen::MatrixXd>* E = nullptr;
    /// N + 1 vector of nc x nu matrices
    const std::vector<Eigen::MatrixXd>* L = nullptr;
    /// N + 1 vector of nc x 1 vectors
    const std::vector<Eigen::VectorXd>* d = nullptr;
    /// nx x 1 vector
    const Eigen::VectorXd* x0 = nullptr;
  };

  /**
   * Structure to hold the initial guess and solution.
   * These vectors will be overwritten by the solve routine.
   */
  struct QPVariable {
    /// decision variables in \reals^nz
    Eigen::VectorXd* z = nullptr;
    /// equality duals/costates in \reals^nl
    Eigen::VectorXd* l = nullptr;
    /// inequality duals in \reals^nv
    Eigen::VectorXd* v = nullptr;
    /// constraint margin, i.e., y = b-Az, in \reals^nv
    Eigen::VectorXd* y = nullptr;
  };

  /**
   * Allocates workspaces needed when solving (1).
   *
   * @param[in] N Horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per timestep
   *
   * Throws an exception if any inputs are nonpositive.
   */
  FBstabMpc(int N, int nx, int nu, int nc);

  /**
   * Solves an instance of (1).
   *
   * @param[in]     qp problem data
   * @param[in,out] x  initial guess, overwritten with the solution
   * @param[in]     use_initial_guess if false the solver is initialized at the
   * origin
   * @return       Summary of the optimizer output, see fbstab_algorithm.h.
   */
  SolverOut Solve(const QPData& qp, const QPVariable* x,
                  bool use_initial_guess = true);

  /**
   * Allows for setting of solver options, see fbstab_algorithm.h for a list.
   * @param option Option name
   * @param value  New value
   */
  void UpdateOption(const char* option, double value);
  void UpdateOption(const char* option, int value);
  void UpdateOption(const char* option, bool value);

  /**
   * Controls the verbosity of the algorithm,
   * see fbstab_algorithm.h for details.
   * @param level new display level
   */
  void SetDisplayLevel(FBstabAlgoMpc::Display level);

 private:
  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals

  std::unique_ptr<FBstabAlgoMpc> algorithm_;
  std::unique_ptr<MpcVariable> x1_;
  std::unique_ptr<MpcVariable> x2_;
  std::unique_ptr<MpcVariable> x3_;
  std::unique_ptr<MpcVariable> x4_;
  std::unique_ptr<MpcResidual> r1_;
  std::unique_ptr<MpcResidual> r2_;
  std::unique_ptr<RiccatiLinearSolver> linear_solver_;
  std::unique_ptr<MpcFeasibility> feasibility_checker_;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
