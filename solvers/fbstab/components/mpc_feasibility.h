#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/mpc_data.h"
#include "drake/solvers/fbstab/components/mpc_variable.h"

namespace drake {
namespace solvers {
namespace fbstab {

/**
 * This class detects infeasibility in quadratic programs, see
 * mpc_data.h for a description of the QPs.
 * It contains methods for determining if a primal-dual variable
 * is a certificate of either unboundedness (dual infeasibility)
 * or primal infeasibility. It implements
 * Algorithm 3 of https://arxiv.org/pdf/1901.04046.pdf.
 */
class MpcFeasibility {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpcFeasibility)
  /**
   * Allocates workspace memory.
   *
   * @param[in] N  horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per stage
   *
   * Throws a runtime_error if any inputs are non-positive.
   */
  MpcFeasibility(int N, int nx, int nu, int nc);

  /**
   * Checks to see if x is an infeasibility certificate for the QP and stores
   * the result internally.
   * @param[in] x   infeasibility certificate candidate
   * @param[in] tol numerical tolerance
   *
   * Throws a runtime_error if x and *this aren't the same size
   * or if the problem data hasn't been linked.
   */
  void ComputeFeasibility(const MpcVariable& x, double tol);

  /**
   * Retrieves the result of the last infeasibility check.
   * @return false if a dual infeasibility certificate was found, true otherwise
   */
  bool IsDualFeasible() const { return dual_feasible_; }

  /**
   * Retrieves the result of the last infeasibility check.
   * @return false if a primal infeasibility certificate was found, true
   * otherwise
   */
  bool IsPrimalFeasible() const { return primal_feasible_; }

 private:
  // Workspaces
  Eigen::VectorXd tz_;
  Eigen::VectorXd tl_;
  Eigen::VectorXd tv_;

  bool primal_feasible_ = true;
  bool dual_feasible_ = true;

  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
