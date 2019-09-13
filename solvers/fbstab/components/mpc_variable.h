#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/solvers/fbstab/components/mpc_data.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Forward declaration of testing class to enable a friend declaration.
namespace test {
class MpcComponentUnitTests;
}  // namespace test

/**
 * This class implements primal-dual variables for model predictive
 * control QPs. See mpc_data.h for the mathematical description.
 * Stores variables and defines methods implementing useful operations.
 *
 * Primal-dual variables have 4 fields:
 * - z: Decision variables (x0,u0,x1,u1, ... xN,uN)
 * - l: Co-states/equality duals (l0, ... ,lN)
 * - v: Inequality duals (v0, ..., vN)
 * - y: Inequality margins (y0, ..., yN)
 *
 * length(z) = nz = (nx*nu)*(N+1)
 * length(l) = nl = nx*(N+1)
 * length(v) = nv = nc*(N+1)
 * length(y) = nv = nc*(N+1)
 */
class MpcVariable {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpcVariable)
  /**
   * Allocates memory for a primal-dual variable.
   *
   * @param[in] N  horizon length
   * @param[in] nx number of states
   * @param[in] nu number of control input
   * @param[in] nc number of constraints per stage
   *
   * Throws a runtime_error if any of the inputs are non-positive.
   */
  MpcVariable(int N, int nx, int nu, int nc);

  /**
   * Creates a primal-dual variable using preallocated memory.
   *
   * @param[in] z    A vector to store the decision variables.
   * @param[in] l    A vector to store the co-states/equality duals.
   * @param[in] v    A vector to store the dual variables.
   * @param[in] y    A vector to store the inequality margin.
   *
   * Throws a runtime_error if sizes are mismatched or if any of the inputs are
   * null.
   */
  MpcVariable(Eigen::VectorXd* z, Eigen::VectorXd* l, Eigen::VectorXd* v,
              Eigen::VectorXd* y);

  /**
   * Links to problem data needed to perform calculations.
   * Calculations cannot be performed until a data object is provided.
   * @param[in] data pointer to the problem data
   */
  void LinkData(const MpcData* data) { data_ = data; }

  /**
   * Fills the variable with one value.
   * @param[in] a
   */
  void Fill(double a);

  /**
   * Sets the constraint margin to y = b - Az.
   * Throws a runtime_error if problem data hasn't been provided.
   */
  void InitializeConstraintMargin();

  /**
   * Performs the operation *this <- a*x + *this.
   * This is a level 1 BLAS operation for this object;
   * see http://www.netlib.org/blas/blasqr.pdf.
   *
   * @param[in] a scalar
   * @param[in] x vector
   *
   * Note that this handles the constraint margin correctly, i.e., after the
   * operation u.y = b - A*(u.z + a*x.z).
   *
   * Throws a runtime_error if problem data hasn't been provided.
   */
  void axpy(double a, const MpcVariable& x);

  /**
   * Deep copies x into this.
   * @param[in] x variable to be copied.
   */
  void Copy(const MpcVariable& x);

  /**
   * Projects the inequality duals onto the non-negative orthant,
   * i.e., v <- max(0,v).
   */
  void ProjectDuals();

  /**
   * Computes the Euclidean norm.
   * @return sqrt(|z|^2 + |l|^2 + |v|^2)
   */
  double Norm() const;

  /** Returns true if x and y have the same dimensions. */
  static bool SameSize(const MpcVariable& x, const MpcVariable& y);

  /** Accessor for the decision variable. */
  Eigen::VectorXd& z() { return *z_; }
  /** Accessor for the co-state. */
  Eigen::VectorXd& l() { return *l_; }
  /** Accessor for the dual variable. */
  Eigen::VectorXd& v() { return *v_; }
  /** Accessor for the inequality margin. */
  Eigen::VectorXd& y() { return *y_; }

  /** Accessor for the decision variable. */
  const Eigen::VectorXd& z() const { return *z_; }
  /** Accessor for the co-state. */
  const Eigen::VectorXd& l() const { return *l_; }
  /** Accessor for the dual variable. */
  const Eigen::VectorXd& v() const { return *v_; }
  /** Accessor for the inequality margin. */
  const Eigen::VectorXd& y() const { return *y_; }

 private:
  Eigen::VectorXd* z_ = nullptr;  // primal variable
  Eigen::VectorXd* l_ = nullptr;  // co-state/ equality dual
  Eigen::VectorXd* v_ = nullptr;  // inequality dual
  Eigen::VectorXd* y_ = nullptr;  // constraint margin
  std::unique_ptr<Eigen::VectorXd> z_storage_;
  std::unique_ptr<Eigen::VectorXd> l_storage_;
  std::unique_ptr<Eigen::VectorXd> v_storage_;
  std::unique_ptr<Eigen::VectorXd> y_storage_;

  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals
  const MpcData* data_ = nullptr;

  // Getter for data_ with a nullptr check.
  const MpcData* data() const;

  friend class MpcResidual;
  friend class MpcFeasibility;
  friend class RiccatiLinearSolver;
  friend class FBstabMpc;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
