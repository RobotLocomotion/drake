#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace solvers {
namespace fbstab {

// Forward declaration of testing class to enable a friend declaration.
namespace test {
class MpcComponentUnitTests;
}  // namespace test

/**
 * This class represents data for quadratic programming problems of the
 * following type (1):
 *
 *     min.  \sum_{i=0}^N 1/2 [x(i)]' * [Q(i) S(i)'] [x(i)] + [q(i)]'*[x(i)]
 *                            [u(i)]    [S(i) R(i) ] [u(i)]   [r(i)]  [u(i)]
 *     s.t.  x(i+1) = A(i)*x(i) + B(i) u(i) + c(i), i = 0 ... N-1
 *           x(0) = x0,
 *           E(i)*x(i) + L(i)*u(i) + d(i) <= 0,     i = 0 ... N
 *
 * The horizon length is N, the dimension of x(i) is nx, of u(i) is nu,
 * and the number of constraints per stage is nc.
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
 * This class contains storage and methods for implicitly working with the
 * compact representation (2) in an efficient manner.
 */
class MpcData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpcData)
  /**
   * Creates problem data and performs input validation. Throws
   * a runtime_error if the problem data aren't consistently sized.
   *
   * This class assumes that the pointers to the data remain valid.
   *
   * All arguments are inputs and point to data defining a linear-quadratic
   * optimal control problem, see the class comment.
   */
  MpcData(const std::vector<Eigen::MatrixXd>* Q,
          const std::vector<Eigen::MatrixXd>* R,
          const std::vector<Eigen::MatrixXd>* S,
          const std::vector<Eigen::VectorXd>* q,
          const std::vector<Eigen::VectorXd>* r,
          const std::vector<Eigen::MatrixXd>* A,
          const std::vector<Eigen::MatrixXd>* B,
          const std::vector<Eigen::VectorXd>* c,
          const std::vector<Eigen::MatrixXd>* E,
          const std::vector<Eigen::MatrixXd>* L,
          const std::vector<Eigen::VectorXd>* d, const Eigen::VectorXd* x0);

  /**
   * Computes the operation y <- a*H*x + b*y without forming H explicitly.
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   *
   * @param[in] x Input vector, length(x) = (nx+nu)*(N+1)
   * @param[in] a Input scaling
   * @param[in] b Scaling
   * @param[in,out] y Output vector, length(y) = (nx+nu)*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void gemvH(const Eigen::VectorXd& x, double a, double b,
             Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*A*x + b*y without forming A explicitly.
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] x Input vector, length(x) = (nx+nu)*(N+1)
   * @param[in] a Input scaling
   * @param[in] b Scaling
   * @param[in,out] y Output vector, length(y) = nc*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void gemvA(const Eigen::VectorXd& x, double a, double b,
             Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*G*x + b*y without forming G explicitly
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] x Input vector, length(x) = (nx+nu)*(N+1)
   * @param[in] a Input scaling
   * @param[in] b Scaling
   * @param[in,out] y Output vector, length(y) = nx*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void gemvG(const Eigen::VectorXd& x, double a, double b,
             Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*A'*x + b*y without forming A explicitly
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] x Input vector, length(x) = nc*(N+1)
   * @param[in] a Input scaling
   * @param[in] b Scaling
   * @param[in,out] y Output vector, length(y) = (nx+nu)*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void gemvAT(const Eigen::VectorXd& x, double a, double b,
              Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*G'*x + b*y without forming G explicitly
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] x Input vector, length(x) = (nx)*(N+1)
   * @param[in] a Input scaling
   * @param[in] b Scaling
   * @param[in,out] y Output vector, length(y) = (nx+nu)*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void gemvGT(const Eigen::VectorXd& x, double a, double b,
              Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*f + y without forming f explicitly.
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] a Scaling factor
   * @param[in,out] y Output vector, length(y) = (nx+nu)*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void axpyf(double a, Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*h + y without forming h explicitly.
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] a Scaling factor
   * @param[in,out] y Output vector, length(y) = nx*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void axpyh(double a, Eigen::VectorXd* y) const;

  /**
   * Computes y <- a*b + y without forming b explicitly.
   * This implements a BLAS operation, see
   * http://www.netlib.org/blas/blasqr.pdf.
   * @param[in] a Scaling factor
   * @param[in,out] y Output vector, length(y) = nc*(N+1)
   *
   * Throws a runtime_error if sizes aren't consistent or y is null.
   */
  void axpyb(double a, Eigen::VectorXd* y) const;

 private:
  int N_ = 0;   // horizon length
  int nx_ = 0;  // number of states
  int nu_ = 0;  // number of controls
  int nc_ = 0;  // constraints per stage
  int nz_ = 0;  // number of primal variables
  int nl_ = 0;  // number of equality duals
  int nv_ = 0;  // number of inequality duals

  const std::vector<Eigen::MatrixXd>* Q_ = nullptr;
  const std::vector<Eigen::MatrixXd>* R_ = nullptr;
  const std::vector<Eigen::MatrixXd>* S_ = nullptr;
  const std::vector<Eigen::VectorXd>* q_ = nullptr;
  const std::vector<Eigen::VectorXd>* r_ = nullptr;
  const std::vector<Eigen::MatrixXd>* A_ = nullptr;
  const std::vector<Eigen::MatrixXd>* B_ = nullptr;
  const std::vector<Eigen::VectorXd>* c_ = nullptr;
  const std::vector<Eigen::MatrixXd>* E_ = nullptr;
  const std::vector<Eigen::MatrixXd>* L_ = nullptr;
  const std::vector<Eigen::VectorXd>* d_ = nullptr;
  const Eigen::VectorXd* x0_ = nullptr;

  // Throws an exception if any of the input vectors have inconsistent lengths.
  void validate_length() const;

  // Throws an exception if any of the input matrices have inconsistent sizes.
  // Assumes that validate_length() has already been called.
  void validate_size() const;

  friend class test::MpcComponentUnitTests;
  friend class RiccatiLinearSolver;
  friend class FBstabMpc;
};

}  // namespace fbstab
}  // namespace solvers
}  // namespace drake
