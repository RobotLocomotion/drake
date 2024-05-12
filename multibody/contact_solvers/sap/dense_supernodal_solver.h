#pragma once

#include <vector>

#include "drake/multibody/contact_solvers/supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// A class that implements the SuperNodalSolver interface using dense algebra.
// As with every other SuperNodalSolver, this class implements a Cholesky
// factorization of a Hessian matrix H of the form H = A + Jᵀ⋅G⋅J, where A is
// referred to as the dynamics matrix and J is the Jacobian matrix.
class DenseSuperNodalSolver final : public SuperNodalSolver {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DenseSuperNodalSolver)

  // Constructs a dense solver for dynamics matrix A and Jacobian matrix J.
  // This class holds references to matrices A and J, and therefore they must
  // outlive this object.
  // @throws std::runtime_error if A or J is nullptr.
  // @throws std::runtime_error if a block in A is not square.
  // @pre each block in A is SPD.
  // @pre The number of columns of J equals the size of A.
  // @note The sizes in J are not checked here, but during SetWeightMatrix().
  DenseSuperNodalSolver(const std::vector<MatrixX<double>>* A,
                        const BlockSparseMatrix<double>* J);

  // @returns the size of the system.
  int size() const { return H_.rows(); }

 private:
  // Implementations of SuperNodalSolver NVIs.
  bool DoSetWeightMatrix(
      const std::vector<Eigen::MatrixXd>& block_diagonal_G) final;
  Eigen::MatrixXd DoMakeFullMatrix() const final { return H_; }
  bool DoFactor() final;
  void DoSolveInPlace(Eigen::VectorXd* b) const final;
  int DoGetSize() const final { return size(); }

  const std::vector<MatrixX<double>>* A_{nullptr};
  const BlockSparseMatrix<double>* J_{nullptr};
  // The support for dense algebra is mostly for testing purposes, even
  // though the computation of the dense H (and in particular of the Jᵀ⋅G⋅J
  // term) is very costly. Therefore below we decided to trade off speed for
  // stability when choosing to use an LDLT decomposition instead of a slightly
  // faster, though less stable, LLT decomposition.
  MatrixX<double> H_;
  // N.B. This instantiates an in-place LDLT solver that uses the memory in H_.
  Eigen::LDLT<MatrixX<double>> Hldlt_{H_};
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
