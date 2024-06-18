#include "drake/multibody/contact_solvers/sap/dense_supernodal_solver.h"

#include <vector>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
const T& SafeDeference(std::string_view variable_name, const T* ptr) {
  if (ptr == nullptr) {
    throw std::runtime_error(
        fmt::format("Condition '{} != nullptr' failed.", variable_name));
  }
  return *ptr;
}

DenseSuperNodalSolver::DenseSuperNodalSolver(
    const std::vector<MatrixX<double>>* A, const BlockSparseMatrix<double>* J)
    : A_(SafeDeference("A", A)),
      J_(SafeDeference("J", J)),
      H_(Eigen::MatrixXd::Zero(J->cols(), J->cols())),
      Hldlt_(H_) {
  const int nv = [this]() {
    int size = 0;
    for (const auto& Ai : A_) {
      DRAKE_THROW_UNLESS(Ai.rows() == Ai.cols());
      size += Ai.rows();
    }
    return size;
  }();
  DRAKE_THROW_UNLESS(nv == J->cols());
}

bool DenseSuperNodalSolver::DoSetWeightMatrix(
    const std::vector<Eigen::MatrixXd>& G) {
  const int nv = H_.rows();

  // Make dense dynamics matrix.
  MatrixX<double> Adense = MatrixX<double>::Zero(nv, nv);
  int offset = 0;
  for (const auto& Ac : A_) {
    const int nv_clique = Ac.rows();
    Adense.block(offset, offset, nv_clique, nv_clique) = Ac;
    offset += nv_clique;
  }

  // Make dense Jacobian matrix.
  const MatrixX<double> Jdense = J_.MakeDenseMatrix();

  // Make dense weight matrix G.
  const int nk = Jdense.rows();
  MatrixX<double> Gdense = MatrixX<double>::Zero(nk, nk);
  offset = 0;
  for (const auto& Gi : G) {
    const int ni = Gi.rows();
    if (offset + ni > nk) {
      // Weight matrix G is incompatible with the Jacobian matrix J.
      return false;
    }
    Gdense.block(offset, offset, ni, ni) = Gi;
    offset += ni;
  }
  if (offset != nk) return false;  // G might miss some rows.

  H_ = Adense + Jdense.transpose() * Gdense * Jdense;

  return true;
}

bool DenseSuperNodalSolver::DoFactor() {
  Hldlt_.compute(H_);
  return Hldlt_.info() == Eigen::Success;
}

void DenseSuperNodalSolver::DoSolveInPlace(Eigen::VectorXd* b) const {
  *b = Hldlt_.solve(*b);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
