#include "drake/multibody/fixed_fem/dev/petsc_schur_complement.h"

#include <iostream>
#include <numeric>

#include <petscksp.h>

#include "drake/multibody/fixed_fem/dev/petsc_internal.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

class PetscSchurComplement::Impl {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Impl);

  Impl() = default;

  Impl(const PetscSymmetricBlockSparseMatrix& symmetric_block_sparse_matrix,
       const std::vector<int>& eliminated, const std::vector<int>& rest) {
    CalcSchurComplement(symmetric_block_sparse_matrix, eliminated, rest);
  }

  ~Impl() = default;

  void CalcSchurComplement(
      const PetscSymmetricBlockSparseMatrix& symmetric_block_sparse_matrix,
      const std::vector<int>& eliminated, const std::vector<int>& rest) {
    p_ = rest.size() * 3;
    q_ = eliminated.size() * 3;
    const PetscSparseMatrix matrix =
        symmetric_block_sparse_matrix.MakePetscSparseMatrix();
    DRAKE_DEMAND(matrix.rows() == matrix.cols());
    if (eliminated.size() == 0) {
      D_complement_ = MakeDenseMatrix(matrix.get_A());
      Dinv_B_transpose_.resize(0, matrix.cols());
      return;
    }
    if (rest.size() == 0) {
      D_complement_.resize(0, 0);
      Dinv_B_transpose_.resize(matrix.cols(), 0);
    }
    // Build indices to create the four blocks in Schur complement.
    IS is0, is1;
    ISCreateBlock(PETSC_COMM_SELF, 3, rest.size(), rest.data(),
                  PETSC_COPY_VALUES, &is0);
    ISCreateBlock(PETSC_COMM_SELF, 3, eliminated.size(), eliminated.data(),
                  PETSC_COPY_VALUES, &is1);

    Mat D_petsc, B_transpose_petsc;
    MatCreateSubMatrix(matrix.get_A(), is1, is0, MAT_INITIAL_MATRIX,
                       &B_transpose_petsc);
    MatCreateSubMatrix(matrix.get_A(), is1, is1, MAT_INITIAL_MATRIX, &D_petsc);

    KSP solver;
    PC preconditioner;
    KSPCreate(PETSC_COMM_SELF, &solver);
    KSPSetOperators(solver, D_petsc, D_petsc);
    KSPSetType(solver, KSPCG);
    KSPGetPC(solver, &preconditioner);
    PCSetType(preconditioner, PCICC);
    KSPSetFromOptions(solver);
    KSPSetUp(solver);

    MatAssemblyBegin(B_transpose_petsc, MAT_FINAL_ASSEMBLY);
    MatAssemblyEnd(B_transpose_petsc, MAT_FINAL_ASSEMBLY);
    Mat B_transpose_petsc_dense;
    MatConvert(B_transpose_petsc, MATDENSE, MAT_INITIAL_MATRIX,
               &B_transpose_petsc_dense);
    Mat Dinv_B_transpose_petsc;
    MatDuplicate(B_transpose_petsc_dense, MAT_DO_NOT_COPY_VALUES,
                 &Dinv_B_transpose_petsc);
    KSPMatSolve(solver, B_transpose_petsc_dense, Dinv_B_transpose_petsc);

    const MatrixX<double> B_transpose =
        MakeDenseMatrix(B_transpose_petsc_dense);
    Dinv_B_transpose_ = MakeDenseMatrix(Dinv_B_transpose_petsc);

    MatDestroy(&B_transpose_petsc_dense);
    MatDestroy(&Dinv_B_transpose_petsc);
    MatDestroy(&B_transpose_petsc);
    MatDestroy(&D_petsc);

    Mat A_petsc;
    MatCreateSubMatrix(matrix.get_A(), is0, is0, MAT_INITIAL_MATRIX, &A_petsc);
    const MatrixX<double> A = MakeDenseMatrix(A_petsc);
    MatDestroy(&A_petsc);

    D_complement_ = A - B_transpose.transpose() * Dinv_B_transpose_;
  }

  VectorX<double> SolveForY(const Eigen::Ref<const VectorX<double>>& x) const {
    /* If M = D, then the system reduces to Dy = 0. */
    if (p_ == 0) {
      return VectorX<double>::Zero(q_);
    }
    DRAKE_DEMAND(x.size() == p_);
    return -Dinv_B_transpose_ * x;
  }

  const MatrixX<double>& get_D_complement() const { return D_complement_; }

 private:
  int p_{0};
  int q_{0};
  MatrixX<double> D_complement_;      // A - BD⁻¹Bᵀ.
  MatrixX<double> Dinv_B_transpose_;  // D⁻¹Bᵀ.
};

PetscSchurComplement::PetscSchurComplement() {
  pimpl_ = std::make_unique<Impl>();
}

PetscSchurComplement::PetscSchurComplement(
    const PetscSymmetricBlockSparseMatrix& matrix,
    const std::vector<int>& eliminated_vertices, const std::vector<int>& rest)
    : pimpl_(new Impl(matrix, eliminated_vertices, rest)) {}

PetscSchurComplement::~PetscSchurComplement() = default;

std::unique_ptr<PetscSchurComplement> PetscSchurComplement::Clone() const {
  std::unique_ptr<PetscSchurComplement> clone(new PetscSchurComplement());
  *clone->pimpl_ = *this->pimpl_;
  return clone;
}

void PetscSchurComplement::CalcSchurComplement(
    const PetscSymmetricBlockSparseMatrix& matrix,
    const std::vector<int>& eliminated_vertices, const std::vector<int>& rest) {
  pimpl_->CalcSchurComplement(matrix, eliminated_vertices, rest);
}

const MatrixX<double>& PetscSchurComplement::get_D_complement() const {
  return pimpl_->get_D_complement();
}

VectorX<double> PetscSchurComplement::SolveForY(
    const Eigen::Ref<const VectorX<double>>& x) const {
  return pimpl_->SolveForY(x);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
