#pragma once

#include <memory>
#include <numeric>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/pooled_sap/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

template <typename T>
class Hessian {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Hessian);

  explicit Hessian(bool is_sparse = true) : is_sparse_(is_sparse) {
    if constexpr (!std::is_same_v<T, double>) is_sparse_ = false;
  }

  /* Sets whether to use sparsity. Ignored for AutoDiffXd, which only supports
  dense. */
  void set_sparse(bool is_sparse = true) {
    unused(is_sparse);
    if constexpr (std::is_same_v<T, double>) {
      is_sparse_ = is_sparse;
    }
  }

  /* Resizes from the provided sparsity. Sparsity pattern ignored if is_sparse()
   * is false. */
  void Resize(internal::BlockSparsityPattern sparsity) {
    if (is_sparse_) {
      // TODO(amcastro-tri): consider how to re-use memory for the Hessian.
      sparse_ = std::make_unique<internal::BlockSparseSymmetricMatrix>(
          std::move(sparsity));
    } else {
      const std::vector<int>& clique_sizes = sparsity.block_sizes();
      const int nv =
          std::accumulate(clique_sizes.begin(), clique_sizes.end(), 0);
      Resize(nv);
      block_size_ = sparsity.block_sizes();
      block_start_.resize(clique_sizes.size());
      block_start_[0] = 0;
      std::partial_sum(clique_sizes.begin(), clique_sizes.end() - 1,
                       block_start_.begin() + 1);
      fmt::print("nc: {}. nv: {}\n", clique_sizes.size(), nv);
      for (int c = 0; c < ssize(clique_sizes); ++c) {
        fmt::print("c: {}. st: {}. sz: {}\n", c, block_start_[c],
                   block_size_[c]);
      }
    }
  }

  /* Resize for a dense Hessian of size nv.
   @pre is_sparse() is true. */
  void Resize(int nv) {
    DRAKE_DEMAND(!is_sparse_);
    dense_.resize(nv, nv);
  }

  void SetZero() {
    if (is_sparse()) {
      sparse_->SetZero();
    } else {
      dense_.setZero();
    }
  }

  /* Adds to the ij-th block, as defined at construction by the specified
  sparsity. */
  void AddToBlock(int i, int j, const Eigen::Ref<const MatrixX<T>>& Aij) {
    DRAKE_DEMAND(i >= j);  // As with BlockSparseSymmetricMatrix.
    if (is_sparse()) {
      if constexpr (std::is_same_v<T, double>) {
        sparse_->AddToBlock(i, j, Aij);
      } else {
        throw std::logic_error("Sparse does not support AutoDiffXd");
      }
    } else {
      const int row_start = block_start_[i];
      const int col_start = block_start_[j];
      const int rows = Aij.rows();
      const int cols = Aij.cols();
      DRAKE_DEMAND(block_size_[i] == rows);
      DRAKE_DEMAND(block_size_[j] == cols);
      dense_.block(row_start, col_start, rows, cols) += Aij;
      if (i > j) {
        dense_.block(col_start, row_start, cols, rows) = Aij.transpose();
      }
    }
  }

  bool is_sparse() const { return is_sparse_; }

  MatrixX<T> MakeDenseMatrix() const {
    if (is_sparse()) {
      if constexpr (std::is_same_v<T, double>) {
        return sparse_->MakeDenseMatrix();
      } else {
        throw std::logic_error("Sparse does not support AutoDiffXd");
      }
    } else {
      return dense_;
    }
  }

 private:
  std::vector<int> block_size_;
  std::vector<int> block_start_;

  // TODO(amcastro-tri): Make a struct with specializations to dense/sparse. Or
  // use variant.
  bool is_sparse_;
  MatrixX<T> dense_;  // Square matrix of size num_velocities().
  copyable_unique_ptr<internal::BlockSparseSymmetricMatrix> sparse_;
};

// template <typename T>
// using HessianVariant =
//     std::variant<DenseHessian<T>, BlockSparseSymmetricMatrix>;

/* SAP generalized velocities v and SAP quantities function of v.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapData);

  struct Cache {
    void Resize(int num_bodies, int num_velocities,
                const std::vector<int>& patch_sizes) {
      const int nv = num_velocities;
      Av.resize(nv);
      gradient.resize(nv);
      spatial_velocities.Resize(num_bodies);
      patch_constraints_data.Resize(num_velocities, patch_sizes);

      // These are members in BlockSparseSuperNodalSolver, along sugar.
      // std::unique_ptr<BlockSparseSymmetricMatrix> H_;
      // BlockSparseCholeskySolver<Eigen::MatrixXd> solver_;
    }

    T momentum_cost{0};
    T constraints_cost{0};
    T cost;
    VectorX<T> Av;        // = A * v.
    VectorX<T> gradient;  // Of size num_velocities().
    Hessian<T> hessian;

    // Rigid body spatial velocities, V_WB.
    EigenPool<Vector6<T>> spatial_velocities;

    // Type-specific constraint pools.
    PatchConstraintsDataPool<T> patch_constraints_data;
  };

  /* Struct to store pre-allocated scratch space.
   This space is not intended for long-term storage and is often cleared or
   overwritten as needed. */
  struct Scratch {
    /* Clears all data without changing capacity. */
    void Clear() {
      VectorX_pool.Clear();
      MatrixX_pool.Clear();
    }
    EigenPool<VectorX<T>> VectorX_pool;
    EigenPool<MatrixX<T>> MatrixX_pool;
  };

  /* Default constructor for empty data. */
  SapData() = default;

  /* @param num_velocities Total number of generalized velocities.
     @param patch_sizes Number of contact pairs for each patch.
     @param patch_num_velocities Number of participating velocities per patch.
     */
  void Resize(int num_bodies, int num_velocities,
              const std::vector<int>& clique_sizes,
              const std::vector<int>& patch_sizes) {
    (void)clique_sizes;
    v_.resize(num_velocities);
    cache_.Resize(num_bodies, num_velocities, patch_sizes);
  }

  int num_velocities() const { return v_.size(); }

  int num_patches() const {
    return cache_.patch_constraints_data.num_patches();
  }

  const VectorX<T>& v() const { return v_; }
  VectorX<T>& v() { return v_; }

  const Cache& cache() const { return cache_; }
  Cache& cache() { return cache_; }

  Scratch& scratch() const { return scratch_; }

 private:
  VectorX<T> v_;  // Generalized velocities.
  Cache cache_;   // All other quantities function of v.
  // We allow PooledSapModel methods to write on the scratch as needed.
  mutable Scratch scratch_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
