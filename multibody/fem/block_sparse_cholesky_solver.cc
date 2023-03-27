#include "drake/multibody/fem/block_sparse_cholesky_solver.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using std::set;
using std::unordered_set;
using Vector4i = Vector4<int>;
using contact_solvers::internal::PartialPermutation;
using std::vector;

vector<set<int>> BuildAdjacencyGraph(int num_verts,
                                     const vector<Vector4i>& elements) {
  vector<set<int>> adj(num_verts);
  for (const Vector4i& e : elements) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        if (e(i) >= e(j)) {
          adj[e(j)].insert(e(i));
        }
      }
    }
  }
  return adj;
}

std::vector<std::vector<int>> CalcSparsityPattern(
    const std::vector<std::set<int>>& adjacency_graph,
    const std::vector<int>& elimination_ordering) {
  int N = elimination_ordering.size();
  DRAKE_DEMAND(static_cast<int>(adjacency_graph.size()) == N);
  std::vector<int> old_to_new(N);
  for (int i = 0; i < N; ++i) {
    old_to_new[elimination_ordering[i]] = i;
  }

  /* Computes the adjacency graph for the new ordering. */
  std::vector<std::set<int>> new_graph(N);
  for (int i = 0; i < N; ++i) {
    for (int v : adjacency_graph[i]) {
      int a = old_to_new[i];
      int b = old_to_new[v];
      if (a >= b) {
        new_graph[b].insert(a);
      } else {
        new_graph[a].insert(b);
      }
    }
  }

  /* children[p] is a vector of sort children of p. */
  std::vector<std::vector<int>> children(N);
  std::vector<std::vector<int>> result(N);
  for (int i = 0; i < N; ++i) {
    /* Turn set into vector. */
    const std::set<int>& neighbor_i = new_graph[i];
    result[i].reserve(N);
    for (int n : neighbor_i) result[i].emplace_back(n);

    /* Merge the neighbors of i and all neighbors of children of i. */
    const auto& children_i = children[i];
    std::vector<int> result_i;  // Temp variable to hold result[i] as we
                                // accumulate all values.
    result_i.reserve(N);
    for (int c : children_i) {
      const auto& neighbor_c = result[c];
      std::set_union(result[i].begin(), result[i].end(), neighbor_c.begin() + 2,
                     neighbor_c.end(), std::back_inserter(result_i));
      result_i.swap(result[i]);
      result_i.clear();
    }
    /* Record the parent of i if i isn't already the root. */
    if (result[i].size() > 1) {
      const int p = result[i][1];
      children[p].emplace_back(i);
    }
  }
  return result;
}

/* Given an input matrix M, and a permutation mapping e, sets the resulting
 matrix M̃ such that M̃(i, j) = M(e(i), e(j)). */
void PermuteSymmetricBlockSparseMatrix(
    const SymmetricBlockSparseMatrix<double>& input,
    const std::vector<int>& permutation,
    SymmetricBlockSparseMatrix<double>* result) {
  DRAKE_DEMAND(result != nullptr);
  const int N = permutation.size();
  DRAKE_DEMAND(3 * N == input.cols());
  DRAKE_DEMAND(3 * N == result->cols());
  result->SetZero();

  /* Construct the inverse mapping of e, f. */
  std::vector<int> inverse_permutation(permutation.size());
  for (int i = 0; i < static_cast<int>(permutation.size()); ++i) {
    inverse_permutation[permutation[i]] = i;
  }
  /* M̃(i, j) = M(e(i), e(j)) is equivalent to M̃(f(i), f(j)) = M(i, j). */
  for (int j = 0; j < N; ++j) {
    const std::vector<int>& row_indices = input.get_col_blocks(j);
    for (int i : row_indices) {
      const Matrix3<double>& block = input.get_block(i, j);
      const int fi = inverse_permutation[i];
      const int fj = inverse_permutation[j];
      if (fi >= fj) {
        result->SetBlock(fi, fj, block);
      } else {
        result->SetBlock(fj, fi, block.transpose());
      }
    }
  }
}

void BlockSparseCholeskySolver::SetMatrix(
    const SymmetricBlockSparseMatrix<double>& A) {
  const std::vector<std::set<int>> adj = A.CalcAdjacencyGraph();
  std::vector<int> elimination_ordering = CalcEliminationOrdering(adj);
  SetMatrixImpl(A, adj, elimination_ordering);
}

void BlockSparseCholeskySolver::SetMatrix(
    const SymmetricBlockSparseMatrix<double>& A,
    const std::vector<int>& elimination_ordering) {
  const std::vector<std::set<int>> adj = A.CalcAdjacencyGraph();
  SetMatrixImpl(A, adj, elimination_ordering);
}

void BlockSparseCholeskySolver::UpdateMatrix(
    const SymmetricBlockSparseMatrix<double>& A) {
  PermuteSymmetricBlockSparseMatrix(A, internal_to_original_.permutation(),
                                    &L_);
  is_factored_ = false;
}

void BlockSparseCholeskySolver::SolveInPlace(VectorX<double>* y) const {
  DRAKE_DEMAND(is_factored_);
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == size());
  VectorX<double> permuted_y(*y);
  internal_to_original_dof_.ApplyInverse(*y, &permuted_y);

  /* Solve Lz = y in place. */
  for (int j = 0; j < block_cols_; ++j) {
    /* Solve for the j-th block entry. */
    permuted_y.segment<3>(3 * j) =
        L_diag_[j].triangularView<Eigen::Lower>().solve(
            permuted_y.segment<3>(3 * j));
    const auto& yj = permuted_y.segment<3>(3 * j);
    /* Eliminate for the j-th block entry from the system. */
    const auto& blocks_in_col_j = L_.get_col_blocks(j);
    for (int flat = 1; flat < static_cast<int>(blocks_in_col_j.size());
         ++flat) {
      const int i = blocks_in_col_j[flat];
      permuted_y.segment<3>(3 * i) -= L_.get_block(i, j) * yj;
    }
  }

  VectorX<double>& permuted_z = permuted_y;
  /* Solve Lᵀx = z in place. */
  for (int j = block_cols_ - 1; j >= 0; --j) {
    /* Eliminate all solved variables. */
    const auto& blocks_in_col_j = L_.get_col_blocks(j);
    for (int flat = 1; flat < static_cast<int>(blocks_in_col_j.size());
         ++flat) {
      const int i = blocks_in_col_j[flat];
      permuted_z.segment<3>(3 * j) -=
          L_.get_block(i, j).transpose() * permuted_z.segment<3>(3 * i);
    }
    /* Solve for the j-th block entry. */
    permuted_z.segment<3>(3 * j) =
        L_diag_[j].transpose().triangularView<Eigen::Upper>().solve(
            permuted_z.segment<3>(3 * j));
  }
  internal_to_original_dof_.Apply(permuted_z, y);
}

VectorX<double> BlockSparseCholeskySolver::Solve(
    const VectorX<double>& y) const {
  VectorX<double> x(y);
  SolveInPlace(&x);
  return x;
}

void BlockSparseCholeskySolver::SetMatrixImpl(
    const SymmetricBlockSparseMatrix<double>& A,
    const std::vector<std::set<int>>& adjacency_graph,
    const std::vector<int>& elimination_ordering) {
  DRAKE_DEMAND(elimination_ordering.size() == adjacency_graph.size());
  block_cols_ = elimination_ordering.size();
  vector<int> dof_permutation(elimination_ordering.size() * 3);
  for (int i = 0; i < static_cast<int>(elimination_ordering.size()); ++i) {
    for (int d = 0; d < 3; ++d) {
      dof_permutation[3 * i + d] = 3 * elimination_ordering[i] + d;
    }
  }
  internal_to_original_ = PartialPermutation(move(elimination_ordering));
  internal_to_original_dof_ = PartialPermutation(move(dof_permutation));

  L_diag_.resize(block_cols_);

  std::vector<std::vector<int>> sparsitiy_pattern =
      CalcSparsityPattern(adjacency_graph, internal_to_original_.permutation());
  L_ = SymmetricBlockSparseMatrix<double>(std::move(sparsitiy_pattern));
  UpdateMatrix(A);
}

void BlockSparseCholeskySolver::FactorImpl(int starting_col_block,
                                           int ending_col_block) {
  for (int j = starting_col_block; j < ending_col_block; ++j) {
    /* Update diagonal. */
    const Matrix3<double>& Ajj = L_.get_diagonal_block(j);
    const auto llt = Eigen::LLT<Matrix3<double>>(Ajj);
    DRAKE_DEMAND(llt.info() == Eigen::Success);
    L_diag_[j] = llt.matrixL();
    /* Technically, there's no need to spell out the diagonal block of the L
     matrix, but we do it here for easy debugging. */
    L_.SetBlockFlat(0, j, L_diag_[j]);

    /* Update column.
     | a₁₁  *  | = | λ₁₁  0 | * | λ₁₁ᵀ L₂₁ᵀ |
     | a₂₁ a₂₂ |   | L₂₁ L₂₂|   |  0   L₂₂ᵀ |
     So we have
      L₂₁λ₁₁ᵀ = a₂₁, and thus
      λ₁₁L₂₁ᵀ = a₂₁ᵀ */
    const std::vector<int>& blocks_in_col_j = L_.get_col_blocks(j);
    for (int a = 0; a < static_cast<int>(blocks_in_col_j.size()) - 1; ++a) {
      const int flat = a + 1;
      const auto& L_diag_j = L_diag_[j].triangularView<Eigen::Lower>();
      const Matrix3<double>& Aij = L_.get_block_flat(flat, j);
      Matrix3<double> Lij = L_diag_j.solve(Aij.transpose()).transpose();
      L_.SetBlockFlat(flat, j, std::move(Lij));
    }
    RightLookingSymmetricRank1Update(j);
  }
  is_factored_ = true;
}

void BlockSparseCholeskySolver::RightLookingSymmetricRank1Update(int j) {
  const std::vector<int>& blocks_in_col_j = L_.get_col_blocks(j);
  const int N = blocks_in_col_j.size();
  /* The following omp parallel for loop is equivalent to this easier to read
   non-openmp compliant for loop. */
  /*
   // We start from f1 = 1 here to skip the j,j entry.
   for (int f1 = 1; f1 < N; ++f1) {
     const int col = blocks_in_col_j[f1];
     const Matrix3<double>& B = L_.get_block_flat(f1, j);
     for (int f2 = f1; f2 < N; ++f2) {
       const int row = blocks_in_col_j[f2];
       const Matrix3<double>& A = L_.get_block_flat(f2, j);
       L_.SubtractProductFromBlock(row, col, A, B);
     }
   }
  */
// const int M = N - 1;
// const int size = N * M / 2;
// #if defined(_OPENMP)
// #pragma omp parallel for num_threads(12)
// #endif
//   for (int x = 0; x < size; ++x) {
//     int f2 = int((std::sqrt(8*x + 1) - 1) / 2) + 1;
//     int f1 = x - (f2*(f2-1)/2) + 1;
//     const int col = blocks_in_col_j[f1];
//     const int row = blocks_in_col_j[f2];
//     const Matrix3<double>& A = L_.get_block_flat(f2, j);
//     const Matrix3<double>& B = L_.get_block_flat(f1, j);
//     L_.SubtractProductFromBlock(row, col, A, B);
//   }
#if defined(_OPENMP)
#pragma omp parallel for num_threads(12)
#endif
  for (int a = 0; a < N - 1; ++a) {
    const int f1 = a + 1;
    const int col = blocks_in_col_j[f1];
    const Matrix3<double>& B = L_.get_block_flat(f1, j);
    for (int f2 = f1; f2 < N; ++f2) {
      const int row = blocks_in_col_j[f2];
      const Matrix3<double>& A = L_.get_block_flat(f2, j);
      L_.SubtractProductFromBlock(row, col, A, B);
    }
  }
  /*
    std::vector<std::array<int, 2>> index_map;
    index_map.reserve(N * N / 2);
    for (int f1 = 1; f1 < N; ++f1) {
      for (int f2 = f1; f2 < N; ++f2) {
        index_map.push_back({f1, f2});
      }
    }
  #if defined(_OPENMP)
  #pragma omp parallel for num_threads(12)
  #endif
    for (int i = 0; i < static_cast<int>(index_map.size()); ++i) {
      const int f1 = index_map[i][0];
      const int f2 = index_map[i][1];
      const int col = blocks_in_col_j[f1];
      const int row = blocks_in_col_j[f2];
      const Matrix3<double>& B = L_.get_block_flat(f1, j);
      const Matrix3<double>& A = L_.get_block_flat(f2, j);
      L_.SubtractProductFromBlock(row, col, A, B);
    }
  */
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
