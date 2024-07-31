#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/matrix_block.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
struct CliqueJacobian {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CliqueJacobian);

  CliqueJacobian(int c, MatrixBlock<T>&& b) : clique(c), block(std::move(b)) {}

  bool operator==(const CliqueJacobian<T>&) const = default;

  int clique;
  MatrixBlock<T> block;
};

// TODO(amcastro-tri): consider extension to support constraints among more than
// two cliques, see issue #16575.

/* Class to abstract the specific structure of Jacobian matrices for SAP
 constraints.

 In general, the constraint velocity vc is given as vc = J⋅v = ∑ᵢ₌₁ᴺJᵢ⋅vᵢ where
 vᵢ is the subvector in the vector of generalized velocities for the i-th clique
 (the ordering of the cliques is arbitrary) and Jᵢ is the non-zero block of the
 full constraint Jacobian J that correspond to contributions from the i-th
 clique. A constraint can only involve a single clique or two cliques, so N = 1
 or 2.

 This class exploits the sparsity in the constraint Jacobian by only storing the
 (at most 2) nonzero Jacobian blocks (as MatrixBlock). Further sparsity
 structure may exist within a single Jacobian block. See MatrixBlock for
 details.

 Therefore, the Jacobian matrix for a SAP constraint encodes sparsity at two
 levels:
  1. Per clique: This class stores a matrix (a MatrixBlock) per each clique that
     participates in the constraint. As explained above, only up to two cliques
     can participate in a given constraint.
  2. Within the clique: The MatrixBlock for a given clique might in turn have
     its own sparsity structure within the clique. See MatrixBlock for details.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapConstraintJacobian {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapConstraintJacobian);

  /* Constraint Jacobian involving a single clique.
   @throws if `clique` is negative. */
  SapConstraintJacobian(int clique, MatrixBlock<T> J);

  /* Constraint Jacobian involving a single clique.
   Alternative signature taking a dense matrix block. */
  SapConstraintJacobian(int clique, MatrixX<T> J);

  /* Constraint Jacobian involving two cliques.
   @throws if `first_clique` is negative.
   @throws if `second_clique` is negative.
   @throws first_clique and second_clique are equal.
   @throws if J_first_clique and J_second_clique have different number of rows.
  */
  SapConstraintJacobian(int first_clique, MatrixBlock<T> J_first_clique,
                        int second_clique, MatrixBlock<T> J_second_clique);

  /* Constraint Jacobian involving two cliques.
   Alternative signature taking dense matrix blocks. */
  SapConstraintJacobian(int first_clique, MatrixX<T> J_first_clique,
                        int second_clique, MatrixX<T> J_second_clique);

  /* Returns the number of rows in the constraint Jacobian. Each clique block
   has the same number of rows. */
  int rows() const {
    // N.B. Constructions guarantees both blocks have the same number of rows.
    // Moreover, the first block always exists.
    return clique_jacobians_[0].block.rows();
  }

  /* Returns the number of cliques, either one or two. */
  int num_cliques() const { return clique_jacobians_.size(); }

  /* Returns the clique index for the first clique (local_index = 0) or the
   second clique (local_index = 1). Refer to the constructor's documentation for
   details. */
  int clique(int local_clique) const {
    DRAKE_DEMAND(0 <= local_clique && local_clique < num_cliques());
    return clique_jacobians_[local_clique].clique;
  }

  /* Returns the Jacobian block for the first clique (local_index = 0) or the
   second clique (local_index = 1). */
  const MatrixBlock<T>& clique_jacobian(int local_clique) const {
    DRAKE_DEMAND(local_clique < num_cliques());
    return clique_jacobians_[local_clique].block;
  }

  /* Returns `true` iff both clique blocks of this Jacobian are dense. */
  bool blocks_are_dense() const;

  // TODO(amcastro-tri): extend this method to support non-dense blocks.
  /* Returns Y = Aᵀ⋅J, with J being `this` Jacobian matrix.
   @pre blocks_are_dense() is true. */
  SapConstraintJacobian<T> LeftMultiplyByTranspose(
      const Eigen::Ref<const MatrixX<T>>& A) const;

  // TODO(amcastro-tri): implement support for non-dense blocks.
  /* When T = double, this method returns a copy of `this` jacobian.
   When T = AutoDiffXd this method returns a copy where gradients were
   discarded.
   @warning Only dense clique jacobians are supported.
   @throws std::exception if any of the Jacobian blocks is not dense. */
  SapConstraintJacobian<double> ToDouble() const;

  bool operator==(const SapConstraintJacobian<T>&) const = default;

 private:
  // Blocks for each block. Up to two entries only.
  std::vector<CliqueJacobian<T>> clique_jacobians_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
