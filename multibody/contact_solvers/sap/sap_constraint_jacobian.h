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
  int clique;
  MatrixBlock<T> block;
};

/* Class to abstract the specific structure of Jacobian matrices for SAP
 constraints.

 In general a SAP constraint will couple the degrees of freedom of two cliques
 (please refer to SapContactProblem's documentation for a definition of
 cliques). Therefore the constraint velocity vc is given as vc = J⋅v = J₁⋅v₁ +
 J₂⋅v₂ where v₁ and v₂ correspond to the vector of generalized velocities for
 the first and second clique respectively (which clique is designated as first
 or second is arbitrary). Similarly, J₁ and J₂ are the non-zero blocks of the
 full Jacobian J that correspond to contributions from the first and second
 cliques respectively. A constraint can also involve only a single clique.
 Examples include a coupler constraint between DOFs in a single clique or a
 contact constraint with the world (the world has no DOFs and therefore has no
 clique.) For the case of a single clique constraint, only one clique and a
 single block J₁ needs to be provided at construction. Refer to the
 SapConstraint class documentation for further details.

 Therefore, the Jacobian matrix for a SAP constraint encodes sparsity at two
 levels:
  1. Per clique: This class stores a matrix (a MatrixBlock) per each clique that
     participates in the constraint. As explained above, only up to two cliques
     can participate in a given constraint.
  2. Within the clique: The MatrixBlock for a given clique might in turn have
     its own sparsity structure within the clique. See MatrixBlock for details.

 TODO(amcastro-tri): consider extension to support constraints among more than
 two cliques, see issue #16575. */
template <typename T>
class SapConstraintJacobian {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapConstraintJacobian);

  /* Constraint Jacobian involving a single clique.
   @throws if `clique` is negative. */
  SapConstraintJacobian(int clique, MatrixBlock<T> J) {
    DRAKE_THROW_UNLESS(clique >= 0);
    clique_jacobians_.resize(1);
    clique_jacobians_[0].clique = clique;
    clique_jacobians_[0].block = std::move(J);
  }

  /* Constraint Jacobian involving a single clique.
   Alternative signature taking a dense matrix block. */
  SapConstraintJacobian(int clique, MatrixX<T> J) {
    DRAKE_THROW_UNLESS(clique >= 0);
    clique_jacobians_.resize(1);
    clique_jacobians_[0].clique = clique;
    clique_jacobians_[0].block = MatrixBlock<T>(std::move(J));
  }

  /* Constraint Jacobian involving two cliques.
   @throws if `first_clique` is negative.
   @throws if `second_clique` is negative.
   @throws first_clique and second_clique are equal.
   @throws if J_first_clique and J_second_clique have different number of rows.
  */
  SapConstraintJacobian(int first_clique, MatrixBlock<T> J_first_clique,
                        int second_clique, MatrixBlock<T> J_second_clique) {
    DRAKE_THROW_UNLESS(first_clique >= 0);
    DRAKE_THROW_UNLESS(second_clique >= 0);
    DRAKE_THROW_UNLESS(first_clique != second_clique);
    DRAKE_THROW_UNLESS(J_first_clique.rows() == J_second_clique.rows());
    clique_jacobians_.resize(2);
    clique_jacobians_[0].clique = first_clique;
    clique_jacobians_[0].block = std::move(J_first_clique);
    clique_jacobians_[1].clique = second_clique;
    clique_jacobians_[1].block = std::move(J_second_clique);
  }

  /* Constraint Jacobian involving two cliques.
   Alternative signature taking dense matrix blocks. */
  SapConstraintJacobian(int first_clique, MatrixX<T> J_first_clique,
                        int second_clique, MatrixX<T> J_second_clique) {
    DRAKE_THROW_UNLESS(first_clique >= 0);
    DRAKE_THROW_UNLESS(second_clique >= 0);
    DRAKE_THROW_UNLESS(first_clique != second_clique);
    DRAKE_THROW_UNLESS(J_first_clique.rows() == J_second_clique.rows());
    clique_jacobians_.resize(2);
    clique_jacobians_[0].clique = first_clique;
    clique_jacobians_[0].block = MatrixBlock<T>(std::move(J_first_clique));
    clique_jacobians_[1].clique = second_clique;
    clique_jacobians_[1].block = MatrixBlock<T>(std::move(J_second_clique));
  }

  /* Returns the number of rows in the constraint Jacobian. Each clique block
   has the same number of rwos. */
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
    DRAKE_DEMAND(local_clique < num_cliques());
    return clique_jacobians_[local_clique].clique;
  }

  /* Returns the Jacobian block for the first clique (local_index = 0) or the
   second clique (local_index = 1). */
  const MatrixBlock<T>& clique_jacobian(int local_clique) const {
    DRAKE_DEMAND(local_clique < num_cliques());
    return clique_jacobians_[local_clique].block;
  }

 private:
  // Blocks for each block. Up to two entries only.
  std::vector<CliqueJacobian<T>> clique_jacobians_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
