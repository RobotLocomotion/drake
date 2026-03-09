#pragma once

#include <map>
#include <set>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/fem/fem_indexes.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* The position, velocity, and acceleration of an FEM node.
 @tparam_default_scalar */
template <typename T>
struct NodeState {
  Vector3<T> q;
  Vector3<T> v;
  Vector3<T> a;
};

/* DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) on FEM models [Zienkiewicz et al. Sec 1.4]. In
 particular, it provides the following functionalities:
 1. storing the information necessary to apply the BC;
 2. modifying a given state to comply with the stored BC;
 3. modifying a given tangent matrix/residual that arises from the FEM model
 without BC and transform it into the tangent matrix/residual for the same
 model under the stored BC.

 We only allow specifying BCs at the node level. That means that either all
 degrees of freedom (DoFs) of a node are subject to BCs or none of the DoFs of
 the node is subject to BCs. For example, we don't allow specifying BCs for
 just the x-components of the states while leaving the y and z components free.

 Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu. The finite element
 method: its basis and fundamentals. Elsevier, 2005.

 @tparam_default_scalar */
template <typename T>
class DirichletBoundaryCondition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirichletBoundaryCondition);

  /* Constructs an empty boundary condition. */
  DirichletBoundaryCondition() {}

  /* Sets the node with the given `index` to be subject to the prescribed
   `boundary_state`. If multiple boundary conditions are specified for the same
    node, the last one will be used.
   @pre index.is_valid() and values provided in `boundary_state` are finite. */
  void AddBoundaryCondition(FemNodeIndex index,
                            const NodeState<T>& boundary_state);

  /* Merges the boundary conditions specified by `other` into this
   boundary condition. If multiple boundary conditions are specified for the
   same node, the last one from `other` will be used. */
  void Merge(const DirichletBoundaryCondition<T>& other);

  /* Returns the boundary conditions as an `std::map` with node indices as keys
   and the prescribed boundary states as values. */
  const std::map<FemNodeIndex, NodeState<T>>& index_to_boundary_state() const {
    return node_to_boundary_state_;
  }

  /* Modifies the given `fem_state` to comply with this boundary condition.
   @pre fem_state != nullptr.
   @pre fem_state is an "owned" FEM state (see FemState).
   @throw std::exception if any node index specified by `this` boundary
   condition is greater than or equal to the `fem_state->num_nodes()`. */
  void ApplyBoundaryConditionToState(FemState<T>* fem_state) const;

  /* Modifies the given tangent matrix that arises from an FEM model into the
   tangent matrix for the same model subject to this BC. More specifically,
   the rows and columns corresponding to DoFs under this BC will be zeroed out
   with the exception of the diagonal entries for those DoFs which will be set
   to be non-zero.
   @pre tangent_matrix != nullptr.
   @pre Indices of all DoFs associated with nodes subject to `this` BC are
   smaller than `tangent_matrix->cols()`. */
  void ApplyBoundaryConditionToTangentMatrix(
      contact_solvers::internal::BlockSparseSymmetricMatrix3d* tangent_matrix)
      const;

  /* Modifies the given vector `v` (e.g, the residual of the system or the
   velocities/positions) that arises from an FEM model without BC into the a
   vector for the same model subject to `this` BC. More specifically, the
   entries corresponding to nodes under the BC will be zeroed out.
   @pre v != nullptr.
   @throw std::exception if the index of any DoF associated with nodes subject
   to `this` BC is greater than or equal to the `v->size()`. */
  void ApplyHomogeneousBoundaryCondition(EigenPtr<VectorX<T>> v) const;

 private:
  /* Throws an exception if the largest node index specified by this BC is
   larger than or equal to the given `size`. */
  void VerifyIndices(int size) const;

  /* We sort the boundary conditions according to node indices for better
   cache consistency when applying the BC. */
  std::map<FemNodeIndex, NodeState<T>> node_to_boundary_state_{};

  /* FemNodeIndex of nodes under BC (represented as ints). */
  std::set<int> node_indices_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
