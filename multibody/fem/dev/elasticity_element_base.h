#pragma once

#include <vector>

#include "drake/multibody/fem/dev/fem_element.h"
#include "drake/multibody/fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri): Remove this class after issue #14302 is resolved.
/** %ElasticityElementBase serves two purposes.
 1. It provides non-templatized ElasticityElement functionalities shared by the
 templatized derived classes.
 2. It provides non-templatized API's specific to ElasticityElement that do not
 exist in FemElement. */
template <typename T>
class ElasticityElementBase : public FemElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityElementBase);

  virtual ~ElasticityElementBase() = default;

  /** The number of dimensions of the elasticity problem. */
  int solution_dimension() const final { return 3; }

  /** Returns the elastic potential energy stored in this element in unit J. */
  virtual T CalcElasticEnergy(const FemState<T>& state) const = 0;

  /** Calculates the element stiffness matrix of this element, evaluated
   at `state`. The stiffness is given by the second derivative of the elastic
   energy with respect to the vertex positions.
   @param[in] state The FEM state at which to evaluate the stiffness matrix.
   @param[out] K The stiffness matrix to be calculated.
   @pre K must be of size 3*num_nodes()-by-3*num_nodes(). */
  virtual void CalcStiffnessMatrix(const FemState<T>& state,
                                   EigenPtr<MatrixX<T>> K) const = 0;

 protected:
  ElasticityElementBase(ElementIndex element_index,
                        const std::vector<NodeIndex>& node_indices)
      : FemElement<T>(element_index, node_indices) {}
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
