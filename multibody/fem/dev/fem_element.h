#pragma once

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/fem_indexes.h"
#include "drake/multibody/fem/dev/fem_state.h"
#include "drake/multibody/fem/dev/isoparametric_element.h"
#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
/** %FemElement is an abstract interface for the "element routines" used in FEM.
 It computes quantities such as the residual and the stiffness matrix on a
 single FEM element given the state of the FEM system. These quantities are
 then assembled into their global counterparts by FemModel.

 %FemElement should be used in tandem with ElementCache. There should be a
 one-to-one correspondence between each %FemElement that performs the element
 routine and each ElementCache that stores the state-dependent quantities used
 in the routine. This correspondence is maintained by the same element index
 that is assigned to the %FemElement and the ElementCache in correspondence.
 Furthermore, the type of %FemElement and ElementCache in correspondence must be
 compatible. More specifically, if the %FemElement is of concrete type `FemFoo`,
 then the ElementCache that shares the same element index must be of concrete
 type `FooElementCache`.
 @tparam_nonsymbolic_scalar T.
 @tparam NaturalDim The dimension of the parent domain of the FEM elements,
 e.g. 2 for triangles and 3 for tetrahedrons.  */
template <typename T, int NaturalDim>
class FemElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemElement);

  virtual ~FemElement() = default;

  const std::vector<NodeIndex>& node_indices() const { return node_indices_; }

  /** Number of quadrature points at which element-wise quantities are
   evaluated. */
  int num_quads() const { return quadrature_.num_points(); }

  /** Number of nodes associated with this element. */
  int num_nodes() const { return shape_.num_nodes(); }

  /** The number of spatial dimensions that this element sits in. */
  virtual int num_spatial_dim() const = 0;

  /** Calculates the element residual of this element evaluated at the input
   state. This method updates the cached quantities in the input FemState if
   they are out of date.
   @param[in] state The FEM state at which to evaluate the residual.
   @returns a vector of residual of size `num_spatial_dim() * num_nodes()`. The
   vector is ordered such that `{i * num_spatial_dim()}`-th to
   `{(i+1) * num_spatial_dim() - 1}`-th entries of the vector stores the
   residual corresponding to the i-th node in this element. */
  VectorX<T> CalcResidual(const FemState<T>& s) const;

  /** Alternative signature that writes the residual in the output argument.
   @pre residual must not be the nullptr, and the vector it points to must have
   size `num_nodes() * num_spatial_dim()`. */
  void CalcResidual(const FemState<T>& s, EigenPtr<VectorX<T>> residual) const;

  // TODO(xuchenhan-tri): Add CalcMassMatrix and CalcStiffnessMatrix etc.

 protected:
  /* Constructs a new FEM element.
    @param[in] element_index The global index of the new element.
    @param[in] shape The shape function to used for this element.
    @param[in] quadrature The quadrature rule to be used for this element.
    @param[in] node_indices The global node indices of the nodes of this
    element.
    @pre element_index must be valid.
    @pre shape.num_nodes() must be the same as size of node_indices. */
  FemElement(ElementIndex element_index,
             const IsoparametricElement<T, NaturalDim>& shape,
             const Quadrature<T, NaturalDim>& quadrature,
             const std::vector<NodeIndex>& node_indices);

  /* Calculates the element residual of this element evaluated at the input
   state.
   @param[in] state The FEM state at which to evaluate the residual.
   @param[out] a vector of residual of size `num_spatial_dim()*num_nodes()`.
   The vector is ordered such that `i*num_spatial_dim()`-th to
   `(i+1)*num_spatial_dim()-1`-th entries of the vector stores the residual
   corresponding to the i-th node in this element.
   @pre residual must not be the nullptr, and the vector it points to must have
   size `num_nodes() * num_spatial_dim()`. */
  virtual void DoCalcResidual(const FemState<T>& s,
                              EigenPtr<VectorX<T>> residual) const = 0;

  // The global index of this element.
  ElementIndex element_index_;
  // The isoparametric shape function used for this element.
  const IsoparametricElement<T, NaturalDim>& shape_;
  // The quadrature rule used for this element.
  const Quadrature<T, NaturalDim>& quadrature_;
  // The global node indices of this element.
  std::vector<NodeIndex> node_indices_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
