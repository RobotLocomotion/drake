#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/fem_indexes.h"
#include "drake/multibody/fem/dev/fem_state.h"

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
 in the routine. This correspondence is maintained by the shared element index
 that is assigned to both the %FemElement and the ElementCache in
 correspondence. Furthermore, the type of %FemElement and ElementCache in
 correspondence must be compatible. More specifically, if the %FemElement is of
 concrete type `FooElement`, then the ElementCache that shares the same element
 index must be of concrete type `FooElementCache`.
 @tparam_nonsymbolic_scalar T.  */
template <typename T>
class FemElement {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemElement);

  virtual ~FemElement() = default;

  /** Global indices of the nodes of this element. */
  const std::vector<NodeIndex>& node_indices() const { return node_indices_; }

  /** Global ElementIndex of this element. */
  ElementIndex element_index() const { return element_index_; }

  /** The dimension of the codomain of the PDE's solution u. */
  // TODO(xuchenhan-tri): Remove this virtual method when we know the sizes of
  // residual, stiffness/tangent matrix, etc at compile time. See issue #14302.
  virtual int solution_dimension() const = 0;

  /** Number of quadrature points at which element-wise quantities are
   evaluated. */
  virtual int num_quadrature_points() const = 0;

  /** Number of nodes associated with this element. */
  virtual int num_nodes() const = 0;

  /** Creates an ElementCache that is compatible with this element. */
  virtual std::unique_ptr<ElementCache<T>> MakeElementCache() const = 0;

  /** Calculates the element residual of this element evaluated at the input
   `state`. This method updates the cached quantities in the input `state` if
   they are out of date.
   @param[in] state The FemState at which to evaluate the residual.
   @param[out] residual The residual vector of size `solution_dimension() *
   num_nodes()`. The vector is ordered such that `i*solution_dimension()`-th to
   `(i+1)*solution_dimension()-1`-th entries of the vector stores the residual
   corresponding to the i-th node in this element.
   @pre `residual` must not be the nullptr, and the vector it points to must
   have size `num_nodes() * solution_dimension()` */
  void CalcResidual(const FemState<T>& state,
                    EigenPtr<VectorX<T>> residual) const;

  // TODO(xuchenhan-tri): Add CalcMassMatrix and CalcStiffnessMatrix etc.

 protected:
  /** Constructs a new FEM element.
    @param[in] element_index The global index of the new element.
    @param[in] node_indices The global node indices of the nodes of this
    element.
    @pre element_index must be valid. */
  FemElement(ElementIndex element_index,
             const std::vector<NodeIndex>& node_indices);

  /** Calculates the element residual of this element evaluated at the input
   state.
   @param[in] state The FemState at which to evaluate the residual.
   @param[out] a vector of residual of size `solution_dimension()*num_nodes()`.
   The vector is ordered such that `i*solution_dimension()`-th to
   `(i+1)*solution_dimension()-1`-th entries of the vector stores the residual
   corresponding to the i-th node in this element.
   @pre residual must not be the nullptr, and the vector it points to must have
   size `num_nodes() * solution_dimension()`. */
  virtual void DoCalcResidual(const FemState<T>& s,
                              EigenPtr<VectorX<T>> residual) const = 0;

 private:
  // The global index of this element.
  ElementIndex element_index_;
  // The global node indices of this element.
  std::vector<NodeIndex> node_indices_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemElement);
