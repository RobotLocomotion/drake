#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/elasticity_element_cache.h"
#include "drake/multibody/fem/dev/fem_element.h"
#include "drake/multibody/fem/dev/fem_state.h"
#include "drake/multibody/fem/dev/isoparametric_element.h"
#include "drake/multibody/fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
/** The FEM element routine for static and dynamic 3D elasticity problems.
 Implements the abstract interface of FemElement.

 See ElasticityElementCache for the corresponding ElementCache for
 %FemElasticity.
 @tparam_nonsymbolic_scalar T.
 @tparam NaturalDim The dimension of the parent domain of the FEM elements,
 e.g. 2 for triangles and 3 for tetrahedrons.  */
template <typename T, int NaturalDim>
class FemElasticity : public FemElement<T, NaturalDim> {
 public:
  using MatrixD3 = Eigen::Matrix<T, NaturalDim, 3>;
  /* Make base class methods visible. */
  using FemElement<T, NaturalDim>::num_nodes;
  using FemElement<T, NaturalDim>::num_quads;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemElasticity);

  /** Constructs a new FEM elasticity element.
   @param[in] element_index The global index of the new element.
   @param[in] shape The shape function to used for this element.
   @param[in] quadrature The quadrature rule to be used for this element.
   @param[in] node_indices The global node indices of the nodes of this
   element.
   @param[in] reference_positions The reference positions of the nodes of
   this element.
   @param[in] constitutive_model The constitutive model to be used for this
   element.
   @pre element_index must be valid.
   @pre @p shape.num_nodes() must be the same as size of @p node_indices.
   @pre @p shape.num_nodes() must be the same as the number of columns of @p
   reference_positions.
   @warning The input `constitutive_model` must be compatible with the
   DeformationGradientCache in the ElasticityElementCache that shares the same
   element index with this %FemElasticity. More specifically, if the
   DeformationGradientCache in the corresponding ElasticityElementCache is of
   type "FooModelCache", then the input `constitutive_model` must be of type
   "FooModel". */
  FemElasticity(ElementIndex element_index,
                const IsoparametricElement<T, NaturalDim>& shape,
                const Quadrature<T, NaturalDim>& quadrature,
                const std::vector<NodeIndex>& node_indices,
                const Eigen::Ref<const Matrix3X<T>>& reference_positions,
                const ConstitutiveModel<T>& constitutive_model);

  virtual ~FemElasticity() = default;

  /** The number of spatial dimensions that this element sits in. */
  int num_spatial_dim() const final { return 3; }

  /** Returns the elastic potential energy stored in this element. */
  T CalcElasticEnergy(const FemState<T>& s) const;

 protected:
  /* Calculates the element residual of this element evaluated at the input
   state.
   @param[in] state The FEM state at which to evaluate the residual.
   @returns a vector of residual of size `3 * num_nodes()`. The vector is
   ordered such that `3*i`-th to `3*i+2`-th entries of the vector stores the
   residual corresponding to the i-th node in this element. */
  void DoCalcResidual(const FemState<T>& state,
                      EigenPtr<VectorX<T>> residual) const final;

 private:
  /* Make base class protected members visible. */
  using FemElement<T, NaturalDim>::node_indices_;
  using FemElement<T, NaturalDim>::element_index_;
  using FemElement<T, NaturalDim>::shape_;

  friend class FemElasticityTest;

  /* Calculates the elastic force on the nodes in this element. Returns a vector
   of elastic force of size 3*num_nodes(). */
  void CalcElasticForce(const FemState<T>& state,
                        EigenPtr<VectorX<T>> force) const;

  /* Calculates the deformation gradient at all quadrature points in the this
   element. */
  void CalcF(const FemState<T>& state, std::vector<Matrix3<T>>* F) const;

  const std::unique_ptr<DeformationGradientCache<T>>&
  CalcDeformationGradientCache(const FemState<T>& state) const;

  /* Calculates the elastic energy density at each quadrature point in this
   * element. */
  void CalcPsi(const FemState<T>& state, std::vector<T>* Psi) const;

  /* Calculates the first Piola stress at each quadrature point in this element.
   */
  void CalcP(const FemState<T>& state, std::vector<Matrix3<T>>* P) const;

  /* The constitutive model that describes the stress-strain relationship for
   this element. */
  const ConstitutiveModel<T>& constitutive_model_;
  /* The inverse element Jacobian evaluated at reference configuration at the
   quadrature points in this element. */
  std::vector<MatrixD3> dxidX_;
  /* The volume evaluated at reference configuration occupied by the quadrature
   points in this element. To integrate a function f over an element, sum
   f(q)*volume_[q] over all the quadrature points in the element. */
  std::vector<T> volume_;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
