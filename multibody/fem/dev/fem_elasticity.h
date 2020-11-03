#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/dev/constitutive_model.h"
#include "drake/multibody/fem/dev/elasticity_element_cache.h"
#include "drake/multibody/fem/dev/fem_elasticity_parameters.h"
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
 %ElasticityElement.
 @tparam_nonsymbolic_scalar T.
 @tparam NaturalDim The dimension of the parent domain of the FEM elements,
 e.g. 2 for triangles and 3 for tetrahedrons.  */
/* TODO(xuchenhan-tri): Consider templatizing on IsoparametricElement and
 Quadrature so that num_quads() and num_nodes() are available at compile time
 and therefore eliminates most heap allocations. */
template <typename T, int NaturalDim>
class ElasticityElement : public FemElement<T, NaturalDim> {
 public:
  /* Make base class methods visible. */
  using FemElement<T, NaturalDim>::num_nodes;
  using FemElement<T, NaturalDim>::num_quads;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityElement);

  /** Constructs a new FEM elasticity element.
   @param[in] element_index The global index of the new element.
   @param[in] shape The shape function to be used for this element. The
   %ElasticityElement being constructed will assume ownership of the
   IsoparametricElement object.
   @param[in] quadrature The quadrature rule to be used for this element. The
   %ElasticityElement being constructed will assume ownership of the Quadrature
   object.
   @param[in] node_indices The global node indices of the nodes of this
   element.
   @param[in] param The ElasticityElementParameters to be used for this element.
   @param[in] reference_positions The reference positions of the nodes of
   this element. The i-th column of `reference_positions` must contain the
   reference position of the node indexed by the i-th index in the
   `node-indices` argument. Furthermore, the element formed by the nodes with
   the given `reference_positions` must not be degenerate (flat or inverted).
   @param[in] constitutive_model The constitutive model to be used for this
   element.
   @pre element_index must be valid.
   @pre @p shape.num_nodes() must be the same as the size of @p node_indices.
   @pre @p shape.num_nodes() must be the same as the number of columns of @p
   reference_positions.
   @warning The input `constitutive_model` must be compatible with the
   DeformationGradientCache in the ElasticityElementCache that shares the same
   element index with this %ElasticityElement. More specifically, if the
   DeformationGradientCache in the corresponding ElasticityElementCache is of
   type "FooModelCache", then the input `constitutive_model` must be of type
   "FooModel". */
  ElasticityElement(ElementIndex element_index,
                    std::unique_ptr<IsoparametricElement<T, NaturalDim>> shape,
                    std::unique_ptr<Quadrature<T, NaturalDim>> quadrature,
                    const std::vector<NodeIndex>& node_indices,
                    const ElasticityElementParameters<T>& param,
                    std::unique_ptr<ConstitutiveModel<T>> constitutive_model);

  virtual ~ElasticityElement() = default;

  /** The number of dimensions of the elasticity problem. */
  int num_problem_dim() const final { return 3; }

  /** Returns the elastic potential energy stored in this element in unit J. */
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

  using MatrixD3 = Eigen::Matrix<T, NaturalDim, 3>;

  friend class ElasticityElementTest;

  /* Calculates the elastic force on the nodes in this element. Returns a vector
   of elastic force of size 3*num_nodes(). */
  void CalcNegativeElasticForce(const FemState<T>& state,
                                EigenPtr<VectorX<T>> force) const;

  /* Calculates the deformation gradient at all quadrature points in the this
   element. */
  void CalcDeformationGradient(const FemState<T>& state,
                               std::vector<Matrix3<T>>* F) const;

  const DeformationGradientCache<T>& CalcDeformationGradientCache(
      const FemState<T>& state) const;

  /* Calculates the elastic energy density per unit reference volume, in unit
   J/m³, at each quadrature point in this element. */
  void CalcElasticEnergyDensity(const FemState<T>& state,
                                std::vector<T>* Psi) const;

  /* Calculates the first Piola stress, in unit Pa, at each quadrature point in
   this element. */
  void CalcFirstPiolaStress(const FemState<T>& state,
                            std::vector<Matrix3<T>>* P) const;

  /* Parameters used in the element routine. */
  ElasticityElementParameters<T> param_;
  /* The constitutive model that describes the stress-strain relationship for
   this element. */
  std::unique_ptr<ConstitutiveModel<T>> constitutive_model_;
  /* The inverse element Jacobian evaluated at reference configuration at the
   quadrature points in this element. */
  std::vector<MatrixD3> dxidX_;
  /* The volume evaluated at reference configuration occupied by the quadrature
   points in this element. To integrate a function f over the reference domain,
   sum f(q)*reference_volume_[q] over all the quadrature points in the element.
  */
  std::vector<T> reference_volume_;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
