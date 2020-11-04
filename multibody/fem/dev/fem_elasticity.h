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
 @tparam IsoparametricElementType The type of IsoparametricElement used in this
 ElasticityElement.
 @tparam QuadratureType The type of Quadrature used in this ElasticityElement.
 */
/* TODO(xuchenhan-tri): Consider making num_quads() and num_nodes() available at
 compile time and thereby eliminating heap allocations. */
/* TODO(xuchenhan-tri): Consider abstracting out the IsoparametricElement and
 the Quadrature to a FixedSizeFemElement class. */
template <typename T, class IsoparametricElementType, class QuadratureType>
class ElasticityElement : public FemElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityElement);

  /** Constructs a new FEM elasticity element.
   @param[in] element_index The global index of the new element.
   @param[in] shape The shape function to be used for this element.
   @param[in] quadrature The quadrature rule to be used for this element.
   @param[in] node_indices The global node indices of the nodes of this
   element.
   @param[in] param The ElasticityElementParameters to be used for this element.
   @param[in] constitutive_model The ConstitutiveModel to be used for this
   element.
   @pre element_index must be valid.
   @pre @p shape.num_nodes() must be the same as the size of @p node_indices.
   @pre @p shape.num_nodes() must be the same as the number of columns of @p
   param.reference_positions.
   @warning The input `constitutive_model` must be compatible with the
   DeformationGradientCache in the ElasticityElementCache that shares the same
   element index with this %ElasticityElement. More specifically, if the
   DeformationGradientCache in the corresponding ElasticityElementCache is of
   type "FooModelCache", then the input `constitutive_model` must be of type
   "FooModel". */
  ElasticityElement(ElementIndex element_index,
                    const IsoparametricElementType& shape,
                    const QuadratureType& quadrature,
                    const std::vector<NodeIndex>& node_indices,
                    const ElasticityElementParameters<T>& param,
                    std::unique_ptr<ConstitutiveModel<T>> constitutive_model);

  virtual ~ElasticityElement() = default;

  /** The number of dimensions of the elasticity problem. */
  int num_problem_dim() const final { return 3; }

  /** Number of quadrature points at which element-wise quantities are
   evaluated. */
  int num_quads() const { return quadrature_.num_points(); }

  /** Number of nodes associated with this element. */
  int num_nodes() const final { return shape_.num_nodes(); }

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
  using MatrixD3 =
      Eigen::Matrix<T, IsoparametricElementType::kNumNaturalDim, 3>;

  friend class ElasticityElementTest;

  /* Calculates the elastic force on the nodes in this element. Returns a vector
   of elastic force of size 3*num_nodes(). */
  void CalcNegativeElasticForce(const FemState<T>& state,
                                EigenPtr<VectorX<T>> force) const;

  /* Calculates the deformation gradient at all quadrature points in the this
   element. */
  void CalcDeformationGradient(const FemState<T>& state,
                               std::vector<Matrix3<T>>* F) const;

  /* Evaluates the DeformationGradientCache for this element. */
  // TODO(xuchenhan-tri): This method unconditionally recomputes the
  // DeformationGradientCache. Enable caching when caching is in place.
  const DeformationGradientCache<T>& EvalDeformationGradientCache(
      const FemState<T>& state) const;

  /* Calculates the elastic energy density per unit reference volume, in unit
   J/m³, at each quadrature point in this element. */
  void CalcElasticEnergyDensity(const FemState<T>& state,
                                std::vector<T>* Psi) const;

  /* Calculates the first Piola stress, in unit Pa, at each quadrature point in
   this element. */
  void CalcFirstPiolaStress(const FemState<T>& state,
                            std::vector<Matrix3<T>>* P) const;

  /* The isoparametric shape function used for this element. */
  IsoparametricElementType shape_;
  /* The quadrature rule used for this element. */
  QuadratureType quadrature_;
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
