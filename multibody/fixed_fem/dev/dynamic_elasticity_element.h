#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/damping_model.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fem {
/** Traits class for FEM dynamic elasticity. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct DynamicElasticityElementTraits
    : public ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                     ConstitutiveModelType> {
  /* The dynamic elasticity problem forms a second order ODE. */
  static constexpr int kOdeOrder = 2;
};

/** The FEM element class for dynamic 3D elasticity problems that model
 inertia effects and damping forces. Implements the CRTP base class,
 ElasticityElement. See ElasticityElement for documentation on the template
 paramenters. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class DynamicElasticityElement final
    : public ElasticityElement<
          IsoparametricElementType, QuadratureType, ConstitutiveModelType,
          DynamicElasticityElement<IsoparametricElementType, QuadratureType,
                                   ConstitutiveModelType>,
          DynamicElasticityElementTraits<IsoparametricElementType,
                                         QuadratureType,
                                         ConstitutiveModelType>> {
 public:
  /** Assignment and copy constructions are prohibited. Move constructor is
   allowed so that DynamicElasticityElement can be stored in `std::vector`. */
  DynamicElasticityElement(const DynamicElasticityElement&) = delete;
  DynamicElasticityElement(DynamicElasticityElement&&) = default;
  const DynamicElasticityElement& operator=(const DynamicElasticityElement&) =
      delete;
  DynamicElasticityElement&& operator=(const DynamicElasticityElement&&) =
      delete;

  using Traits =
      DynamicElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                     ConstitutiveModelType>;
  using T = typename Traits::T;

  /** Constructs a new FEM dynamic elasticity element.
   @pre density > 0. */
  DynamicElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, Traits::kSolutionDimension,
                                           Traits::kNumNodes>>&
          reference_positions,
      const T& density, const Vector<T, Traits::kSpatialDimension>& gravity,
      const DampingModel<T>& damping_model)
      : ElasticityElementType(element_index, node_indices, constitutive_model,
                              reference_positions, density, gravity),
        damping_model_(damping_model) {}

 private:
  /* Type alias for convenience and readability. */
  using ElementType =
      DynamicElasticityElement<IsoparametricElementType, QuadratureType,
                               ConstitutiveModelType>;
  using ElasticityElementType =
      ElasticityElement<IsoparametricElementType, QuadratureType,
                        ConstitutiveModelType, ElementType, Traits>;
  using FemElementType = FemElement<ElementType, Traits>;
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend FemElementType;
  friend class DynamicElasticityElementTest;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<ElementType>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    /* residual = Ma-fₑ(x)-fᵥ(x, v)-fₑₓₜ, where M is the mass matrix, fₑ(x) is
     the elastic force, fᵥ(x, v) is the damping force and fₑₓₜ is the external
     force. */
    *residual += this->mass_matrix() *
                 this->ExtractElementDofs(this->node_indices(), state.qddot());
    this->AddNegativeElasticForce(state, residual);
    AddNegativeDampingForce(state, residual);
    this->AddScaledExternalForce(state, -1.0, residual);
  }

  /* Adds the negative damping force on the nodes of this element into the given
   `negative_damping_force`. The negative damping force is given by the product
   of the damping matrix with the velocity of the nodes. */
  void AddNegativeDampingForce(
      const FemState<ElementType>& state,
      EigenPtr<Vector<T, Traits::kNumDofs>> negative_damping_force) const {
    Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> damping_matrix;
    this->CalcDampingMatrix(state, &damping_matrix);
    /* Note that the damping force fᵥ = -D * v, where D is the damping matrix.
     As we are accumulating the negative damping force here, the `+=` sign
     should be used. */
    *negative_damping_force +=
        damping_matrix *
        this->ExtractElementDofs(this->node_indices(), state.qdot());
  }

  /* Implements FemElement::CalcStiffnessMatrix().
   @warning This method calculates a first-order approximation of the stiffness
   matrix. In other words, the contribution of the term ∂fᵥ(x, v)/∂x is ignored
   as it involves second derivatives of the elastic force. */
  void DoCalcStiffnessMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    this->AddNegativeElasticForceDerivative(state, K);
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    /* D = αM + βK, where α is the mass damping coefficient and β is the
     stiffness damping coefficient. */
    this->CalcStiffnessMatrix(state, D);
    *D *= damping_model_.stiffness_coeff();
    *D += damping_model_.mass_coeff() * this->mass_matrix();
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>&,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    *M = ElasticityElementType::mass_matrix();
  }

  DampingModel<T> damping_model_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
