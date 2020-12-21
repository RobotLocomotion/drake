#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/damping_model.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Traits class for FEM dynamic elasticity. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct DynamicElasticityElementTraits
    : public ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                     ConstitutiveModelType> {
  /* The dynamic elasticity problem forms a second order ODE. */
  static constexpr int kOdeOrder = 2;
};

/** The FEM element class for dynamic 3D elasticity problems. Implements the
 CRTP base class, FemElement.
 @tparam IsoparametricElementType    The type of isoparametric element used in
 this %DynamicElasticityElement. IsoparametricElementType must be a derived
 class from IsoparametricElement.
 @tparam QuadratureType    The type of quadrature rule used in this
 %DynamicElasticityElement. QuadratureType must be a derived class from
 Quadrature.
 @tparam ConstitutiveModelType    The type of constitutive model used in this
 %DynamicElasticityElement. %ConstitutiveModelType must be a derived class from
 ConstitutiveModel. */
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

  /** Constructs a new FEM dynamic elasticity element. */
  DynamicElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, Traits::kSolutionDimension,
                                           Traits::kNumNodes>>&
          reference_positions,
      const T& density, const DampingModel& damping_model)

      : ElasticityElementType(element_index, node_indices, constitutive_model,
                              reference_positions),
        density_(density),
        damping_model_(damping_model),
        mass_matrix_(PrecomputeMassMatrix()) {}

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
    /* residual = Ma-fₑ(x)-fᵥ(x, v)+fₑₓₜ, where M is the mass matrix, fₑ(x) is
     the elastic force, fᵥ(x, v) is the damping force and fₑₓₜ is the external
     force. */
    ElasticityElementType::AddNegativeElasticForce(state, residual);
    AddNegativeDampingForce(state, residual);
    *residual += mass_matrix_ * state.qddot();
    // TODO(xuchenhan-tri): Add external force component.
  }

  /* Adds the negative damping force on the nodes of this element into the given
   `negative_damping_force`. The negative damping force is given by the product
   of the damping matrix with the velocity of the nodes. */
  void AddNegativeDampingForce(
      const FemState<ElementType>& state,
      EigenPtr<Vector<T, Traits::kNumDofs>> negative_damping_force) const {
    Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> damping_matrix;
    FemElementType::CalcDampingMatrix(state, &damping_matrix);
    /* The sign is correct here because fᵥ = -D * v, where D is the damping
     matrix. */
    *negative_damping_force += damping_matrix * state.qdot();
  }

  /* Implements FemElement::CalcStiffnessMatrix().
   @warning This method calculates a first-order approximation of the stiffness
   matrix. In other words, the contribution of the term ∂fᵥ(x, v)/∂x is ignored
   as it involves second derivatives of the elastic force. */
  void DoCalcStiffnessMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    ElasticityElementType::AddNegativeElasticForceDerivative(state, K);
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    /* D = αM + βK, where α is the mass damping coefficient and β is the
     stiffness damping coefficient. */
    FemElementType::CalcStiffnessMatrix(state, D);
    *D *= damping_model_.stiffness_damping;
    *D += damping_model_.mass_damping * mass_matrix_;
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    *M = mass_matrix_;
  }

  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> PrecomputeMassMatrix()
      const {
    Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> mass =
        Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Zero();
    const std::array<Vector<T, Traits::kNumNodes>,
                     Traits::kNumQuadraturePoints>& S =
        ElasticityElementType::isoparametric_element().GetShapeFunctions();
    /* S_mat is the matrix representation of S. */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumQuadraturePoints> S_mat;
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      S_mat.col(q) = S[q];
    }
    /* weighted_shape calculates the shape function weighted by the reference
     volume of the quadrature point. */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumQuadraturePoints>
        weighted_S(S_mat);
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      weighted_S.col(q) *= ElasticityElementType::reference_volume()[q];
    }
    /* weighted_SST = weighted_S * Sᵀ. The ij-th entry approximates the integral
     ∫SᵢSⱼ dX */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumNodes> weighted_SST =
        weighted_S * S_mat.transpose();
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      for (int j = 0; j < Traits::kNumNodes; ++j) {
        mass.template block<Traits::kSpatialDimension,
                            Traits::kSpatialDimension>(
            Traits::kSpatialDimension * i, Traits::kSpatialDimension * j) =
            Eigen::Matrix<T, Traits::kSpatialDimension,
                          Traits::kSpatialDimension>::Identity() *
            weighted_SST(i, j) * density_;
      }
    }
    return mass;
  }

  /* The mass density of the element in the reference configuration with
   unit kg/m³. */
  T density_;
  DampingModel damping_model_;
  /* Precomputed mass matrix. */
  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> mass_matrix_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
