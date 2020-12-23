#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Traits class for FEM dynamic elasticity. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel>
struct DynamicElasticityElementTraits
    : public ElasticityElementTraits<IsoparametricElement, Quadrature,
                                     ConstitutiveModel> {
  /* The dynamic elasticity problem forms a second order ODE. */
  static constexpr int kODEOrder = 2;
};

// TODO(xuchenhan-tri): Move the damping model into its own file and add
//  documentation.
/* Rayleigh damping model. */
struct DampingModel {
  double mass_damping_;
  double stiffness_damping_;
};

/** The FEM element class for dynamic 3D elasticity problems. Implements the
 interface provided in the CRTP base class, FemElement.
 @tparam IsoparametricElement The type of isoparametric element used in this
 %ElasticityElement. %IsoparametricElement must be a derived class from
 IsoparametricElement.
 @tparam Quadrature The type of quadrature rule used in this %ElasticityElement.
 %Quadrature must be a derived class from Quadrature.
 @tparam ConstitutiveModel The type of constitutive model used in this
 %ElasticityElement. %ConstitutiveModel must be a derived class from
 ConstitutiveModel. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel>
class DynamicElasticityElement final
    : public ElasticityElement<
          IsoparametricElement, Quadrature, ConstitutiveModel,
          DynamicElasticityElement<IsoparametricElement, Quadrature,
                                   ConstitutiveModel>,
          DynamicElasticityElementTraits<IsoparametricElement, Quadrature,
                                         ConstitutiveModel>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DynamicElasticityElement);
  using Traits = DynamicElasticityElementTraits<IsoparametricElement,
                                                Quadrature, ConstitutiveModel>;
  /** Type of this class. */
  using ElementType = DynamicElasticityElement<IsoparametricElement, Quadrature,
                                               ConstitutiveModel>;
  /** Type of the cache entry compatible with this class. */
  using ElementCacheEntryType = typename Traits::ElementCacheEntryType;
  /** Type of intermediate CRTP class that this class directly inherits from. */
  using ElasticityElementType =
      ElasticityElement<IsoparametricElement, Quadrature, ConstitutiveModel,
                        ElementType, Traits>;
  /** Type of the base CRTP class that this class indirectly inherits from. */
  using FemElementType = FemElement<ElementType, Traits>;
  /* Promoted types and methods from the base class for readability. */
  using T = typename FemElementType::T;
  using FemElementType::num_dofs;
  using FemElementType::num_nodes;
  using FemElementType::num_quadrature_points;
  using FemElementType::solution_dimension;
  using FemElementType::spatial_dimension;

  /** Constructs a new FEM static elasticity element. */
  DynamicElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, num_nodes()>& node_indices,
      const ConstitutiveModel& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, solution_dimension(),
                                           num_nodes()>>& reference_positions,
      const T& density, const DampingModel& damping_model)

      : ElasticityElementType(element_index, node_indices, constitutive_model,
                              reference_positions),
        density_(density),
        damping_model_(damping_model),
        mass_matrix_(PrecomputeMassMatrix()) {}

 private:
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend FemElementType;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<ElementType>& state,
                      EigenPtr<Vector<T, num_dofs()>> residual) const {
    /* residual = Ma-fₑ(x)-fᵥ(x, v)+fₑₓₜ. */
    ElasticityElementType::CalcNegativeElasticForce(state, residual);
    AccumulateNegativeDampingForce(state, residual);
    *residual += GetMassMatrix() * state.qddot();
    // TODO(xuchenhan-tri): Add external force component.
  }

  void AccumulateNegativeDampingForce(
      const FemState<ElementType>& state,
      EigenPtr<Vector<T, num_dofs()>> negative_damping_force) const {
    Eigen::Matrix<T, num_dofs(), num_dofs()> damping_matrix;
    FemElementType::CalcDampingMatrix(state, &damping_matrix);
    /* The sign is correct here because fᵥ = -D * v. */
    *negative_damping_force += damping_matrix * state.qdot();
  }

  /* Implements FemElement::CalcStiffnessMatrix(). */
  void DoCalcStiffnessMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> K) const {
    ElasticityElementType::CalcNegativeElasticForceDerivative(state, K);
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> D) const {
    /* D = αM + βK, where α is the mass damping coefficient and β is the
     stiffness damping coefficient. */
    FemElementType::CalcStiffnessMatrix(state, D);
    *D *= damping_model_.stiffness_damping_;
    *D += damping_model_.mass_damping_ * GetMassMatrix();
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> M) const {
    *M = GetMassMatrix();
  }

  const Eigen::Matrix<T, num_dofs(), num_dofs()>& GetMassMatrix() const {
    return mass_matrix_;
  }

  Eigen::Matrix<T, num_dofs(), num_dofs()> PrecomputeMassMatrix() const {
    Eigen::Matrix<T, num_dofs(), num_dofs()> mass;
    const std::array<Vector<T, num_nodes()>, num_quadrature_points()>& S =
        ElasticityElementType::isoparametric_element().GetShapeFunctions();
    /* S_mat is the matrix representation of S. */
    Eigen::Matrix<T, num_nodes(), num_quadrature_points()> S_mat;
    for (int q = 0; q < num_quadrature_points(); ++q) {
      S_mat.col(q) = S[q];
    }
    /* weighted_shape calculates the shape function weighted by the reference
     volume of the quadrature point. */
    Eigen::Matrix<T, num_nodes(), num_quadrature_points()> weighted_S(S_mat);
    for (int q = 0; q < num_quadrature_points(); ++q) {
      weighted_S.col(q) *= ElasticityElementType::reference_volume()[q];
    }
    /* weighted_SST = weighted_S * Sᵀ. The ij-th entry approximates the integral
     ∫SᵢSⱼ dX */
    Eigen::Matrix<T, num_nodes(), num_nodes()> weighted_SST =
        weighted_S * S_mat.transpose();
    for (int i = 0; i < num_nodes(); ++i) {
      for (int j = 0; j < num_nodes(); ++j) {
        for (int d = 0; d < spatial_dimension(); ++d) {
          mass(spatial_dimension() * i + d, spatial_dimension() * j + d) =
              weighted_SST(i, j) * density_;
        }
      }
    }
    return mass;
  }

  /* The mass density of the element in the reference configuration with
   unit kg/m³. */
  T density_;
  DampingModel damping_model_;
  /* Precomputed mass matrix. */
  Eigen::Matrix<T, num_dofs(), num_dofs()> mass_matrix_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
