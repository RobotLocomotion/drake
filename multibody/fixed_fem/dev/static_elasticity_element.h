#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Traits class for FEM static elasticity. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel>
struct StaticElasticityElementTraits
    : public ElasticityElementTraits<IsoparametricElement, Quadrature,
                                     ConstitutiveModel> {
  /* The static elasticity problem does not involve time derivatives. */
  static constexpr int kODEOrder = 0;
};

/** The FEM element class for static 3D elasticity problems. Implements the
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
class StaticElasticityElement final
    : public ElasticityElement<
          IsoparametricElement, Quadrature, ConstitutiveModel,
          StaticElasticityElement<IsoparametricElement, Quadrature,
                                  ConstitutiveModel>,
          StaticElasticityElementTraits<IsoparametricElement, Quadrature,
                                        ConstitutiveModel>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StaticElasticityElement);
  using Traits = StaticElasticityElementTraits<IsoparametricElement, Quadrature,
                                               ConstitutiveModel>;
  /** Type of this class. */
  using ElementType = StaticElasticityElement<IsoparametricElement, Quadrature,
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
  using FemElementType::solution_dimension;

  /** Constructs a new FEM static elasticity element. */
  StaticElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, num_nodes()>& node_indices, const T& density,
      const ConstitutiveModel& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, solution_dimension(),
                                           num_nodes()>>& reference_positions)
      : ElasticityElementType(element_index, node_indices, density,
                              constitutive_model, reference_positions) {}

 private:
  /* Friend the base class so that the interface in the CRTP base class can
   access the private implementations of this class. */
  friend FemElementType;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<ElementType>& state,
                      Vector<T, num_dofs()>* residual) const {
    /* residual = -fₑ(x) + fₑₓₜ. */
    DRAKE_ASSERT(residual != nullptr);
    ElasticityElementType::CalcNegativeElasticForce(state, residual);
    // TODO(xuchenhan-tri): Add external force component.
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
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "damping matrix.");
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> M) const {
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "mass matrix.");
  }
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
