#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fem {
/** Traits class for FEM static elasticity. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct StaticElasticityElementTraits
    : ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType> {
  /* The static elasticity problem does not involve time derivatives. */
  static constexpr int kOdeOrder = 0;
};

/** The FEM element class for static 3D elasticity problems. Implements the
 CRTP base class, ElasticityElement. See ElasticityElement for documentation on
 the template paramenters. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class StaticElasticityElement final
    : public ElasticityElement<
          IsoparametricElementType, QuadratureType, ConstitutiveModelType,
          StaticElasticityElement<IsoparametricElementType, QuadratureType,
                                  ConstitutiveModelType>,
          StaticElasticityElementTraits<IsoparametricElementType,
                                        QuadratureType,
                                        ConstitutiveModelType>> {
 public:
  /** Assignment and copy constructions are prohibited. Move constructor is
   allowed so that %StaticElasticityElement can be stored in `std::vector`. */
  StaticElasticityElement(const StaticElasticityElement&) = delete;
  StaticElasticityElement(StaticElasticityElement&&) = default;
  const StaticElasticityElement& operator=(const StaticElasticityElement&) =
      delete;
  StaticElasticityElement&& operator=(const StaticElasticityElement&&) = delete;

  using Traits =
      StaticElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                    ConstitutiveModelType>;
  using T = typename Traits::T;

  /** Constructs a new FEM static elasticity element.
   @pre density > 0. */
  StaticElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, Traits::kSolutionDimension,
                                           Traits::kNumNodes>>&
          reference_positions,
      const T& density, const Vector<T, Traits::kSpatialDimension>& gravity)
      : ElasticityElement<
            IsoparametricElementType, QuadratureType, ConstitutiveModelType,
            StaticElasticityElement<IsoparametricElementType, QuadratureType,
                                    ConstitutiveModelType>,
            Traits>(element_index, node_indices, constitutive_model,
                    reference_positions, density, gravity) {}

 private:
  /** Type alias for convenience and readability. */
  using ElementType =
      StaticElasticityElement<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType>;
  using ElasticityElementType =
      ElasticityElement<IsoparametricElementType, QuadratureType,
                        ConstitutiveModelType, ElementType, Traits>;
  using FemElementType = FemElement<ElementType, Traits>;

  /* Friend the base class so that the public methods in the CRTP base class can
   access the private implementations of this class. */
  friend FemElementType;
  friend class StaticElasticityElementTest;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<ElementType>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    /* residual = -fₑ(x) - fₑₓₜ, where fₑ(x) is the elastic force and fₑₓₜ is
     the external force. */
    this->AddNegativeElasticForce(state, residual);
    this->AddScaledExternalForce(state, -1, residual);
  }

  /* Implements FemElement::CalcStiffnessMatrix(). */
  void DoCalcStiffnessMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    this->AddNegativeElasticForceDerivative(state, K);
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "damping matrix.");
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "mass matrix.");
  }
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
