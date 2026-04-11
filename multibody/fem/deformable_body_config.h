#pragma once

#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace multibody {
namespace fem {

// TODO(xuchenhang-tri): Move everything in this file outside of the FEM module
// to be shared with MPM.
/** Types of material models for the deformable body. */
enum class MaterialModel {
  /** Linear corotational model as described in [Han et al., 2023]. It provides
   a combination of accuracy, robustness, and speed. Recommended in most
   scenarios.
   [Han et al., 2023] Han, Xuchen, Joseph Masterjohn, and Alejandro Castro. "A
   Convex Formulation of Frictional Contact between Rigid and Deformable
   Bodies." arXiv preprint arXiv:2303.08912 (2023). */
  kLinearCorotated,
  /** Corotational model. More computationally expensive. Recommended when
   capturing large rotation velocity is important.

   [Stomakhin et al., 2012] Stomakhin, Alexey, et al. "Energetically consistent
   invertible elasticity." Proceedings of the 11th ACM SIGGRAPH/Eurographics
   conference on Computer Animation. 2012. */
  kCorotated,
  /** Neohookean model. More computationally expensive. Recommended when
   capturing large rotation velocity is important. There are subtle differences
   in physical behavior between the Corotated model and the Neo-Hookean model.
   See [Smith et al., 2019; Stomakhin et al., 2012] for details. In practice, we
   often choose the Neo-Hookean model over the Corotated model simply because
   it's computationally more efficient.

   [Smith et al., 2019] Smith, Breannan, Fernando De Goes, and Theodore Kim.
   "Stable Neo-Hookean flesh simulation." ACM Transactions on Graphics
   (TOG) 37.2 (2018): 1-15. */
  kNeoHookean,
  /** Linear elasticity model (rarely used).
   Less computationally expensive than other models but leads to artifacts
   when large rotational deformations occur. */
  kLinear,
};

/** %DeformableBodyConfig stores the physical parameters for a deformable body.
 A default constructed configuration approximately represents a hard rubber
 material (density, elasticity, and poisson's ratio) without any damping.
 Damping coefficients are generally difficult to measure and we expect users
 will typically start with zero damping and tune the values to achieve
 reasonable dynamics.
 The config contains the following fields with their corresponding valid ranges:
 - Young's modulus: Measures the stiffness of the material, has unit N/m². Must
   be positive. Default to 1e8.
 - Poisson's ratio: Measures the Poisson effect (how much the material expands
   or contracts in directions perpendicular to the direction of loading) of the
   material, unitless. Must be greater than -1 and less than 0.5. Default to
   0.49.
   <!-- TODO(xuchenhan-tri): Add a reference to Rayleigh damping coefficients.
   -->
 - Mass damping coefficient: Controls the strength of mass damping, has unit
   1/s. Must be non-negative. Default to 0. See DampingModel.
 - Stiffness damping coefficient: Controls the strength of stiffness damping,
   has unit s. Must be non-negative. Default to 0. See DampingModel.
 - Mass density: Has unit kg/m³. Must be positive. Default to 1.5e3.
 - Material model: The constitutive model that describes the stress-strain
   relationship of the body, see MaterialModel. Default to
   MaterialModel::kLinearCorotated.
 - Element subdivision count: to integrate external volumetric forces, elements
   can be subdivided to resolve large non-linearities within the domain of the
   element. The number of resulting quadrature points is equal to 4^N where N is
   the subdivision count. Zero means no subdivision. Default to 0.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DeformableBodyConfig {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableBodyConfig);

  DeformableBodyConfig() = default;

  /** @pre youngs_modulus > 0. */
  void set_youngs_modulus(T youngs_modulus) {
    DRAKE_THROW_UNLESS(youngs_modulus > 0);
    youngs_modulus_ = std::move(youngs_modulus);
  }

  /** @pre -1 < poissons_ratio < 0.5. */
  void set_poissons_ratio(T poissons_ratio) {
    DRAKE_THROW_UNLESS(-1 < poissons_ratio && poissons_ratio < 0.5);
    poissons_ratio_ = std::move(poissons_ratio);
  }

  /** @pre mass_damping_coefficient >= 0. */
  void set_mass_damping_coefficient(T mass_damping_coefficient) {
    DRAKE_THROW_UNLESS(mass_damping_coefficient_ >= 0);
    mass_damping_coefficient_ = std::move(mass_damping_coefficient);
  }

  /**  @pre stiffness_damping_coefficient >= 0. */
  void set_stiffness_damping_coefficient(T stiffness_damping_coefficient) {
    DRAKE_THROW_UNLESS(stiffness_damping_coefficient_ >= 0);
    stiffness_damping_coefficient_ = std::move(stiffness_damping_coefficient);
  }

  /**  @pre mass_density > 0. */
  void set_mass_density(T mass_density) {
    DRAKE_THROW_UNLESS(mass_density > 0);
    mass_density_ = std::move(mass_density);
  }

  void set_material_model(MaterialModel material_model) {
    material_model_ = material_model;
  }

  /** @pre 0 <= element_subdivision_count <= 4. */
  void set_element_subdivision_count(int element_subdivision_count) {
    DRAKE_THROW_UNLESS(element_subdivision_count >= 0);
    DRAKE_THROW_UNLESS(element_subdivision_count <= 4);
    element_subdivision_count_ = element_subdivision_count;
  }

  /** Returns the Young's modulus, with unit of N/m². */
  const T& youngs_modulus() const { return youngs_modulus_; }
  /** Returns the Poisson's ratio, unitless. */
  const T& poissons_ratio() const { return poissons_ratio_; }
  /** Returns the mass damping coefficient. See DampingModel. */
  const T& mass_damping_coefficient() const {
    return mass_damping_coefficient_;
  }
  /** Returns the stiffness damping coefficient. See DampingModel. */
  const T& stiffness_damping_coefficient() const {
    return stiffness_damping_coefficient_;
  }
  /** Returns the mass density, with unit of kg/m³. */
  const T& mass_density() const { return mass_density_; }
  /** Returns the constitutive model of the material. */
  MaterialModel material_model() const { return material_model_; }
  /** Returns the number of times each element is subdivided when evaluating the
   external forces. Useful when elements are too coarse to resolve external
   force fields. */
  int element_subdivision_count() const { return element_subdivision_count_; }

 private:
  T youngs_modulus_{1e8};
  T poissons_ratio_{0.49};
  T mass_damping_coefficient_{0};
  T stiffness_damping_coefficient_{0};
  T mass_density_{1.5e3};
  MaterialModel material_model_{MaterialModel::kLinearCorotated};
  int element_subdivision_count_{0};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake
