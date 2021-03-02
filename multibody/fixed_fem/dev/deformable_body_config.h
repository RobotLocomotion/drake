#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
// TODO(xuchenhan-tri): Add unit tests for this class.
/** %DeformableBodyConfig stores the physical parameters for a deformable body.
 It contains the following fields with their corresponding valid ranges:
 - Youngs modulus: Measures the stiffness of the material, has unit Pa. Must be
   positive.
 - Poisson ratio: Measures the Poisson effect (how much the material expand or
   contract in directions perpendicular to the direction of loading) of the
   material, unitless. Must be greater than -1 and less than 0.5.
 - Mass damping coefficient: Controls the strength of mass damping. The damping
   ratio contributed by mass damping is inversely proportional to the frequency
   of the motion. Must be non-negative.
 - Stiffness damping coefficient: Controls the strength of stiffness damping.
   The damping ratio contributed by stiffness damping is proportional to the
   frequency of the motion. Must be non-negative.
 - Mass density: Has unit kg/m³. Must be positive.
 - Material model: The constitutive model that describe the stress-strain
   relationship of the body. See MaterialModel below. Must not be
   MaterialModel::Invalid. */
template <typename T>
class DeformableBodyConfig {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableBodyConfig);
  DeformableBodyConfig() = default;

  /** Types of material models for the deformable body. */
  enum class MaterialModel {
    /** Linear elasticity model. Recommended for modeling small deformations.
     Less computationally expensive than non-linear models but is inaccurate
     for large deformations. */
    Linear,
    /** Corotational model. Recommended for modeling large deformations. More
     computationally expensive than the linear elasticity model. */
    Corotated,
    Invalid,
  };

  /** Returns false if any body config is invalid. Returns true otherwise. */
  bool IsValid() const {
    if (youngs_modulus_ <= 0) {
      return false;
    }
    if (poisson_ratio_ <= -1 || poisson_ratio_ >= 0.5) {
      return false;
    }
    if (mass_damping_coefficient_ < 0) {
      return false;
    }
    if (stiffness_damping_coefficient_ < 0) {
      return false;
    }
    if (mass_density_ <= 0) {
      return false;
    }
    if (material_model_ == MaterialModel::Invalid) {
      return false;
    }
    return true;
  }

  /* Setters. */
  /** @pre youngs_modulus > 0. */
  void set_youngs_modulus(const T& youngs_modulus) {
    DRAKE_DEMAND(youngs_modulus > 0);
    youngs_modulus_ = youngs_modulus;
  }

  /** @pre -1 < poisson_ratio < 0.5. */
  void set_poisson_ratio(const T& poisson_ratio) {
    DRAKE_DEMAND(-1 < poisson_ratio && poisson_ratio < 0.5);
    poisson_ratio_ = poisson_ratio;
  }

  /** @pre mass_damping_coefficient >= 0. */
  void set_mass_damping_coefficient(const T& mass_damping_coefficient) {
    DRAKE_DEMAND(mass_damping_coefficient_ >= 0);
    mass_damping_coefficient_ = mass_damping_coefficient;
  }

  /**  @pre stiffness_damping_coefficient >= 0. */
  void set_stiffness_damping_coefficient(
      const T& stiffness_damping_coefficient) {
    DRAKE_DEMAND(stiffness_damping_coefficient_ >= 0);
    stiffness_damping_coefficient_ = stiffness_damping_coefficient;
  }

  /**  @pre mass_density > 0. */
  void set_mass_density(const T& mass_density) {
    DRAKE_DEMAND(mass_density > 0);
    mass_density_ = mass_density;
  }

  void set_material_model(MaterialModel material_model) {
    material_model_ = material_model;
  }

  /* Getters. */
  const T& youngs_modulus() const { return youngs_modulus_; }
  const T& poisson_ratio() const { return poisson_ratio_; }
  const T& mass_damping_coefficient() const {
    return mass_damping_coefficient_;
  }
  const T& stiffness_damping_coefficient() const {
    return stiffness_damping_coefficient_;
  }
  const T& mass_density() const { return mass_density_; }
  MaterialModel material_model() const { return material_model_; }

 private:
  T youngs_modulus_{0};
  T poisson_ratio_{-1};
  T mass_damping_coefficient_{0};
  T stiffness_damping_coefficient_{0};
  T mass_density_{0};
  MaterialModel material_model_{MaterialModel::Invalid};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
