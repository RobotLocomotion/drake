#pragma once

#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/parallelism.h"
#include "drake/multibody/fem/corotated_model.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/fem/linear_constitutive_model.h"
#include "drake/multibody/fem/linear_corotated_model.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* TODO(xuchenhan-tri): Move constitutive models outside of the FEM module so
 that it can be more easily shared with MPM code. */
template <typename T>
using ConstitutiveModelVariant =
    std::variant<fem::internal::CorotatedModel<T>,
                 fem::internal::LinearCorotatedModel<T>,
                 fem::internal::LinearConstitutiveModel<T>>;

template <typename T>
using DeformationGradientDataVariant =
    std::variant<fem::internal::CorotatedModelData<T>,
                 fem::internal::LinearCorotatedModelData<T>,
                 fem::internal::LinearConstitutiveModelData<T>>;

/* The collection of all physical attributes we care about for all particles in
 a full MPM model. All quantities are measured and expressed in the world frame
 (when applicable).
 @tparam T The scalar type, can be a double, float, or AutoDiffXd. */
template <typename T>
class ParticleData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ParticleData);

  using Scalar = T;

  /* Constructs an empty particle data with no particle. */
  ParticleData();

  int num_particles() const { return ssize(m_); }

  /* Accessors */
  const std::vector<T>& m() const { return m_; }
  const std::vector<Vector3<T>>& x() const { return x_; }
  const std::vector<Vector3<T>>& v() const { return v_; }
  const std::vector<Matrix3<T>>& F() const { return F_; }
  const std::vector<Matrix3<T>>& F0() const { return F0_; }
  const std::vector<Matrix3<T>>& C() const { return C_; }
  const std::vector<T>& volume() const { return volume_; }
  const std::vector<ConstitutiveModelVariant<T>>& constitutive_models() const {
    return constitutive_models_;
  }
  const std::vector<bool>& in_constraint() const { return in_constraint_; }
  const std::vector<Matrix3<T>>& tau_volume() const { return tau_volume_; }

  /* Mutable accessors */
  std::vector<Vector3<T>>& mutable_x() { return x_; }
  std::vector<Vector3<T>>& mutable_v() { return v_; }
  std::vector<Matrix3<T>>& mutable_F() { return F_; }
  std::vector<Matrix3<T>>& mutable_F0() { return F0_; }
  std::vector<Matrix3<T>>& mutable_C() { return C_; }
  std::vector<bool>& mutable_in_constraint() { return in_constraint_; }
  std::vector<Matrix3<T>>& mutable_tau_volume() { return tau_volume_; }

  /* Computes the total elastic potential energy stored in the particles
   assuming that the deformation gradients and their previous time step values
   are given by `F` and `F0`.
   @pre F and F0 have the same size and ordering as this ParticleData.
   @note The deformation gradients (and their previous time step values) stored
   in `this` ParticleData are not necessarily used for the computation; the
   passed in F and F0 are used instead. */
  T ComputeTotalEnergy(const std::vector<Matrix3<T>>& F,
                       const std::vector<Matrix3<T>>& F0) const;

  /* Computes the volume scaled Kirchhoff stress of each particle assuming that
   the deformation gradients and their previous time step values are given by
   `F` and `F0`.
   @param[in] F            The deformation gradients of the particles.
   @param[in] F0           The deformation gradients at the previous time step.
   @param[out] tau_volume  The Kirchhoff stress of each particle scaled by the
                           reference volume of the particle.
   @param[in] parallelism  Specifies the degree of parallelism to use.
   @pre F and F0 have the same size and ordering as this ParticleData.
   @pre volume_scaled_stress != nullptr.
   @warn F and F0 are used to compute the strain, but the deformation gradients
   stored in `this` ParticleData are used to convert First-Piola stress into
   Kirchhoff stress τ. In other words, `tau_volume` is given by

    volume * τ = volume * ∂Ψ(F, F0)/∂F * Fpᵀ

   where F and F0 are supplied as arguments, and Fp is the deformation gradients
   stored in `this` ParticleData. */
  void ComputeKirchhoffStress(const std::vector<Matrix3<T>>& F,
                              const std::vector<Matrix3<T>>& F0,
                              std::vector<Matrix3<T>>* tau_volume,
                              Parallelism parallelism = false) const;

  /* Computes the volume scaled first Piola-Kirchhoff stress derivatives of each
   particle assuming that the deformation gradients and their previous time step
   values are given by `F` and `F0`.
   @param[in] F            The deformation gradients of the particles.
   @param[in] F0           The deformation gradients at the previous time step.
   @param[in] parallelism  Specifies the degree of parallelism to use.
   @pre F and F0 have the same size and ordering as this ParticleData.
   @pre dPdF_volume != nullptr.
   @note The deformation gradients (and their previous time step values) stored
   in `this` ParticleData are not necessarily used for the computation; the
   passed in F and F0 are used instead. */
  void ComputePK1StressDerivatives(
      const std::vector<Matrix3<T>>& F, const std::vector<Matrix3<T>>& F0,
      std::vector<math::internal::FourthOrderTensor<T>>* dPdF_volume,
      Parallelism parallelism = false) const;

  // TODO(xuchenhan-tri): Support Rayleigh damping for MPM.
  /* Appends default particle data to `this` ParticleData using the given
   parameters. The velocities of the particles are set to zeros. The deformation
   gradients are set to identities. This function can be called repeatedly on a
   single ParticleData object to add particles in multiple passes.
   @param positions     The positions of the new particles in the world frame.
   @param total_volume  The per particle volume of the new particles is given by
                        total_volume divided by the number of new particles.
                        This is using the assumption that the new particles are
                        evenly distributed across the domain.
   @param config        Provides the constitutive models and the mass densities
                        of the new particles.
   @pre total_volume > 0. */
  void AddParticles(const std::vector<Vector3<double>>& positions,
                    double total_volume,
                    const fem::DeformableBodyConfig<double>& config);

  /* Per particle state and data. All of the following fields have the same
   size and ordering. */
  /* State */
  std::vector<Vector3<T>> x_;  // positions
  std::vector<Vector3<T>> v_;  // velocity
  std::vector<Matrix3<T>> F_;  // deformation gradient
  std::vector<Matrix3<T>>
      F0_;                     // deformation gradient at the previous time step
  std::vector<Matrix3<T>> C_;  // affine velocity field
  /* Constant data. */
  std::vector<T> m_;       // mass
  std::vector<T> volume_;  // reference volume
  std::vector<ConstitutiveModelVariant<T>> constitutive_models_;
  /* State dependent data. */
  std::vector<bool> in_constraint_;  // whether the particle is participating
                                     // in a constraint
  std::vector<Matrix3<T>>
      tau_volume_;  // Kirchhoff stress scaled by reference volume
  mutable std::vector<DeformationGradientDataVariant<T>>
      deformation_gradient_data_;  // Deformation gradient dependent data that
                                   // is used to calculate the energy density
                                   // and its derivatives.
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
