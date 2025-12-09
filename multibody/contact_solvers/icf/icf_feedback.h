#pragma once

#include <memory>
#include <tuple>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/icf/icf_linear_feedback_gains.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Utility object that (partially) linearizes all the external (controller)
systems connected to the plant with finite differences.

Torques from externally connected systems are given by
    τ = B u(x) + τₑₓₜ(x),
which we will approximate as
    τ ≈ clamp(-Kᵤ v + bᵤ) - Kₑ v + bₑ,
where Kᵤ, Kₑ are diagonal and positive semi-definite.

Note that contributions due to controls u(x) will be clamped to effort limits,
while contributions due to external generalized and spatial forces τₑₓₜ(x) will
not be.

@tparam_nonsymbolic_scalar */
template <class T>
class IcfFeedback {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfFeedback);

  /* Constructs using the given plant.
  The plant pointer is retained by this object so must outlive us. */
  explicit IcfFeedback(const MultibodyPlant<T>* plant);

  ~IcfFeedback();

  /* Linearizes torques as explained in the class overview.

  Returns a tuple [has_actuation_forces, has_external_forces] where a given
  element is true iff any input is connected for the corresponding type. */
  std::tuple<bool, bool> LinearizeExternalSystem(
      const T& h, systems::Context<T>& mutable_plant_context,
      IcfLinearFeedbackGains<T>* actuation_feedback,
      IcfLinearFeedbackGains<T>* external_feedback) const;

 private:
  /* Preallocated scratch space. */
  struct Scratch {
    /* Sizes the member fields to accommodate the given plant. */
    explicit Scratch(const MultibodyPlant<T>& plant);
    ~Scratch();

    std::unique_ptr<MultibodyForces<T>> f_ext;
    VectorX<T> v0;        // Unperturbed plant velocities.
    VectorX<T> gu0;       // Actuation gu(x) = B u(x) at x₀.
    VectorX<T> ge0;       // External forces ge(x) = τ_ext(x) at x₀.
    VectorX<T> x_prime;   // Perturbed plant state x' for finite differences.
    VectorX<T> gu_prime;  // Perturbed actuation gu(x') = B u(x').
    VectorX<T> ge_prime;  // Perturbed external forces ge(x') = τ_ext(x').
    MatrixX<T> N;         // Kinematic map q̇ = N v.
  };

  /* Computes external forces τ = τₑₓₜ(x) from the plant's spatial and
  generalized force input ports. */
  void CalcExternalForces(const systems::Context<T>& plant_context,
                          VectorX<T>* tau) const;

  /* Computes actuator forces τ = B u(x) from the plant's actuation input ports
  (including the general actuation input port and any model-instance-specific
  input ports). */
  void CalcActuationForces(const systems::Context<T>& plant_context,
                           VectorX<T>* tau) const;

  const MultibodyPlant<T>& plant_;

  // Pre-allocated scratch space for intermediate calculations.
  mutable Scratch scratch_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfFeedback);
