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

/* Helper class that linearizes torques dτ/dv from plant input ports using
finite differences. The linerization is approximate because it only keeps
diagonal terms and forces them to be non-negative, so that the ICF problem
remains convex.

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

  @param[in] h The time step used to advance positions during differencing.
    TODO(jwnimmer-tri) This doesn't seem like a thing we should depend on?
  @param[in,out] mutable_plant_context The plant context to linearize around,
    used both as input (for the current state and inputs) and to mutate during
    differencing while computing changes in torque due to changes in state, but
    all mutations are undone prior to returning.
  @param[out] actuation_feedback The linearization of actuation torques, valid
    iff the `has_actuation_forces` return value is true.
  @param[out] actuation_feedback The linearization of external torques, valid
    iff the `has_external_forces` return value is true.
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
    VectorX<T> gu0;       // Actuation gu(x) = B u(x) at x₀.
    VectorX<T> ge0;       // External forces ge(x) = τ_ext(x) at x₀.
    VectorX<T> x_prime;   // Perturbed plant state x' for finite differences.
    VectorX<T> gu_prime;  // Perturbed actuation gu(x') = B u(x').
    VectorX<T> ge_prime;  // Perturbed external forces ge(x') = τ_ext(x').
    MatrixX<T> N;         // Kinematic map q̇ = N v.
  };

  /* Computes external forces τ = τₑₓₜ(x) from the plant's spatial and
  generalized force input ports.
  @param[out] tau Output storage for τ. */
  void CalcExternalForces(const systems::Context<T>& plant_context,
                          VectorX<T>* tau) const;

  /* Computes actuator forces τ = B u(x) from the plant's actuation input ports
  (including the general actuation input port and any model-instance-specific
  input ports).
  @param[out] tau Output storage for τ. */
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
