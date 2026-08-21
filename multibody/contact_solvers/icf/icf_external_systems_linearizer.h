#pragma once

#include <tuple>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/icf/icf_linear_feedback_gains.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Helper class that linearizes external generalized forces with respect to
generalized velocities. The linearization is approximate because it only keeps
diagonal terms and forces them to be non-negative, so that the ICF problem
remains convex.

To enforce effort limit constraints, ICF needs to differentiate generalized
forces due to actuation. We therefore write the total generalized forces as
  τ(x) = τᵤ(x) + τₑ(x),
with the generalized forces due to actuation τᵤ(x)= B⋅u(x) and all other
external sources in τₑ(x) (these could be body forces, drag, or any other
externally applied generalized forces). For ICF, each contribution is linearized
separately to write
  τᵤ(v) ≈ clamp(-Kᵤ⋅v + bᵤ; e),
  τₑ(v) ≈ -Kₑ⋅v + bₑ,
where Kᵤ, Kₑ are positive diagonal matrices, v are the multibody generalized
velocities, and e is the vector of effort limits. For clarity, from now on we
linearize a single term τ(x), with the understanding that we do this separately
for both τᵤ(x) and τₑ(x).

We can derive these approximations by considering the discrete momentum balance.
Without loss of generality, we can ignore contact and other constraints to write
  M⋅(v−v₀) = h⋅k₀ + h⋅τ(q, v),
  q = q₀ + h⋅N(q₀)⋅v.
It is important to note that:
 1. We evaluate actuation and external forces τ(q, v) implicitly for stability.
 2. The scheme is semi-implicit in q for better stability and energy
    conservation properties.

We define x̃(v) = [q₀ + h⋅v; v], and approximate the external generalized forces
as a function of generalized velocities:
  τ̃(v) = τ(x̃(v)) ≈  τ(x̃₀) + dτ/dv(x̃₀)⋅(v − v₀),
with x̃₀ = [q₀ + h⋅v₀; v₀].

Further, we define x₀=[q₀; v₀] and make the component-wise approximation:
  (Implicit) τ̃ᵢ ≈  τᵢ(x̃₀) + dτᵢ/dvᵢ⋅(v − v₀), when dτᵢ/dvᵢ < 0 and,
  (Explicit) τ̃ᵢ ≈  τᵢ(x₀), when dτᵢ/dvᵢ ≥ 0.
That is, this approximation is implicit for stabilizing force terms (such as
most controllers) and explicit otherwise. Within the CENIC framework, error-
control is able to stabilize explicit terms in the scheme when needed.

In total, we write the approximations above as
  τ̃(v) = b - K⋅v,
where K is the positive diagonal matrix with entries
  K = max(0, -diag(dτ/dv)),
and bias term
  bᵢ = τᵢ(x̃₀) + Kᵢ⋅v₀ᵢ, for dτᵢ/dvᵢ(x̃₀) < 0,
  bᵢ = τᵢ(x₀)         , otherwise.

@tparam_nonsymbolic_scalar */
template <class T>
class IcfExternalSystemsLinearizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IcfExternalSystemsLinearizer);

  /* Constructs using the given plant.
  The plant pointer is retained by this object so must outlive us. */
  explicit IcfExternalSystemsLinearizer(const MultibodyPlant<T>* plant);

  ~IcfExternalSystemsLinearizer();

  /* Linearizes generalized forces as explained in the class overview.

  @param[in] h The time step used to advance positions during differencing,
    since the linearization is performed around x̃₀ = [q₀ + h⋅v₀; v₀].
  @param[in] mutable_plant_context The plant context to linearize around,
    used both as input (for the current state and inputs) and to mutate during
    differencing while computing changes in generalized forces due to changes in
    state. All mutations are undone prior to returning.
  @param[out] actuation_feedback The linearization of actuation torques, valid
    on output iff the `has_actuation_forces` return value is true. Must be sized
    to `num_velocities` when passed in.
  @param[out] external_feedback The linearization of external torques, valid
    on output iff the `has_external_forces` return value is true. Must be sized
    to `num_velocities` when passed in.
  Returns a tuple [has_actuation_forces, has_external_forces] where a given
  element is true iff any input is connected for the corresponding type. */
  std::tuple<bool, bool> LinearizeExternalSystem(
      const T& h, systems::Context<T>& mutable_plant_context,
      IcfLinearFeedbackGains<T>* actuation_feedback,
      IcfLinearFeedbackGains<T>* external_feedback) const;

 private:
  /* Pre-allocated scratch space. */
  struct Scratch {
    /* Sizes the member fields to accommodate the given plant. */
    explicit Scratch(const MultibodyPlant<T>& plant);
    ~Scratch();

    MultibodyForces<T> f_ext;

    VectorX<T> tau_u0;  // Actuation τᵤ(x) = B u(x) at x₀.
    VectorX<T> tau_e0;  // External forces τₑ(x) = τ_ext(x) at x₀.

    VectorX<T> x_tilde0;      // Plant state x̃₀ = x̃(v₀).
    VectorX<T> tau_u_tilde0;  // Actuation at τᵤ(x̃₀) at x̃₀ = x̃(v₀).
    VectorX<T> tau_e_tilde0;  // External forces τₑ(x̃₀) at x̃₀ = x̃(v₀).

    VectorX<T> x_prime;      // Perturbed plant state x′ for finite differences.
    VectorX<T> tau_u_prime;  // Perturbed actuation τᵤ(x′) = B u(x′).
    VectorX<T> tau_e_prime;  // Perturbed external forces τₑ(x′) = τ_ext(x′).
  };

  /* Computes actuator forces τ = B u(x) from the plant's actuation input ports
  (including the plant-wide actuation input port and any model-instance-specific
  input ports).
  @param[out] tau Output storage for τ. */
  void CalcActuationForces(const systems::Context<T>& plant_context,
                           VectorX<T>* tau) const;

  /* Computes external forces τ = τₑ(x) from the plant's spatial and
  generalized force input ports.
  @param[out] tau Output storage for τ. */
  void CalcExternalForces(const systems::Context<T>& plant_context,
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
    class ::drake::multibody::contact_solvers::icf::internal::
        IcfExternalSystemsLinearizer);
