#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"

#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/multibody_plant_icf_attorney.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using systems::Context;

template <typename T>
IcfExternalSystemsLinearizer<T>::IcfExternalSystemsLinearizer(
    const MultibodyPlant<T>* plant)
    : plant_(DRAKE_DEREF(plant)), scratch_(plant_) {}

template <typename T>
IcfExternalSystemsLinearizer<T>::~IcfExternalSystemsLinearizer() = default;

template <typename T>
std::tuple<bool, bool> IcfExternalSystemsLinearizer<T>::LinearizeExternalSystem(
    const T& h, Context<T>& mutable_plant_context,
    IcfLinearFeedbackGains<T>* actuation_feedback,
    IcfLinearFeedbackGains<T>* external_feedback) const {
  using std::abs;
  using std::isfinite;
  DRAKE_ASSERT(isfinite(h) && h > 0);
  DRAKE_ASSERT(actuation_feedback != nullptr);
  DRAKE_ASSERT(external_feedback != nullptr);
  const Context<T>& plant_context = mutable_plant_context;
  plant_.ValidateContext(plant_context);

  // Check which (or both) of the two feedback values we actually need.
  const bool has_actuation_forces = plant_.num_actuators() > 0;
  const bool has_external_forces =
      plant_.get_applied_generalized_force_input_port().HasValue(
          plant_context) ||
      plant_.get_applied_spatial_force_input_port().HasValue(plant_context);

  // Maybe there's nothing to do.
  if (!has_actuation_forces && !has_external_forces) {
    return {false, false};
  }

  // Get references to the feedback gains that we'll set as output.
  VectorX<T>& Ku = actuation_feedback->K;
  VectorX<T>& bu = actuation_feedback->b;
  VectorX<T>& Ke = external_feedback->K;
  VectorX<T>& be = external_feedback->b;
  DRAKE_ASSERT(Ku.size() == plant_.num_velocities());
  DRAKE_ASSERT(bu.size() == plant_.num_velocities());
  DRAKE_ASSERT(Ke.size() == plant_.num_velocities());
  DRAKE_ASSERT(be.size() == plant_.num_velocities());

  // Get references to pre-allocated variables.
  VectorX<T>& tau_u0 = scratch_.tau_u0;
  VectorX<T>& tau_e0 = scratch_.tau_e0;
  VectorX<T>& x_tilde0 = scratch_.x_tilde0;
  VectorX<T>& tau_u_tilde0 = scratch_.tau_u_tilde0;
  VectorX<T>& tau_e_tilde0 = scratch_.tau_e_tilde0;
  VectorX<T>& x_prime = scratch_.x_prime;
  VectorX<T>& tau_u_prime = scratch_.tau_u_prime;
  VectorX<T>& tau_e_prime = scratch_.tau_e_prime;

  // Compute some quantities that depend on the current state, before messing
  // with the state with finite differences.
  // TODO(jwnimmer-tri) Avoid the memory allocation of N0.
  const MatrixX<T> N0 = plant_.MakeVelocityToQDotMap(plant_context);
  if (has_actuation_forces) {
    // tau_u0ᵢ is really only needed if Kuᵢ < 0.
    // TODO(amcastro-tri): Only compute if any Kuᵢ < 0.
    CalcActuationForces(plant_context, &tau_u0);
  }
  if (has_external_forces) {
    // tau_e0ᵢ is really only needed if Keᵢ < 0.
    // TODO(amcastro-tri): Only compute if any Keᵢ < 0.
    CalcExternalForces(plant_context, &tau_e0);
  }

  // Initial state.
  // TODO(jwnimmer-tri) Avoid the memory allocation of x0.
  const int nq = plant_.num_positions();
  const int nv = plant_.num_velocities();
  const VectorX<T> x0 =
      plant_context.get_continuous_state_vector().CopyToVector();
  auto v0 = x0.segment(nq, nv);

  // Compute the state x̃(v) at v₀, x̃₀ = x̃(v₀) = [q₀ + h⋅N₀⋅v₀; v₀].
  x_tilde0 = x0;
  auto q_tilde0 = x_tilde0.head(nq);
  auto v_tilde0 = x_tilde0.segment(nq, nv);
  q_tilde0 += h * N0 * v0;

  // Store x̃₀ in plant_context.
  mutable_plant_context.SetContinuousState(x_tilde0);
  if (has_actuation_forces) {
    CalcActuationForces(plant_context, &tau_u_tilde0);
  }
  if (has_external_forces) {
    CalcExternalForces(plant_context, &tau_e_tilde0);
  }

  // Perturbed state.
  x_prime = x_tilde0;
  auto q_prime = x_prime.head(nq);
  auto v_prime = x_prime.segment(nq, nv);

  // Compute τ̃(v) = b - K⋅(v − v₀), using forward differences.
  // We do this separately for τᵤ(x) and τₑ(x), as needed by ICF.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int i = 0; i < nv; ++i) {
    // Choose a step size (the same way as how implicit_integrator.cc does it).
    const T abs_vi = abs(v_tilde0(i));
    T dvi = (abs_vi <= 1) ? T{eps} : eps * abs_vi;

    // Adjust Δvᵢ to be an exactly representable number while adding it to v′ᵢ.
    v_prime(i) += dvi;
    dvi = v_prime(i) - v_tilde0(i);

    // Perturb q as well, using the fact that q′ = q̃₀ + h N₀ dv.
    // TODO(jwnimmer-tri) This seems wasteful; the (v_prime - v_tilde0) is
    // sparse (only one element is non-zero) and typically N0 is also
    // sparse. Consider using plant.MapVelocityToQDot or similar.
    q_prime += h * N0 * (v_prime - v_tilde0);

    // Put x′ in the context and mark the state as stale.
    mutable_plant_context.SetContinuousState(x_prime);

    if (has_actuation_forces) {
      // Compute the relevant matrix entries for actuation inputs:
      //   Ku = -dgu/dv
      CalcActuationForces(mutable_plant_context, &tau_u_prime);
      Ku(i) = -(tau_u_prime(i) - tau_u_tilde0(i)) / dvi;

      if (Ku(i) > 0.0) {
        // Implicit approximation.
        bu(i) = tau_u_tilde0(i) + Ku(i) * v0(i);
      } else {
        // Explicit approximation.
        // TODO(#23918) None of our test cases reach this case.
        Ku(i) = 0.0;
        bu(i) = tau_u0(i);
      }
    }

    if (has_external_forces) {
      // Same thing, but for external forces:
      //  Ke = -dge/dv
      CalcExternalForces(mutable_plant_context, &tau_e_prime);
      Ke(i) = -(tau_e_prime(i) - tau_e_tilde0(i)) / dvi;
      if (Ke(i) > 0.0) {
        // Implicit approximation.
        be(i) = tau_e_tilde0(i) + Ke(i) * v0(i);
      } else {
        // Explicit approximation.
        Ke(i) = 0.0;
        be(i) = tau_e0(i);
      }
    }

    // Reset the state for the next iteration.
    q_prime(i) = q_tilde0(i);
    v_prime(i) = v_tilde0(i);
  }

  // Reset the context back to how we found it.
  mutable_plant_context.SetContinuousState(x0);

  return {has_actuation_forces, has_external_forces};
}

template <typename T>
IcfExternalSystemsLinearizer<T>::Scratch::Scratch(
    const MultibodyPlant<T>& plant)
    : f_ext(plant) {
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();

  tau_u0.resize(nv);
  tau_e0.resize(nv);

  x_tilde0.resize(nq + nv);
  tau_u_tilde0.resize(nv);
  tau_e_tilde0.resize(nv);

  x_prime.resize(nq + nv);
  tau_u_prime.resize(nv);
  tau_e_prime.resize(nv);
}

template <typename T>
IcfExternalSystemsLinearizer<T>::Scratch::~Scratch() = default;

template <typename T>
void IcfExternalSystemsLinearizer<T>::CalcActuationForces(
    const Context<T>& context, VectorX<T>* tau) const {
  using Attorney = multibody::internal::MultibodyPlantIcfAttorney<T>;
  tau->setZero();
  Attorney::AddJointActuationForces(plant_, context, tau);
}

template <typename T>
void IcfExternalSystemsLinearizer<T>::CalcExternalForces(
    const Context<T>& context, VectorX<T>* tau) const {
  using Attorney = multibody::internal::MultibodyPlantIcfAttorney<T>;
  MultibodyForces<T>& forces = scratch_.f_ext;
  forces.SetZero();
  Attorney::AddAppliedExternalSpatialForces(plant_, context, &forces);
  Attorney::AddAppliedExternalGeneralizedForces(plant_, context, &forces);
  plant_.CalcGeneralizedForces(context, forces, tau);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        IcfExternalSystemsLinearizer);
