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

using multibody::internal::MultibodyPlantIcfAttorney;
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
  DRAKE_ASSERT(actuation_feedback != nullptr);
  DRAKE_ASSERT(external_feedback != nullptr);
  const Context<T>& plant_context = mutable_plant_context;
  plant_.ValidateContext(plant_context);

  // Extract the feedback gains that we'll set.
  VectorX<T>& Ku = actuation_feedback->K;
  VectorX<T>& bu = actuation_feedback->b;
  VectorX<T>& Ke = external_feedback->K;
  VectorX<T>& be = external_feedback->b;
  DRAKE_ASSERT(Ku.size() == plant_.num_velocities());
  DRAKE_ASSERT(bu.size() == plant_.num_velocities());
  DRAKE_ASSERT(Ke.size() == plant_.num_velocities());
  DRAKE_ASSERT(be.size() == plant_.num_velocities());

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

  // Get references to pre-allocated variables.
  VectorX<T>& gu0 = scratch_.gu0;
  VectorX<T>& ge0 = scratch_.ge0;
  VectorX<T>& gu_tilde0 = scratch_.gu_tilde0;
  VectorX<T>& ge_tilde0 = scratch_.ge_tilde0;
  VectorX<T>& x_prime = scratch_.x_prime;
  VectorX<T>& gu_prime = scratch_.gu_prime;
  VectorX<T>& ge_prime = scratch_.ge_prime;
  MatrixX<T>& N0 = scratch_.N0;

  // Compute some quantities that depend on the current state, before messing
  // with the state with finite differences.
  N0 = plant_.MakeVelocityToQDotMap(plant_context);
  if (has_actuation_forces) {
    // gu0ᵢ is really only needed if Kuᵢ < 0.
    // TODO(amcastro-tri): Only compute if any Kuᵢ < 0.
    CalcActuationForces(plant_context, &gu0);
  }
  if (has_external_forces) {
    // gueᵢ is really only needed if Keᵢ < 0.
    // TODO(amcastro-tri): Only compute if any Keᵢ < 0.
    CalcExternalForces(plant_context, &ge0);
  }

  // Initial state.
  const int nq = plant_.num_positions();
  const int nv = plant_.num_velocities();
  const VectorX<T> x0 =
      plant_context.get_continuous_state_vector().CopyToVector();
  auto v0 = x0.segment(nq, nv);

  // Semi-implicit state x̃(v) at v₀, x̃₀ = x̃(v₀) = [q₀ + h⋅N₀⋅v₀; v₀]
  VectorX<T> x_tilde0 = x0;
  auto q_tilde0 = x_tilde0.head(nq);
  auto v_tilde0 = x_tilde0.segment(nq, nv);
  q_tilde0 += h * N0 * v0;

  // Store x̃₀ in plant_context.
  mutable_plant_context.SetContinuousState(x_tilde0);
  if (has_actuation_forces) {
    CalcActuationForces(plant_context, &gu_tilde0);
  }
  if (has_external_forces) {
    CalcExternalForces(plant_context, &ge_tilde0);
  }

  // Perturbed state.
  x_prime = x_tilde0;
  auto q_prime = x_prime.head(nq);
  auto v_prime = x_prime.segment(nq, nv);

  // Compute τ(v) = b -  K⋅(v − v₀), using forward differences.
  // We do this separately for τᵤ(x) and τₑ(x), as needed by ICF.
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int i = 0; i < nv; ++i) {
    // Choose a step size (the same way as how implicit_integrator.cc does it).
    const T abs_vi = abs(v_tilde0(i));
    T dvi = (abs_vi <= 1) ? T{eps} : eps * abs_vi;

    // Ensure that v' and v differ by an exactly representable number.
    v_prime(i) = v_tilde0(i) + dvi;
    dvi = v_prime(i) - v_tilde0(i);

    // Perturb q as well, using the fact that q' = q + h N dv.
    // TODO(jwnimmer-tri) This seems wasteful; the (v_prime - v) is sparse (only
    // one element is non-zero) and typically N is also sparse. Consider using
    // plant.MapVelocityToQDot or similar.
    q_prime = q_tilde0 + h * N0 * (v_prime - v_tilde0);

    // Put x' in the context and mark the state as stale.
    mutable_plant_context.SetContinuousState(x_prime);

    if (has_actuation_forces) {
      // Compute the relevant matrix entries for actuation inputs:
      //   Ku = -dgu/dv
      CalcActuationForces(mutable_plant_context, &gu_prime);
      Ku(i) = -(gu_prime(i) - gu_tilde0(i)) / dvi;

      if (Ku(i) > 0.0) {
        // Implicit approximation.
        bu(i) = gu_tilde0(i) + Ku(i) * v0(i);
      } else {
        // Explicit approximation.
        Ku(i) = 0.0;
        bu(i) = gu0(i);
      }
    }

    if (has_external_forces) {
      // Same thing, but for external forces:
      //  Ke = -dge/dv
      CalcExternalForces(mutable_plant_context, &ge_prime);
      Ke(i) = -(ge_prime(i) - ge_tilde0(i)) / dvi;
      if (Ke(i) > 0.0) {
        // Implicit approximation.
        be(i) = ge_tilde0(i) + Ke(i) * v0(i);
      } else {
        // Explicit approximation.
        Ke(i) = 0.0;
        be(i) = ge0(i);
      }
    }

    // Reset the state for the next iteration.
    v_prime(i) = v_tilde0(i);
  }

  // Reset the context back to how we found it. That means v is now back to v0.
  mutable_plant_context.SetContinuousState(x0);

  return {has_actuation_forces, has_external_forces};
}

template <typename T>
IcfExternalSystemsLinearizer<T>::Scratch::Scratch(
    const MultibodyPlant<T>& plant) {
  const int nv = plant.num_velocities();
  const int nq = plant.num_positions();
  f_ext = std::make_unique<MultibodyForces<T>>(plant);
  gu0.resize(nv);
  ge0.resize(nv);
  gu_tilde0.resize(nv);
  ge_tilde0.resize(nv);
  x_prime.resize(nq + nv);
  gu_prime.resize(nv);
  ge_prime.resize(nv);
  N0.resize(nq, nv);
}

template <typename T>
IcfExternalSystemsLinearizer<T>::Scratch::~Scratch() = default;

template <typename T>
void IcfExternalSystemsLinearizer<T>::CalcExternalForces(
    const Context<T>& context, VectorX<T>* tau) const {
  MultibodyForces<T>& forces = *scratch_.f_ext;
  forces.SetZero();
  MultibodyPlantIcfAttorney<T>::AddAppliedExternalSpatialForces(plant_, context,
                                                                &forces);
  MultibodyPlantIcfAttorney<T>::AddAppliedExternalGeneralizedForces(
      plant_, context, &forces);
  plant_.CalcGeneralizedForces(context, forces, tau);
}

template <typename T>
void IcfExternalSystemsLinearizer<T>::CalcActuationForces(
    const Context<T>& context, VectorX<T>* tau) const {
  tau->setZero();
  MultibodyPlantIcfAttorney<T>::AddJointActuationForces(plant_, context, tau);
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        IcfExternalSystemsLinearizer);
