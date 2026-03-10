#include "drake/multibody/contact_solvers/icf/icf_external_systems_linearizer.h"

#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/multibody/plant/desired_state_input.h"
#include "drake/multibody/plant/multibody_plant_icf_attorney.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using multibody::internal::DesiredStateInput;
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

  // Check which feedback values might be non-zero.
  bool has_actuation_input =
      plant_.get_actuation_input_port().HasValue(plant_context);
  bool has_desired_state_input = false;
  for (ModelInstanceIndex i{0}; i < plant_.num_model_instances(); ++i) {
    has_actuation_input =
        has_actuation_input ||
        plant_.get_actuation_input_port(i).HasValue(plant_context);
    has_desired_state_input =
        has_desired_state_input ||
        plant_.get_desired_state_input_port(i).HasValue(plant_context);
  }
  const bool has_external_force_input =
      plant_.get_applied_generalized_force_input_port().HasValue(
          plant_context) ||
      plant_.get_applied_spatial_force_input_port().HasValue(plant_context);

  // Maybe there's nothing to do.
  if (!has_actuation_input && !has_desired_state_input &&
      !has_external_force_input) {
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

  // We only need to use forward differences for these two terms.
  if (has_actuation_input || has_external_force_input) {
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
    if (has_actuation_input) {
      // tau_u0ᵢ is really only needed if Kuᵢ < 0.
      // TODO(amcastro-tri): Only compute if any Kuᵢ < 0.
      CalcActuationForces(plant_context, &tau_u0);
    }
    if (has_external_force_input) {
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
    if (has_actuation_input) {
      CalcActuationForces(plant_context, &tau_u_tilde0);
    }
    if (has_external_force_input) {
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
      // Choose a step size (the same way as how implicit_integrator.cc does
      // it).
      const T abs_vi = abs(v_tilde0(i));
      T dvi = (abs_vi <= 1) ? T{eps} : eps * abs_vi;

      // Adjust Δvᵢ to be an exactly representable number while adding it to
      // v′ᵢ.
      v_prime(i) += dvi;
      dvi = v_prime(i) - v_tilde0(i);

      // Perturb q as well, using the fact that q′ = q̃₀ + h N₀ dv.
      // TODO(jwnimmer-tri) This seems wasteful; the (v_prime - v_tilde0) is
      // sparse (only one element is non-zero) and typically N0 is also
      // sparse. Consider using plant.MapVelocityToQDot or similar.
      q_prime += h * N0 * (v_prime - v_tilde0);

      // Put x′ in the context and mark the state as stale.
      mutable_plant_context.SetContinuousState(x_prime);

      if (has_actuation_input) {
        // Compute the relevant matrix entries for actuation inputs:
        //   Ku = -dgu/dv
        CalcActuationForces(mutable_plant_context, &tau_u_prime);
        Ku(i) = -(tau_u_prime(i) - tau_u_tilde0(i)) / dvi;

        if (Ku(i) > 0.0) {
          // Implicit approximation.
          bu(i) = tau_u_tilde0(i) + Ku(i) * v0(i);
        } else {
          // Explicit approximation.
          Ku(i) = 0.0;
          bu(i) = tau_u0(i);
        }
      }

      if (has_external_force_input) {
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
  }

  // Add in any desired_state PD terms.
  if (has_desired_state_input) {
    if (!has_actuation_input) {
      Ku.setZero();
      bu.setZero();
    }
    using Attorney = multibody::internal::MultibodyPlantIcfAttorney<T>;
    const DesiredStateInput<T>& desired_state =
        Attorney::EvalDesiredStateInput(plant_, plant_context);
    const Eigen::VectorBlock<const VectorX<T>> full_q =
        plant_.GetPositions(plant_context);
    for (const typename DesiredStateInput<T>::Item& item :
         desired_state.items) {
      const JointActuator<T>& actuator =
          plant_.get_joint_actuator(item.actuator_index);
      const Joint<T>& joint = actuator.joint();
      const PdControllerGains& gains = actuator.get_controller_gains();
      const T& Kp = gains.p;
      const T& Kd = gains.d;
      const T& q0 = full_q[joint.position_start()];
      const T& qd = item.qd;
      const T& vd = item.vd;
      // The PD controller's force contribution is:
      //    -Kp⋅(q - qd) - Kd⋅(v - vd)
      //  = -Kp⋅((q₀ + h⋅v) - qd) - Kd⋅(v - vd)
      //  = -Kp⋅q₀ - Kp⋅h⋅v + Kp⋅qd - Kd⋅v + Kd⋅vd
      // Regrouping into terms with `v` vs terms without gives:
      //  = (-Kp⋅h⋅v - Kd⋅v) + (-Kp⋅q₀ + Kp⋅qd + Kd⋅vd)
      //  = -(Kp⋅h + Kd)⋅v + (-Kp⋅(q₀ - qd) + Kd⋅vd)
      // Relating that to our chosen linearization τᵤ(v) ≈ clamp(-Kᵤ⋅v + bᵤ; e),
      // we can read out the terms:
      Ku[joint.velocity_start()] += h * Kp + Kd;
      bu[joint.velocity_start()] += -Kp * (q0 - qd) - Kd * vd;
    }
  }

  return {has_actuation_input || has_desired_state_input,
          has_external_force_input};
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
  const VectorX<T>& u = Attorney::EvalActuationInput(
      plant_, context, /* apply_effort_limit = */ false);
  for (JointActuatorIndex actuator_index : plant_.GetJointActuatorIndices()) {
    const JointActuator<T>& actuator =
        plant_.get_joint_actuator(actuator_index);
    const Joint<T>& joint = actuator.joint();
    DRAKE_DEMAND(joint.num_velocities() == 1);
    (*tau)[joint.velocity_start()] += u[actuator.input_start()];
  }
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
