#include "drake/multibody/cenic/continuous_icf_force_manager.h"

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/plant/multibody_plant_icf_attorney.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

using contact_solvers::icf::internal::IcfBuilder;
using systems::Context;

namespace {

// Shifts a spatial force/impulse F given about point Bo to be about point Ao,
// given p_AB (the position of Bo relative to Ao) expressed in the same frame as
// F. With F = [τ; f] (rotational; translational), this returns
// [τ + p_AB × f; f]. This matches ShiftSpatialForce() used internally by
// PatchConstraintsPool::AccumulateGradient().
template <typename T>
Vector6<T> ShiftSpatialForce(const Vector6<T>& F, const Vector3<T>& p_AB) {
  Vector6<T> result;
  result.template head<3>() =
      F.template head<3>() + p_AB.cross(F.template tail<3>());
  result.template tail<3>() = F.template tail<3>();
  return result;
}

}  // namespace

template <typename T>
ContinuousIcfForceManager<T>::ContinuousIcfForceManager(
    const MultibodyPlant<T>* plant)
    : plant_(plant) {
  DRAKE_THROW_UNLESS(plant_ != nullptr);
  DRAKE_THROW_UNLESS(plant_->is_finalized());
  DRAKE_THROW_UNLESS(!plant_->is_discrete());
  builder_ = std::make_unique<IcfBuilder<T>>(plant_);
}

template <typename T>
ContinuousIcfForceManager<T>::~ContinuousIcfForceManager() = default;

template <typename T>
void ContinuousIcfForceManager<T>::UpdateModelAndDataAtCurrentState(
    const Context<T>& context) const {
  // Build the ICF problem around the current state (q₀, v₀) at the fixed
  // reporting time step. Actuation/external forces are handled among the
  // non-contact forces (see CalcAppliedForces), so we pass null feedback gains
  // and the model carries no gain (actuation/external) constraints.
  builder_->UpdateModel(context, T(kReportingTimeStep),
                        /* actuation_feedback = */ nullptr,
                        /* external_feedback = */ nullptr, &model_);
  model_.ResizeData(&data_);

  // Evaluate the constraint force law at the current velocity v₀ (no solve).
  model_.CalcData(plant_->GetVelocities(context), &data_);
}

template <typename T>
void ContinuousIcfForceManager<T>::AddInIcfConstraintForces(
    MultibodyForces<T>* forces) const {
  const T dt(kReportingTimeStep);
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();
  VectorX<T>& tau = forces->mutable_generalized_forces();

  // Adds a per-body spatial impulse Γ (about the body origin, world frame),
  // converted to a force by /dt, into the body's MultibodyForces slot. Skips
  // anchored bodies (which carry no dynamics).
  auto add_body_spatial_impulse = [&](int icf_body,
                                      const Vector6<T>& Gamma_body_W) {
    if (model_.is_anchored(icf_body)) return;
    const MobodIndex mobod =
        plant_->get_body(BodyIndex(icf_body)).mobod_index();
    const Vector6<T> F_body_W = Gamma_body_W / dt;
    F_Bo_W_array[mobod] += SpatialForce<T>(F_body_W);
  };

  // --- Contact patches: Cartesian forces on bodies A and B. -----------------
  // The patch pool stores the spatial impulse Γ_Bo on body B; the reaction on
  // body A is -shift(Γ_Bo, p_AB). (Mirrors PatchConstraintsPool.)
  {
    const auto& patches = model_.patch_constraints_pool();
    const auto& patch_data = data_.patch_constraints_data();
    for (int p = 0; p < patches.num_patches(); ++p) {
      const int body_b = patches.bodies()[p].first;
      const int body_a = patches.bodies()[p].second;
      const Vector6<T>& Gamma_Bo_W = patch_data.Gamma_Bo_W_pool()[p];
      add_body_spatial_impulse(body_b, Gamma_Bo_W);
      if (!model_.is_anchored(body_a)) {
        const Vector6<T> Gamma_Ao_W =
            -ShiftSpatialForce(Gamma_Bo_W, patches.p_AB_W()[p]);
        add_body_spatial_impulse(body_a, Gamma_Ao_W);
      }
    }
  }

  // --- Weld constraints: Cartesian forces on bodies A and B. ----------------
  // For holonomic pools, body_pairs()[k] = (A, B). CalcSpatialImpulses returns
  // the impulses already resolved on each body (no extra negation).
  {
    const auto& welds = model_.weld_constraints_pool();
    const auto& weld_data = data_.weld_constraints_data();
    for (int k = 0; k < welds.num_constraints(); ++k) {
      const int body_a = welds.body_pairs()[k].first;
      const int body_b = welds.body_pairs()[k].second;
      Vector6<T> Gamma_Bo_W, Gamma_Ao_W;
      welds.CalcSpatialImpulses(k, weld_data.gamma(k), &Gamma_Bo_W,
                                &Gamma_Ao_W);
      add_body_spatial_impulse(body_b, Gamma_Bo_W);
      add_body_spatial_impulse(body_a, Gamma_Ao_W);
    }
  }

  // --- Ball constraints: Cartesian forces on bodies A and B. ----------------
  {
    const auto& balls = model_.ball_constraints_pool();
    const auto& ball_data = data_.ball_constraints_data();
    for (int k = 0; k < balls.num_constraints(); ++k) {
      const int body_a = balls.body_pairs()[k].first;
      const int body_b = balls.body_pairs()[k].second;
      Vector6<T> Gamma_Bo_W, Gamma_Ao_W;
      balls.CalcSpatialImpulses(k, ball_data.gamma(k), &Gamma_Bo_W,
                                &Gamma_Ao_W);
      add_body_spatial_impulse(body_b, Gamma_Bo_W);
      add_body_spatial_impulse(body_a, Gamma_Ao_W);
    }
  }

  // --- Joint-limit and coupler constraints: generalized forces. -------------
  // These constraints live in generalized (velocity-indexed) space. Each pool's
  // AccumulateGradient adds -Jᵀγ, so accumulating into a zero vector yields
  // -Σ Jᵀγ; the generalized constraint force is +Jᵀγ / dt = -g / dt.
  {
    VectorX<T> g = VectorX<T>::Zero(model_.num_velocities());
    model_.limit_constraints_pool().AccumulateGradient(data_, &g);
    model_.coupler_constraints_pool().AccumulateGradient(data_, &g);
    tau -= g / dt;
  }
}

template <typename T>
void ContinuousIcfForceManager<T>::CalcAppliedForces(
    const Context<T>& context, MultibodyForces<T>* forces) const {
  DRAKE_THROW_UNLESS(forces != nullptr);
  DRAKE_THROW_UNLESS(forces->CheckHasRightSizeForModel(*plant_));

  // Non-contact forces: gravity, force elements (incl. joint damping),
  // actuation, and other input-port forces. This resets `forces` first. ICF
  // gain constraints are intentionally not modeled (see
  // UpdateModelAndDataAtCurrentState), so actuation/external forces are
  // accounted for here and nowhere else.
  MultibodyPlantIcfAttorney<T>::CalcNonContactForcesContinuous(*plant_, context,
                                                               forces);

  // ICF contact and constraint forces evaluated at the current state.
  UpdateModelAndDataAtCurrentState(context);
  AddInIcfConstraintForces(forces);
}

}  // namespace internal

template <typename T>
void AddIcfContinuousForceReporting(MultibodyPlant<T>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  plant->SetContinuousContactForceReporter(
      std::make_unique<internal::ContinuousIcfForceManager<T>>(plant));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&AddIcfContinuousForceReporting<T>));

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::internal::ContinuousIcfForceManager);
