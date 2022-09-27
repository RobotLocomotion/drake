#include "drake/multibody/plant/deformable_driver.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/fem/fem_solver.h"
#include "drake/multibody/fem/velocity_newmark_scheme.h"
#include "drake/systems/framework/context.h"

using drake::multibody::fem::FemModel;
using drake::multibody::fem::FemState;
using drake::multibody::fem::internal::FemSolver;
using drake::multibody::fem::internal::FemSolverScratchData;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DeformableDriver<T>::DeformableDriver(
    const DeformableModel<T>* deformable_model,
    const DiscreteUpdateManager<T>* manager)
    : deformable_model_(deformable_model), manager_(manager) {
  DRAKE_DEMAND(deformable_model != nullptr);
  DRAKE_DEMAND(manager != nullptr);
  // TODO(xuchenhan-tri): Expose the integrator as a config.
  /* Set the time integrator for advancing deformable states in time to be the
   midpoint rule, i.e., q = q₀ + δt/2 *(v₀ + v). */
  integrator_ = std::make_unique<fem::internal::VelocityNewmarkScheme<T>>(
      manager_->plant().time_step(), 1.0, 0.5);
}

template <typename T>
DeformableDriver<T>::~DeformableDriver() = default;

template <typename T>
void DeformableDriver<T>::DeclareCacheEntries(
    DiscreteUpdateManager<T>* manager) {
  DRAKE_DEMAND(manager_ == manager);
  for (DeformableBodyIndex i(0); i < deformable_model_->num_bodies(); ++i) {
    const DeformableBodyId id = deformable_model_->GetBodyId(i);
    const fem::FemModel<T>& fem_model = deformable_model_->GetFemModel(id);
    std::unique_ptr<fem::FemState<T>> model_state = fem_model.MakeFemState();
    /* Cache entry for current FEM state. */
    const auto& fem_state_cache_entry = manager->DeclareCacheEntry(
        fmt::format("FEM state for body with index {}", i),
        systems::ValueProducer(
            *model_state,
            std::function<void(const systems::Context<T>&, fem::FemState<T>*)>{
                [this, i](const systems::Context<T>& context,
                          fem::FemState<T>* state) {
                  this->CalcFemState(context, i, state);
                }}),
        {systems::System<T>::xd_ticket()});
    cache_indexes_.fem_states.emplace_back(fem_state_cache_entry.cache_index());

    /* Cache entry for free motion FEM state. */
    const auto& free_motion_fem_state_cache_entry = manager->DeclareCacheEntry(
        fmt::format("Free motion FEM state for body with index {}", i),
        systems::ValueProducer(
            *model_state,
            std::function<void(const systems::Context<T>&, fem::FemState<T>*)>{
                [this, i](const systems::Context<T>& context,
                          fem::FemState<T>* free_motion_state) {
                  this->CalcFreeMotionFemState(context, i, free_motion_state);
                }}),
        {fem_state_cache_entry.ticket()});
    cache_indexes_.free_motion_fem_states.emplace_back(
        free_motion_fem_state_cache_entry.cache_index());

    /* Cache entry for FEM state at next time step. */
    const auto& next_fem_state_cache_entry = manager->DeclareCacheEntry(
        fmt::format("FEM state for body with index {} at next time step", i),
        systems::ValueProducer(
            *model_state,
            std::function<void(const systems::Context<T>&, fem::FemState<T>*)>{
                [this, i](const systems::Context<T>& context,
                          fem::FemState<T>* next_fem_state) {
                  this->CalcNextFemState(context, i, next_fem_state);
                }}),
        {systems::SystemBase::all_sources_ticket()});
    cache_indexes_.next_fem_states.emplace_back(
        next_fem_state_cache_entry.cache_index());

    /* Scatch data for FEM solver. */
    FemSolverScratchData scratch(fem_model);
    const auto& scratch_entry = manager->DeclareCacheEntry(
        fmt::format("FEM solver scratch workspace for body with index {}", i),
        systems::ValueProducer(scratch, &systems::ValueProducer::NoopCalc),
        {systems::SystemBase::nothing_ticket()});
    cache_indexes_.fem_solver_scratches.emplace_back(
        scratch_entry.cache_index());
  }
}

template <typename T>
void DeformableDriver<T>::CalcFemState(const systems::Context<T>& context,
                                       DeformableBodyIndex index,
                                       FemState<T>* fem_state) const {
  const DeformableBodyId id = deformable_model_->GetBodyId(index);
  const systems::BasicVector<T>& discrete_state =
      context.get_discrete_state().get_vector(
          deformable_model_->GetDiscreteStateIndex(id));
  const VectorX<T>& discrete_value = discrete_state.value();
  DRAKE_DEMAND(discrete_value.size() % 3 == 0);
  const int num_dofs = discrete_value.size() / 3;
  const auto& q = discrete_value.head(num_dofs);
  const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
  const auto& qddot = discrete_value.tail(num_dofs);
  fem_state->SetPositions(q);
  fem_state->SetVelocities(qdot);
  fem_state->SetAccelerations(qddot);
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalFemState(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.fem_states.at(index))
      .template Eval<FemState<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcFreeMotionFemState(
    const systems::Context<T>& context, DeformableBodyIndex index,
    FemState<T>* fem_state_star) const {
  const FemState<T>& fem_state = EvalFemState(context, index);
  const DeformableBodyId id = deformable_model_->GetBodyId(index);
  const FemModel<T>& model = deformable_model_->GetFemModel(id);
  // TODO(xuchenhan-tri): We should expose an API to set the solver tolerance
  // here.
  const FemSolver<T> solver(&model, integrator_.get());
  FemSolverScratchData<T>& scratch =
      manager_->plant()
          .get_cache_entry(cache_indexes_.fem_solver_scratches.at(index))
          .get_mutable_cache_entry_value(context)
          .template GetMutableValueOrThrow<FemSolverScratchData<T>>();
  solver.AdvanceOneTimeStep(fem_state, fem_state_star, &scratch);
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalFreeMotionFemState(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.free_motion_fem_states.at(index))
      .template Eval<FemState<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcNextFemState(const systems::Context<T>& context,
                                           DeformableBodyIndex index,
                                           FemState<T>* next_fem_state) const {
  // TODO(xuchenhan-tri): Update this implementation to include the effect of
  // contact and constraints.
  const FemState<T>& free_motion_state = EvalFreeMotionFemState(context, index);
  next_fem_state->SetPositions(free_motion_state.GetPositions());
  next_fem_state->SetVelocities(free_motion_state.GetVelocities());
  next_fem_state->SetAccelerations(free_motion_state.GetAccelerations());
}

template <typename T>
const FemState<T>& DeformableDriver<T>::EvalNextFemState(
    const systems::Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.next_fem_states.at(index))
      .template Eval<FemState<T>>(context);
}

template class DeformableDriver<double>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
