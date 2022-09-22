#include "drake/multibody/plant/deformable_driver.h"

#include <memory>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/systems/framework/context.h"

using drake::geometry::GeometryId;
using drake::geometry::internal::DeformableContact;
using drake::multibody::contact_solvers::internal::PartialPermutation;
using drake::multibody::fem::FemModel;
using drake::multibody::fem::FemState;
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
}

template <typename T>
DeformableDriver<T>::~DeformableDriver() = default;

template <typename T>
void DeformableDriver<T>::DeclareCacheEntries(
    DiscreteUpdateManager<T>* manager) {
  DRAKE_DEMAND(manager_ == manager);
  const auto& deformable_contact_cache_entry = manager->DeclareCacheEntry(
      "Deformable Contact Data.",
      systems::ValueProducer(this, DeformableContact<T>(),
                             &DeformableDriver<T>::CalcDeformableContact),
      {systems::System<T>::configuration_ticket()});
  cache_indexes_.deformable_contact =
      deformable_contact_cache_entry.cache_index();

  const auto& participating_velocity_mux_cache_entry =
      manager->DeclareCacheEntry(
          "Multiplexer for participating velocities.",
          systems::ValueProducer(
              this, Multiplexer(),
              &DeformableDriver<T>::CalcParticipatingVelocityMultiplexer),
          {deformable_contact_cache_entry.ticket()});
  cache_indexes_.participating_velocity_mux =
      participating_velocity_mux_cache_entry.cache_index();

  const auto& participating_velocities_cache_entry = manager->DeclareCacheEntry(
      fmt::format("Participating velocities for all bodies"),
      systems::ValueProducer(this, VectorX<T>(),
                             &DeformableDriver<T>::CalcParticipatingVelocities),
      {deformable_contact_cache_entry.ticket(),
       systems::System<T>::xd_ticket()});
  cache_indexes_.participating_velocities =
      participating_velocities_cache_entry.cache_index();

  for (DeformableBodyIndex i(0); i < deformable_model_->num_bodies(); ++i) {
    const DeformableBodyId id = deformable_model_->GetBodyId(i);
    const fem::FemModel<T>& fem_model = deformable_model_->GetFemModel(id);
    std::unique_ptr<fem::FemState<T>> model_state = fem_model.MakeFemState();
    const auto& fem_state_cache_entry = manager->DeclareCacheEntry(
        fmt::format("FEM state for body with index {}", i),
        systems::ValueProducer(
            *model_state,
            std::function<void(const Context<T>&, fem::FemState<T>*)>{
                [this, i](const Context<T>& context, fem::FemState<T>* state) {
                  this->CalcFemState(context, i, state);
                }}),
        {systems::System<T>::xd_ticket()});
    cache_indexes_.fem_states.emplace_back(fem_state_cache_entry.cache_index());

    const auto& dof_permutation_cache_entry = manager->DeclareCacheEntry(
        fmt::format("Partial permutation for dofs of body {} based on "
                    "participation in contact",
                    i),
        systems::ValueProducer(
            std::function<void(const Context<T>&, PartialPermutation*)>{
                [this, i](const Context<T>& context,
                          PartialPermutation* result) {
                  this->CalcDofPermutation(context, i, result);
                }}),
        {deformable_contact_cache_entry.ticket()});
    cache_indexes_.dof_permutations.emplace_back(
        dof_permutation_cache_entry.cache_index());
  }
}

template <typename T>
DeformableDriver<T>::Multiplexer::Multiplexer(std::vector<int> sizes)
    : sizes_(std::move(sizes)) {
  DRAKE_THROW_UNLESS(!sizes_.empty());
  DRAKE_THROW_UNLESS(sizes_[0] >= 0);
  offsets_.resize(sizes_.size());
  offsets_[0] = 0;
  for (int i = 1; i < num_channels(); ++i) {
    DRAKE_THROW_UNLESS(sizes_[i] >= 0);
    offsets_[i] = offsets_[i - 1] + sizes_[i - 1];
  }
  num_entries_ = std::accumulate(sizes_.begin(), sizes_.end(), 0);
}

template <typename T>
VectorX<T> DeformableDriver<T>::Multiplexer::Multiplex(
    std::vector<VectorX<T>>&& inputs) const {
  VectorX<T> result(num_entries_);
  DRAKE_THROW_UNLESS(static_cast<int>(inputs.size()) == num_channels());
  for (int i = 0; i < num_channels(); ++i) {
    DRAKE_THROW_UNLESS(sizes_[i] == inputs[i].size());
    result.segment(offsets_[i], sizes_[i]) = std::move(inputs[i]);
  }
  return result;
}

template <typename T>
VectorX<T> DeformableDriver<T>::Multiplexer::Demultiplex(
    const VectorX<T>& input, int index) const {
  DRAKE_THROW_UNLESS(0 <= index && index < num_channels());
  DRAKE_THROW_UNLESS(input.size() == num_entries_);
  const VectorX<T> result = input.segment(offsets_[index], sizes_[index]);
  return result;
}

template <typename T>
void DeformableDriver<T>::CalcFemState(const Context<T>& context,
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
    const Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.fem_states.at(index))
      .template Eval<FemState<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcDeformableContact(
    const Context<T>& context, DeformableContact<T>* result) const {
  const geometry::QueryObject<T>& query_object =
      manager_->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  query_object.ComputeDeformableContact(result);
}

template <typename T>
const DeformableContact<T>& DeformableDriver<T>::EvalDeformableContact(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.deformable_contact)
      .template Eval<DeformableContact<T>>(context);
}

template <typename T>
void DeformableDriver<T>::CalcDofPermutation(const Context<T>& context,
                                             DeformableBodyIndex index,
                                             PartialPermutation* result) const {
  const DeformableBodyId body_id = deformable_model_->GetBodyId(index);
  const GeometryId geometry_id = deformable_model_->GetGeometryId(body_id);
  *result = EvalDeformableContact(context)
                .contact_participation(geometry_id)
                .CalcDofPartialPermutation();
}

template <typename T>
const PartialPermutation& DeformableDriver<T>::EvalDofPermutation(
    const Context<T>& context, DeformableBodyIndex index) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.dof_permutations.at(index))
      .template Eval<PartialPermutation>(context);
}

template <typename T>
void DeformableDriver<T>::CalcParticipatingVelocityMultiplexer(
    const Context<T>& context,
    typename DeformableDriver<T>::Multiplexer* result) const {
  const int num_bodies = deformable_model_->num_bodies();
  std::vector<int> num_participating_dofs(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    num_participating_dofs[i] =
        EvalDofPermutation(context, i).permuted_domain_size();
  }
  *result = Multiplexer(std::move(num_participating_dofs));
}

template <typename T>
const typename DeformableDriver<T>::Multiplexer&
DeformableDriver<T>::EvalParticipatingVelocityMultiplexer(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.participating_velocity_mux)
      .template Eval<DeformableDriver<T>::Multiplexer>(context);
}

template <typename T>
void DeformableDriver<T>::CalcParticipatingVelocities(
    const Context<T>& context, VectorX<T>* result) const {
  const int num_bodies = deformable_model_->num_bodies();
  std::vector<VectorX<T>> participating_velocities(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    const PartialPermutation& permutation = EvalDofPermutation(context, i);
    const VectorX<T>& v = EvalFemState(context, i).GetVelocities();
    participating_velocities[i].resize(permutation.permuted_domain_size());
    permutation.Apply(v, &participating_velocities[i]);
  }
  *result = EvalParticipatingVelocityMultiplexer(context).Multiplex(
      std::move(participating_velocities));
}

template <typename T>
const VectorX<T>& DeformableDriver<T>::EvalParticipatingVelocities(
    const Context<T>& context) const {
  return manager_->plant()
      .get_cache_entry(cache_indexes_.participating_velocities)
      .template Eval<VectorX<T>>(context);
}

template class DeformableDriver<double>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
