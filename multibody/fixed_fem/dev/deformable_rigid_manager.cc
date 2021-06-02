#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
template <typename T>
void DeformableRigidManager<T>::ExtractModelInfo() {
  bool extracted_deformable_model = false;
  const std::vector<std::unique_ptr<multibody::internal::PhysicalModel<T>>>&
      physical_models = this->plant().physical_models();
  for (const auto& model : physical_models) {
    const auto* deformable_model =
        dynamic_cast<const DeformableModel<T>*>(model.get());
    if (deformable_model != nullptr) {
      if (extracted_deformable_model) {
        throw std::logic_error(
            "More than one DeformableModel are specified in the "
            "MultibodyPlant.");
      }
      deformable_model_ = deformable_model;
      MakeFemSolvers();
      extracted_deformable_model = true;
    }
  }
  if (!extracted_deformable_model) {
    throw std::logic_error(
        "The owning MultibodyPlant does not have any deformable model.");
  }
}

template <typename T>
void DeformableRigidManager<T>::MakeFemSolvers() {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  for (int i = 0; i < deformable_model_->num_bodies(); ++i) {
    const FemModelBase<T>& fem_model =
        deformable_model_->fem_model(SoftBodyIndex(i));
    fem_solvers_.emplace_back(std::make_unique<FemSolver<T>>(&fem_model));
  }
}

template <typename T>
void DeformableRigidManager<T>::DeclareCacheEntries(MultibodyPlant<T>* plant) {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  for (int i = 0; i < deformable_model_->num_bodies(); ++i) {
    const FemModelBase<T>& fem_model =
        deformable_model_->fem_model(SoftBodyIndex(i));
    auto allocate_fem_state_base = [&]() {
      return AbstractValue::Make(*fem_model.MakeFemStateBase());
    };
    /* Lambda function to extract the q, qdot, and qddot from context and set
     them to the cached state0. */
    auto calc_state0 = [this, i](const systems::ContextBase& context_base,
                                 AbstractValue* cache_value) {
      const auto& context =
          dynamic_cast<const systems::Context<T>&>(context_base);
      const systems::DiscreteValues<T>& all_discrete_states =
          context.get_discrete_state();
      /* Extract q, qdot and qddot from context. */
      const systems::BasicVector<T>& discrete_state =
          all_discrete_states.get_vector(
              deformable_model_->discrete_state_indexes()[i]);
      const auto& discrete_value = discrete_state.get_value();
      const int num_dofs = discrete_value.size() / 3;
      const auto& q = discrete_value.head(num_dofs);
      const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
      const auto& qddot = discrete_value.tail(num_dofs);
      auto& state0 = cache_value->template get_mutable_value<FemStateBase<T>>();
      state0.SetQ(q);
      state0.SetQdot(qdot);
      state0.SetQddot(qddot);
    };
    const auto& state0_cache_entry = this->DeclareCacheEntry(
        plant, "state0", allocate_fem_state_base, std::move(calc_state0),
        {systems::System<T>::xd_ticket()});
    state0s_.emplace_back(state0_cache_entry.cache_index());

    /* Lambda function to calculate the free-motion velocity for the i-th
     deformable body. */
    auto calc_state_star = [this, i](const systems::ContextBase& context_base,
                                     AbstractValue* cache_value) {
      const auto& context =
          dynamic_cast<const systems::Context<T>&>(context_base);
      const auto& state0 = this->plant()
                               .get_cache_entry(state0s_[i])
                               .template Eval<FemStateBase<T>>(context);
      auto& state_star =
          cache_value->template get_mutable_value<FemStateBase<T>>();
      // TODO(xuchenhan-tri): FemState needs a SetFrom() method.
      state_star.SetQ(state0.q());
      state_star.SetQdot(state0.qdot());
      state_star.SetQddot(state0.qddot());
      /* Obtain the contact-free state for deformable body i. */
      fem_solvers_[i]->AdvanceOneTimeStep(state0, &state_star);
    };
    /* Declares the free-motion cache entry `state_star` which only depends on
     the state at the previous time step, `state0`. */
    state_stars_.emplace_back(
        this->DeclareCacheEntry(
                plant, "state0", std::move(allocate_fem_state_base),
                std::move(calc_state_star), {state0_cache_entry.ticket()})
            .cache_index());
  }
}

// TODO(xuchenhan-tri): Implement the discrete update for rigid dofs.
template <typename T>
void DeformableRigidManager<T>::DoCalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  /* Only deformable dofs are supported at the moment. */
  auto x0 =
      context.get_discrete_state(this->multibody_state_index()).get_value();
  DRAKE_DEMAND(x0.size() == 0);
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes =
      deformable_model_->discrete_state_indexes();
  /* Evaluates the deformable free-motion states. */
  for (int deformable_body_id = 0;
       deformable_body_id < static_cast<int>(state_stars_.size());
       ++deformable_body_id) {
    const auto& state_star =
        this->plant()
            .get_cache_entry(state_stars_[deformable_body_id])
            .template Eval<FemStateBase<T>>(context);
    const int num_dofs = state_star.num_generalized_positions();
    // TODO(xuchenhan-tri): This assumes no contact exists. Modify this to
    //  include the effect of contact.
    /* Copy new state to output variable. */
    systems::BasicVector<T>& next_discrete_state =
        updates->get_mutable_vector(discrete_state_indexes[deformable_body_id]);
    Eigen::VectorBlock<VectorX<T>> next_discrete_value =
        next_discrete_state.get_mutable_value();
    next_discrete_value.head(num_dofs) = state_star.q();
    next_discrete_value.segment(num_dofs, num_dofs) = state_star.qdot();
    next_discrete_value.tail(num_dofs) = state_star.qddot();
  }
}

}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager);
