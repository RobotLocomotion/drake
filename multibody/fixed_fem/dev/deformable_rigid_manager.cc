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
      SetFemSolvers(*deformable_model);
      extracted_deformable_model = true;
    }
  }
  if (!extracted_deformable_model) {
    throw std::logic_error(
        "The owning MultibodyPlant does not have any deformable model.");
  }
}

template <typename T>
void DeformableRigidManager<T>::SetFemSolvers(
    const DeformableModel<T>& deformable_model) {
  discrete_state_indexes_ = deformable_model.discrete_state_indexes();
  for (int i = 0; i < deformable_model.num_bodies(); ++i) {
    const FemModelBase<T>& fem_model =
        deformable_model.fem_model(SoftBodyIndex(i));
    /* Pre-allocate FEM state to avoid repeated allocation in inner loops. */
    state0s_.emplace_back(fem_model.MakeFemStateBase());
    state_stars_.emplace_back(fem_model.MakeFemStateBase());
    fem_solvers_.emplace_back(std::make_unique<FemSolver<T>>(&fem_model));
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
  /* Calculates the deformable contact-free states. */
  const systems::DiscreteValues<T>& all_discrete_states =
      context.get_discrete_state();
  for (int i = 0; i < static_cast<int>(discrete_state_indexes_.size()); ++i) {
    /* Extract q, qdot and qddot from context. */
    const systems::BasicVector<T>& discrete_state =
        all_discrete_states.get_vector(discrete_state_indexes_[i]);
    const auto& discrete_value = discrete_state.get_value();
    const int num_dofs = discrete_value.size() / 3;
    const auto& q = discrete_value.head(num_dofs);
    const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
    const auto& qddot = discrete_value.tail(num_dofs);
    /* Set up FemState and advance to the next time step. */
    FemStateBase<T>& state0 = *state0s_[i];
    FemStateBase<T>& state_star = *state_stars_[i];
    state0.SetQ(q);
    state0.SetQdot(qdot);
    state0.SetQddot(qddot);
    // TODO(xuchenhan-tri): FemState needs a SetFrom() method. Setting
    //  DiscreteValues from FemStateBase (and vice-versa) should also be made
    //  more compact.
    state_star.SetQ(q);
    state_star.SetQdot(qdot);
    state_star.SetQddot(qddot);
    /* Obtain the contact-free state for deformable body i. */
    fem_solvers_[i]->AdvanceOneTimeStep(state0, &state_star);
    // TODO(xuchenhan-tri): This assumes no contact exists. Modify this to
    //  include the effect of contact. */
    /* Copy new state to output variable. */
    systems::BasicVector<T>& next_discrete_state =
        updates->get_mutable_vector(discrete_state_indexes_[i]);
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
