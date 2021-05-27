#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

template <typename T>
void DeformableRigidManager<T>::RegisterCollisionObjects(
    const geometry::SceneGraph<T>& scene_graph) {
  const geometry::SceneGraphInspector<T>& inspector =
      scene_graph.model_inspector();
  /* Make sure that the owning plant is registered at the given scene graph.
   */
  DRAKE_THROW_UNLESS(
      inspector.SourceIsRegistered(this->plant().get_source_id().value()));
  for (const auto& per_body_collision_geometries :
       this->collision_geometries()) {
    for (geometry::GeometryId id : per_body_collision_geometries) {
      /* Sanity check that the geometry comes from the owning MultibodyPlant
       indeed. */
      DRAKE_DEMAND(
          inspector.BelongsToSource(id, this->plant().get_source_id().value()));
      std::unique_ptr<geometry::GeometryInstance> geometry_instance =
          inspector.CloneGeometryInstance(id);
      const geometry::ProximityProperties* props =
          geometry_instance->proximity_properties();
      /* Collision geometry must have proximity properties attached to it. */
      DRAKE_DEMAND(props != nullptr);
      collision_objects_.AddCollisionObject(id, geometry_instance->shape(),
                                            *props);
    }
  }
}

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
      RegisterDeformableGeometries();
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
void DeformableRigidManager<T>::RegisterDeformableGeometries() {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  deformable_meshes_ = deformable_model_->reference_configuration_meshes();
  deformable_proximity_properties_ = deformable_model_->proximity_properties();
  vertex_positions_output_port_ =
      &deformable_model_->get_vertex_positions_output_port();
}

template <typename T>
void DeformableRigidManager<T>::DeclareCacheEntries(MultibodyPlant<T>* plant) {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  for (SoftBodyIndex deformable_body_id(0);
       deformable_body_id < deformable_model_->num_bodies();
       ++deformable_body_id) {
    const FemModelBase<T>& fem_model =
        deformable_model_->fem_model(deformable_body_id);
    auto allocate_fem_state_base = [&]() {
      return AbstractValue::Make(*fem_model.MakeFemStateBase());
    };
    /* Lambda function to extract the q, qdot, and qddot from context and copy
     them to the cached fem state. */
    auto copy_to_fem_state = [this, deformable_body_id](
                                 const systems::ContextBase& context_base,
                                 AbstractValue* cache_value) {
      const auto& context =
          dynamic_cast<const systems::Context<T>&>(context_base);
      const systems::DiscreteValues<T>& all_discrete_states =
          context.get_discrete_state();
      /* Extract q, qdot and qddot from context. */
      const systems::BasicVector<T>& discrete_state =
          all_discrete_states.get_vector(
              deformable_model_->discrete_state_indexes()[deformable_body_id]);
      const auto& discrete_value = discrete_state.get_value();
      DRAKE_DEMAND(discrete_value.size() % 3 == 0);
      const int num_dofs = discrete_value.size() / 3;
      const auto& q = discrete_value.head(num_dofs);
      const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
      const auto& qddot = discrete_value.tail(num_dofs);
      auto& fem_state =
          cache_value->template get_mutable_value<FemStateBase<T>>();
      fem_state.SetQ(q);
      fem_state.SetQdot(qdot);
      fem_state.SetQddot(qddot);
    };
    const auto& fem_state_cache_entry = this->DeclareCacheEntry(
        plant, "FEM state", allocate_fem_state_base,
        std::move(copy_to_fem_state), {systems::System<T>::xd_ticket()});
    fem_state_cache_indexes_.emplace_back(fem_state_cache_entry.cache_index());

    /* Lambda function to calculate the free-motion velocity for the deformable
     body. */
    auto calc_fem_state_star = [this, deformable_body_id](
                                   const systems::ContextBase& context_base,
                                   AbstractValue* cache_value) {
      const auto& context =
          dynamic_cast<const systems::Context<T>&>(context_base);
      const FemStateBase<T>& fem_state =
          EvalFemStateBase(context, deformable_body_id);
      auto& fem_state_star =
          cache_value->template get_mutable_value<FemStateBase<T>>();
      // TODO(xuchenhan-tri): FemState needs a SetFrom() method.
      fem_state_star.SetQ(fem_state.q());
      fem_state_star.SetQdot(fem_state.qdot());
      fem_state_star.SetQddot(fem_state.qddot());
      /* Obtain the contact-free state for the deformable body. */
      fem_solvers_[deformable_body_id]->AdvanceOneTimeStep(fem_state,
                                                           &fem_state_star);
    };
    /* Declares the free-motion cache entry which only depends on the fem state.
     */
    free_motion_cache_indexes_.emplace_back(
        this->DeclareCacheEntry(plant, "Free motion FEM state",
                                std::move(allocate_fem_state_base),
                                std::move(calc_fem_state_star),
                                {fem_state_cache_entry.ticket()})
            .cache_index());
  }
}

// TODO(xuchenhan-tri): Implement the discrete update for rigid dofs.
template <typename T>
void DeformableRigidManager<T>::DoCalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  /* Only deformable dofs are supported at the moment. */
  auto x =
      context.get_discrete_state(this->multibody_state_index()).get_value();
  DRAKE_DEMAND(x.size() == 0);
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes =
      deformable_model_->discrete_state_indexes();
  /* Evaluates the deformable free-motion states. */
  for (SoftBodyIndex deformable_body_id(0);
       deformable_body_id < free_motion_cache_indexes_.size();
       ++deformable_body_id) {
    const FemStateBase<T>& state_star =
        EvalFreeMotionFemStateBase(context, deformable_body_id);
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

template <typename T>
void DeformableRigidManager<T>::UpdateCollisionObjectPoses(
    const systems::Context<T>& context) const {
  const geometry::QueryObject<T>& query_object =
      this->plant()
          .get_geometry_query_input_port()
          .template Eval<geometry::QueryObject<T>>(context);
  const std::vector<geometry::GeometryId>& geometry_ids =
      collision_objects_.geometry_ids();
  for (const geometry::GeometryId id : geometry_ids) {
    const math::RigidTransform<T>& pose = query_object.GetPoseInWorld(id);
    collision_objects_.set_pose_in_world(id, pose);
  }
}

template <typename T>
void DeformableRigidManager<T>::UpdateDeformableVertexPositions(
    const systems::Context<T>& context) const {
  const std::vector<VectorX<T>>& vertex_positions =
      vertex_positions_output_port_->template Eval<std::vector<VectorX<T>>>(
          context);
  DRAKE_DEMAND(vertex_positions.size() == deformable_meshes_.size());
  for (int i = 0; i < static_cast<int>(vertex_positions.size()); ++i) {
    const VectorX<T>& q = vertex_positions[i];
    const auto p_WVs = Eigen::Map<const Matrix3X<T>>(q.data(), 3, q.size() / 3);
    std::vector<geometry::VolumeElement> tets =
        deformable_meshes_[i].tetrahedra();
    // TODO(xuchenhan-tri): We assume the deformable body frame is always the
    //  same as the world frame for now. It probably makes sense to have the
    //  frame move with the body in the future.
    std::vector<geometry::VolumeVertex<T>> vertices_D;
    for (int j = 0; j < p_WVs.cols(); ++j) {
      vertices_D.push_back(geometry::VolumeVertex<T>(p_WVs.col(j)));
    }
    deformable_meshes_[i] = {std::move(tets), std::move(vertices_D)};
  }
}
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager);
