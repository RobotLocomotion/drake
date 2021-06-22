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
      const geometry::Shape& shape = inspector.GetShape(id);
      const geometry::ProximityProperties* props =
          dynamic_cast<const geometry::ProximityProperties*>(
              inspector.GetProperties(id, geometry::Role::kProximity));
      /* Collision geometry must have proximity properties attached to it. */
      DRAKE_DEMAND(props != nullptr);
      collision_objects_.AddCollisionObject(id, shape, *props);
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
    const auto& fem_state_cache_entry = plant->DeclareCacheEntry(
        "FEM state", allocate_fem_state_base, std::move(copy_to_fem_state),
        {systems::System<T>::xd_ticket()});
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
        plant
            ->DeclareCacheEntry("Free motion FEM state",
                                std::move(allocate_fem_state_base),
                                std::move(calc_fem_state_star),
                                {fem_state_cache_entry.ticket()})
            .cache_index());
  }
}

template <typename T>
void DeformableRigidManager<T>::DoCalcContactSolverResults(
    const systems::Context<T>& context,
    contact_solvers::internal::ContactSolverResults<T>* results) const {
  /* Assert this method was called on a context storing discrete state. */
  this->plant().ValidateContext(context);
  DRAKE_ASSERT(context.num_continuous_states() == 0);
  const int nv = this->plant().num_velocities();

  // TODO(xuchenhan-tri): This is not true when there are deformable dofs.
  //  Modify it when deformable-rigid contact is supported.
  /* Quick exit if there are no moving objects. */
  if (nv == 0) return;

  // TODO(xuchenhan-tri): Incorporate deformable-rigid contact pairs.
  /* Compute all rigid-rigid contact pairs, including both penetration pairs
   and quadrature pairs for discrete hydroelastic. */
  const std::vector<multibody::internal::DiscreteContactPair<T>>&
      rigid_contact_pairs = this->EvalDiscreteContactPairs(context);
  const int num_rigid_contacts = rigid_contact_pairs.size();

  /* Extract all information needed by the contact/TAMSI solver. */
  multibody::internal::ContactJacobians<T> rigid_contact_jacobians;
  VectorX<T> v(nv);
  MatrixX<T> M(nv, nv);
  VectorX<T> minus_tau(nv);
  VectorX<T> mu(num_rigid_contacts);
  VectorX<T> phi(num_rigid_contacts);
  VectorX<T> fn(num_rigid_contacts);
  VectorX<T> stiffness(num_rigid_contacts);
  VectorX<T> damping(num_rigid_contacts);
  CalcContactQuantities(context, rigid_contact_pairs, &rigid_contact_jacobians,
                        &v, &M, &minus_tau, &mu, &phi, &fn, &stiffness,
                        &damping);

  /* Call the contact solver if one exists. Otherwise, invoke the TAMSI
   solver. */
  if (contact_solver_ != nullptr) {
    this->CallContactSolver(contact_solver_.get(), context.get_time(), v, M,
                            minus_tau, phi, rigid_contact_jacobians.Jc,
                            stiffness, damping, mu, results);
  } else {
    this->CallTamsiSolver(
        context.get_time(), v, M, minus_tau, fn, rigid_contact_jacobians.Jn,
        rigid_contact_jacobians.Jt, stiffness, damping, mu, results);
  }
}

template <typename T>
void DeformableRigidManager<T>::CalcContactQuantities(
    const systems::Context<T>& context,
    const std::vector<multibody::internal::DiscreteContactPair<T>>&
        rigid_contact_pairs,
    multibody::internal::ContactJacobians<T>* rigid_contact_jacobians,
    EigenPtr<VectorX<T>> v, EigenPtr<MatrixX<T>> M,
    EigenPtr<VectorX<T>> minus_tau, EigenPtr<VectorX<T>> mu,
    EigenPtr<VectorX<T>> phi, EigenPtr<VectorX<T>> fn,
    EigenPtr<VectorX<T>> stiffness, EigenPtr<VectorX<T>> damping) const {
  DRAKE_DEMAND(v != nullptr);
  DRAKE_DEMAND(M != nullptr);
  DRAKE_DEMAND(minus_tau != nullptr);
  DRAKE_DEMAND(mu != nullptr);
  DRAKE_DEMAND(phi != nullptr);
  DRAKE_DEMAND(fn != nullptr);
  DRAKE_DEMAND(stiffness != nullptr);
  DRAKE_DEMAND(damping != nullptr);
  /* Compute the contact Jacobians for the rigid dofs. */
  *rigid_contact_jacobians = this->EvalContactJacobians(context);

  /* Compute the generalized velocities. */
  auto x =
      context.get_discrete_state(this->multibody_state_index()).get_value();
  const int nv = this->plant().num_velocities();
  DRAKE_DEMAND(v->size() == nv);
  DRAKE_DEMAND(M->rows() == nv);
  DRAKE_DEMAND(M->cols() == nv);
  DRAKE_DEMAND(minus_tau->size() == nv);
  *v = x.bottomRows(nv);

  /* Compute the mass matrix. */
  this->plant().CalcMassMatrix(context, M);

  /* Computes the negative generalized non-contact forces on the rigid dofs,
   `minus_tau`. */
  MultibodyForces<T> forces(this->internal_tree());
  this->CalcNonContactForces(context, true /* discrete */, &forces);
  /* Workspace for inverse dynamics: Bodies' accelerations, ordered by
   BodyNodeIndex. */
  std::vector<SpatialAcceleration<T>> A_WB_array(this->plant().num_bodies());
  /* Generalized accelerations. */
  const VectorX<T> vdot = VectorX<T>::Zero(nv);
  /* Body forces (alias to forces). */
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  /* With vdot = 0, this computes:
     -tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W. */
  *minus_tau = forces.mutable_generalized_forces();
  this->internal_tree().CalcInverseDynamics(
      context, vdot, F_BBo_W_array, *minus_tau, &A_WB_array,
      &F_BBo_W_array, /* Note: these arrays get overwritten on output. */
      minus_tau);

  /* Computes friction coefficient. Static friction is ignored by the time
   stepping scheme. */
  const int num_contacts = rigid_contact_pairs.size();
  DRAKE_DEMAND(mu->size() == num_contacts);
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      this->CalcCombinedFrictionCoefficients(context, rigid_contact_pairs);
  std::transform(combined_friction_pairs.begin(), combined_friction_pairs.end(),
                 mu->data(),
                 [](const CoulombFriction<double>& coulomb_friction) {
                   return coulomb_friction.dynamic_friction();
                 });

  /* Compute penetration, normal contact force, stiffness, and damping. */
  DRAKE_DEMAND(phi->size() == num_contacts);
  DRAKE_DEMAND(fn->size() == num_contacts);
  DRAKE_DEMAND(stiffness->size() == num_contacts);
  DRAKE_DEMAND(damping->size() == num_contacts);
  for (int i = 0; i < num_contacts; ++i) {
    (*phi)[i] = rigid_contact_pairs[i].phi0;
    (*fn)[i] = rigid_contact_pairs[i].fn0;
    (*stiffness)[i] = rigid_contact_pairs[i].stiffness;
    (*damping)[i] = rigid_contact_pairs[i].damping;
  }
}

template <typename T>
void DeformableRigidManager<T>::DoCalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  /* Get the rigid dofs from context. */
  auto x =
      context.get_discrete_state(this->multibody_state_index()).get_value();
  const auto& q = x.topRows(this->plant().num_positions());

  const contact_solvers::internal::ContactSolverResults<T>& solver_results =
      this->EvalContactSolverResults(context);
  const auto& v_next = solver_results.v_next;

  VectorX<T> qdot_next(this->plant().num_positions());
  this->plant().MapVelocityToQDot(context, v_next, &qdot_next);
  const double dt = this->plant().time_step();
  const auto& q_next = q + dt * qdot_next;

  VectorX<T> x_next(this->plant().num_multibody_states());
  x_next << q_next, v_next;
  updates->get_mutable_vector(this->multibody_state_index())
      .SetFromVector(x_next);

  /* Evaluates the deformable free-motion states. */
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes =
      deformable_model_->discrete_state_indexes();
  for (SoftBodyIndex deformable_body_id(0);
       deformable_body_id < free_motion_cache_indexes_.size();
       ++deformable_body_id) {
    const FemStateBase<T>& state_star =
        EvalFreeMotionFemStateBase(context, deformable_body_id);
    const int num_dofs = state_star.num_generalized_positions();
    // TODO(xuchenhan-tri): This assumes no deformable-rigid contact exists.
    //  Modify this to include the effect of contact.
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
    const math::RigidTransform<T>& X_WG = query_object.GetPoseInWorld(id);
    collision_objects_.set_pose_in_world(id, X_WG);
  }
}

template <typename T>
void DeformableRigidManager<T>::UpdateDeformableVertexPositions(
    const systems::Context<T>& context) const {
  const std::vector<VectorX<T>>& vertex_positions =
      deformable_model_->get_vertex_positions_output_port()
          .template Eval<std::vector<VectorX<T>>>(context);
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

template <typename T>
internal::DeformableRigidContactPair<T>
DeformableRigidManager<T>::CalcDeformableRigidContactPair(
    geometry::GeometryId rigid_id, SoftBodyIndex deformable_id) const {
  DeformableContactSurface<T> contact_surface = ComputeTetMeshTriMeshContact(
      deformable_meshes_[deformable_id], collision_objects_.mesh(rigid_id),
      collision_objects_.pose_in_world(rigid_id));

  const auto get_point_contact_parameters =
      [this](const geometry::ProximityProperties& props) -> std::pair<T, T> {
    return std::make_pair(props.template GetPropertyOrDefault<T>(
                              geometry::internal::kMaterialGroup,
                              geometry::internal::kPointStiffness,
                              this->default_contact_stiffness()),
                          props.template GetPropertyOrDefault<T>(
                              geometry::internal::kMaterialGroup,
                              geometry::internal::kHcDissipation,
                              this->default_contact_dissipation()));
  };
  /* Extract the stiffness, dissipation and friction parameters of the
   deformable body. */
  DRAKE_DEMAND(deformable_model_ != nullptr);
  const geometry::ProximityProperties& deformable_props =
      deformable_model_->proximity_properties()[deformable_id];
  const auto [deformable_stiffness, deformable_dissipation] =
      get_point_contact_parameters(deformable_props);
  const CoulombFriction<T> deformable_mu =
      deformable_props.GetProperty<CoulombFriction<T>>(
          geometry::internal::kMaterialGroup, geometry::internal::kFriction);

  /* Extract the stiffness, dissipation and friction parameters of the rigid
   body. */
  const geometry::ProximityProperties& rigid_proximity_properties =
      collision_objects_.proximity_properties(rigid_id);
  const auto [rigid_stiffness, rigid_dissipation] =
      get_point_contact_parameters(rigid_proximity_properties);
  const CoulombFriction<T> rigid_mu =
      rigid_proximity_properties.GetProperty<CoulombFriction<T>>(
          geometry::internal::kMaterialGroup, geometry::internal::kFriction);

  /* Combine the stiffness, dissipation and friction parameters for the
   contact points. */
  auto [k, d] = multibody::internal::CombinePointContactParameters(
      deformable_stiffness, rigid_stiffness, deformable_dissipation,
      rigid_dissipation);
  const CoulombFriction<T> mu =
      CalcContactFrictionFromSurfaceProperties(deformable_mu, rigid_mu);
  return internal::DeformableRigidContactPair<T>(std::move(contact_surface),
                                                 rigid_id, deformable_id, k, d,
                                                 mu.dynamic_friction());
}
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager);
