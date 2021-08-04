#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <set>

#include "drake/multibody/fixed_fem/dev/permute_block_sparse_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {

using multibody::contact_solvers::internal::BlockSparseMatrix;
using multibody::contact_solvers::internal::BlockSparseMatrixBuilder;
using multibody::internal::DiscreteContactPair;

template <typename T>
void DeformableRigidManager<T>::RegisterCollisionObjects(
    const geometry::SceneGraph<T>& scene_graph) const {
  const geometry::SceneGraphInspector<T>& inspector =
      scene_graph.model_inspector();
  /* Make sure that the owning plant is registered with the given scene graph.
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
        deformable_model_->fem_model(DeformableBodyIndex(i));
    fem_solvers_.emplace_back(std::make_unique<FemSolver<T>>(&fem_model));
  }
}

template <typename T>
void DeformableRigidManager<T>::RegisterDeformableGeometries() {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  const auto& ref_geometries =
      deformable_model_->reference_configuration_geometries();
  deformable_meshes_.reserve(ref_geometries.size());
  for (const internal::ReferenceDeformableGeometry<T>& geometry :
       ref_geometries) {
    deformable_meshes_.emplace_back(geometry.mesh());
  }
}

template <typename T>
void DeformableRigidManager<T>::DeclareCacheEntries() {
  DRAKE_DEMAND(deformable_model_ != nullptr);

  for (DeformableBodyIndex deformable_body_id(0);
       deformable_body_id < deformable_model_->num_bodies();
       ++deformable_body_id) {
    const FemModelBase<T>& fem_model =
        deformable_model_->fem_model(deformable_body_id);
    const std::unique_ptr<const FemStateBase<T>> model_fem_state =
        fem_model.MakeFemStateBase();

    /* Extracts the q, qdot, and qddot from the given context and copies them
     to the cached fem state. */
    const auto& fem_state_cache_entry = this->DeclareCacheEntry(
        fmt::format("FEM state {}", deformable_body_id),
        systems::ValueProducer(
            *model_fem_state,
            std::function<void(const systems::Context<T>&, FemStateBase<T>*)>(
                std::bind(&DeformableRigidManager<T>::CalcFemStateBase, this,
                          std::placeholders::_1, deformable_body_id,
                          std::placeholders::_2))),
        {systems::System<T>::xd_ticket()});
    fem_state_cache_indexes_.emplace_back(fem_state_cache_entry.cache_index());

    /* Calculates the free-motion velocity for the deformable body. */
    const auto& free_motion_cache_entry = this->DeclareCacheEntry(
        fmt::format("Free motion FEM state {}", deformable_body_id),
        systems::ValueProducer(
            *model_fem_state,
            std::function<void(const systems::Context<T>&, FemStateBase<T>*)>(
                std::bind(
                    &DeformableRigidManager<T>::CalcFreeMotionFemStateBase,
                    this, std::placeholders::_1, deformable_body_id,
                    std::placeholders::_2))),
        {fem_state_cache_entry.ticket()});
    free_motion_cache_indexes_.emplace_back(
        free_motion_cache_entry.cache_index());

    /* Allocates and calculates the free-motion tangent matrix for the
     deformable body. */
    Eigen::SparseMatrix<T> model_tangent_matrix(fem_model.num_dofs(),
                                                fem_model.num_dofs());
    fem_model.SetTangentMatrixSparsityPattern(&model_tangent_matrix);
    const auto& tangent_matrix_cache_entry = this->DeclareCacheEntry(
        fmt::format("Free motion FEM tangent matrix {}", deformable_body_id),
        systems::ValueProducer(model_tangent_matrix,
                               std::function<void(const systems::Context<T>&,
                                                  Eigen::SparseMatrix<T>*)>{
                                   [this, deformable_body_id](
                                       const systems::Context<T>& context,
                                       Eigen::SparseMatrix<T>* tangent_matrix) {
                                     this->CalcFreeMotionTangentMatrix(
                                         context, deformable_body_id,
                                         tangent_matrix);
                                   }}),
        {free_motion_cache_entry.ticket()});
    tangent_matrix_cache_indexes_.emplace_back(
        tangent_matrix_cache_entry.cache_index());

    /* Calculates the Schur complement of the free-motion tangent matrix for the
     deformable body. */
    const auto& tangent_matrix_schur_complement_cache_entry =
        this->DeclareCacheEntry(
            fmt::format("Free motion FEM tangent matrix Schur complement {}",
                        deformable_body_id),
            systems::ValueProducer(std::function{
                [this, deformable_body_id](
                    const systems::Context<T>& context,
                    internal::SchurComplement<T>* schur_complement) {
                  this->CalcFreeMotionTangentMatrixSchurComplement(
                      context, deformable_body_id, schur_complement);
                }}),
            {free_motion_cache_entry.ticket(),
             tangent_matrix_cache_entry.ticket()});
    tangent_matrix_schur_complement_cache_indexes_.emplace_back(
        tangent_matrix_schur_complement_cache_entry.cache_index());
  }

  /* Calculates the contact data for all deformable bodies. */
  const auto& deformable_contact_data_cache_entry = this->DeclareCacheEntry(
      "Deformable contact data",
      systems::ValueProducer(
          this, &DeformableRigidManager<T>::CalcDeformableRigidContact),
      {systems::System<T>::xd_ticket()});
  deformable_contact_data_cache_index_ =
      deformable_contact_data_cache_entry.cache_index();

  /* Calculates the tangent matrix for the contact solver. */
  const auto& contact_tangent_matrix_cache_entry = this->DeclareCacheEntry(
      "Tangent matrix for contact",
      systems::ValueProducer(
          this, &DeformableRigidManager<T>::CalcContactTangentMatrix),
      {systems::System<T>::xd_ticket()});
  contact_tangent_matrix_cache_index_ =
      contact_tangent_matrix_cache_entry.cache_index();

  const auto& free_motion_rigid_velocities_cache_entry =
      this->DeclareCacheEntry(
          "Free motion velocities for rigid dofs",
          systems::ValueProducer(
              this, &DeformableRigidManager<T>::CalcFreeMotionRigidVelocities),
          {systems::System<T>::all_sources_ticket()});
  free_motion_rigid_velocities_cache_index_ =
      free_motion_rigid_velocities_cache_entry.cache_index();

  const auto& participating_free_motion_velocities_cache_entry =
      this->DeclareCacheEntry(
          "Participating free motion velocities for contact",
          systems::ValueProducer(this,
                                 &DeformableRigidManager<
                                     T>::CalcParticipatingFreeMotionVelocities),
          {systems::System<T>::xd_ticket(),
           free_motion_rigid_velocities_cache_entry.ticket()});
  participating_free_motion_velocities_cache_index_ =
      participating_free_motion_velocities_cache_entry.cache_index();
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
  const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs =
      this->EvalDiscreteContactPairs(context);
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
    const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs,
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
  updates->set_value(this->multibody_state_index(), x_next);

  /* Evaluates the deformable free-motion states. */
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes =
      deformable_model_->discrete_state_indexes();
  for (DeformableBodyIndex deformable_body_id(0);
       deformable_body_id < free_motion_cache_indexes_.size();
       ++deformable_body_id) {
    const FemStateBase<T>& state_star =
        EvalFreeMotionFemStateBase(context, deformable_body_id);
    const int num_dofs = state_star.num_generalized_positions();
    // TODO(xuchenhan-tri): This assumes no deformable-rigid contact exists.
    //  Modify this to include the effect of contact.
    /* Copy new state to output variable. */
    Eigen::VectorBlock<VectorX<T>> next_discrete_value =
        updates->get_mutable_value(discrete_state_indexes[deformable_body_id]);
    next_discrete_value.head(num_dofs) = state_star.q();
    next_discrete_value.segment(num_dofs, num_dofs) = state_star.qdot();
    next_discrete_value.tail(num_dofs) = state_star.qddot();
  }
}

template <typename T>
void DeformableRigidManager<T>::CalcFemStateBase(
    const systems::Context<T>& context, DeformableBodyIndex id,
    FemStateBase<T>* fem_state) const {
  const systems::BasicVector<T>& discrete_state =
      context.get_discrete_state().get_vector(
          deformable_model_->discrete_state_indexes()[id]);
  const auto& discrete_value = discrete_state.get_value();
  DRAKE_DEMAND(discrete_value.size() % 3 == 0);
  const int num_dofs = discrete_value.size() / 3;
  const auto& q = discrete_value.head(num_dofs);
  const auto& qdot = discrete_value.segment(num_dofs, num_dofs);
  const auto& qddot = discrete_value.tail(num_dofs);
  fem_state->SetQ(q);
  fem_state->SetQdot(qdot);
  fem_state->SetQddot(qddot);
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionFemStateBase(
    const systems::Context<T>& context, DeformableBodyIndex id,
    FemStateBase<T>* fem_state_star) const {
  const FemStateBase<T>& fem_state = EvalFemStateBase(context, id);
  // TODO(xuchenhan-tri): FemState needs a SetFrom() method.
  fem_state_star->SetQ(fem_state.q());
  fem_state_star->SetQdot(fem_state.qdot());
  fem_state_star->SetQddot(fem_state.qddot());
  /* Obtain the contact-free state for the deformable body. */
  fem_solvers_[id]->AdvanceOneTimeStep(fem_state, fem_state_star);
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionTangentMatrix(
    const systems::Context<T>& context, DeformableBodyIndex index,
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  const FemStateBase<T>& free_motion_fem_state =
      EvalFreeMotionFemStateBase(context, index);
  const FemModelBase<T>& fem_model = deformable_model_->fem_model(index);
  fem_model.CalcTangentMatrix(free_motion_fem_state, tangent_matrix);
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionTangentMatrixSchurComplement(
    const systems::Context<T>& context, DeformableBodyIndex index,
    internal::SchurComplement<T>* schur_complement) const {
  const Eigen::SparseMatrix<T>& free_motion_tangent_matrix =
      EvalFreeMotionTangentMatrix(context, index);
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);
  /* Reindex the deformable dofs so that the tangent matrix assumes a block
   structure associated with the contact pattern that facilitates subsequent
   computations. */
  const Eigen::SparseMatrix<T> permuted_tangent_matrix =
      internal::PermuteBlockSparseMatrix(
          free_motion_tangent_matrix,
          deformable_contact_data[index].permuted_vertex_indexes());
  constexpr int kDimension = 3;
  const int dofs_in_contact =
      deformable_contact_data[index].num_vertices_in_contact() * kDimension;
  const int total_dofs = free_motion_tangent_matrix.rows();
  *schur_complement = internal::SchurComplement<T>(
      permuted_tangent_matrix.topLeftCorner(dofs_in_contact, dofs_in_contact),
      permuted_tangent_matrix.bottomLeftCorner(total_dofs - dofs_in_contact,
                                               dofs_in_contact),
      permuted_tangent_matrix.bottomRightCorner(total_dofs - dofs_in_contact,
                                                total_dofs - dofs_in_contact));
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
    deformable_meshes_[i].UpdateVertexPositions(q);
  }
}

template <typename T>
internal::DeformableRigidContactPair<T>
DeformableRigidManager<T>::CalcDeformableRigidContactPair(
    geometry::GeometryId rigid_id, DeformableBodyIndex deformable_id) const {
  DeformableContactSurface<T> contact_surface = ComputeTetMeshTriMeshContact(
      deformable_meshes_[deformable_id], collision_objects_.mesh(rigid_id),
      collision_objects_.bvh(rigid_id),
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

template <typename T>
internal::DeformableContactData<T>
DeformableRigidManager<T>::CalcDeformableContactData(
    DeformableBodyIndex deformable_id) const {
  /* Calculate all rigid-deformable contact pairs for this deformable body. */
  const std::vector<geometry::GeometryId>& rigid_ids =
      collision_objects_.geometry_ids();
  const int num_rigid_bodies = rigid_ids.size();
  std::vector<internal::DeformableRigidContactPair<T>>
      deformable_rigid_contact_pairs;
  deformable_rigid_contact_pairs.reserve(num_rigid_bodies);
  for (int i = 0; i < num_rigid_bodies; ++i) {
    internal::DeformableRigidContactPair<T> contact_pair =
        CalcDeformableRigidContactPair(rigid_ids[i], deformable_id);
    if (contact_pair.num_contact_points() != 0) {
      deformable_rigid_contact_pairs.emplace_back(std::move(contact_pair));
    }
  }
  return {
      std::move(deformable_rigid_contact_pairs),
      deformable_model_->reference_configuration_geometries()[deformable_id]};
}

template <typename T>
void DeformableRigidManager<T>::CalcDeformableRigidContact(
    const systems::Context<T>& context,
    std::vector<internal::DeformableContactData<T>>* result) const {
  result->clear();
  UpdateCollisionObjectPoses(context);
  UpdateDeformableVertexPositions(context);
  const int num_bodies = deformable_model_->num_bodies();
  result->reserve(num_bodies);
  for (DeformableBodyIndex i(0); i < num_bodies; ++i) {
    result->emplace_back(CalcDeformableContactData(i));
  }
}

template <typename T>
BlockSparseMatrix<T> DeformableRigidManager<T>::CalcContactJacobian(
    const systems::Context<T>& context) const {
  /* Get the rigid-rigid contact info. */
  const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs =
      this->EvalDiscreteContactPairs(context);
  /* Get the deformable-rigid contact info. */
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);

  /* Each deformable body in contact forms a block column in the contact
   jacobian. */
  const int num_deformable_block_cols = std::count_if(
      deformable_contact_data.begin(), deformable_contact_data.end(),
      [](const internal::DeformableContactData<T>& contact_data) {
        return contact_data.num_contact_points() != 0;
      });

  /* Return an empty matrix if no contact exists. */
  if (num_deformable_block_cols == 0 && rigid_contact_pairs.empty()) {
    return BlockSparseMatrix<T>();
  }

  // TODO(xuchenhan-tri): Further exploit the sparsity in contact jacobian
  //  using contact graph for rigid dofs.
  /* For now, all rigid dofs are viewed as a single column block in the contact
   jacobian. */
  const int num_rigid_block_cols = 1;
  /* For now, all rigid-rigid contacts (if they exist) are viewed as a single
   row block in the contact jacobian. */
  const int num_rigid_block_rows = rigid_contact_pairs.empty() ? 0 : 1;
  /* Each deformable body in contact forms one row block. */
  const int num_deformable_block_rows = num_deformable_block_cols;
  /* The number of total row/col blocks is the sum of deformable and rigid
   row/col blocks. */
  const int num_block_rows = num_deformable_block_rows + num_rigid_block_rows;
  const int num_block_cols = num_rigid_block_cols + num_deformable_block_cols;

  /* Rigid-rigid contact accounts for one block in the matrix. Each
   rigid-deformable contact produces two blocks (one in the rigid column block
   and one in the deformable column for the deformable body). If we strictly
   declare 2X the total number of rows, we'll have a safe and tight upper
   bound. */
  BlockSparseMatrixBuilder<T> builder(num_block_rows, num_block_cols,
                                      2 * num_block_rows);

  if (num_rigid_block_rows != 0) {
    /* The rigid-rigid block. */
    builder.PushBlock(0, 0, this->EvalContactJacobians(context).Jc);
  }

  /* The row blocks corresponding to deformable-rigid contacts start after
    rigid-rigid contacts. */
  int row_block = num_rigid_block_rows;
  /* The block columns corresponding to deformable dofs start after the
   rigid dofs. */
  int col_block_deformable = num_rigid_block_cols;
  const int col_block_rigid = 0;

  for (const internal::DeformableContactData<T>& contact_data :
       deformable_contact_data) {
    /* Skip deformable bodies that are not in contact. */
    if (contact_data.num_contact_points() == 0) {
      continue;
    }
    builder.PushBlock(row_block, col_block_rigid,
                      CalcContactJacobianRigidBlock(context, contact_data));
    builder.PushBlock(row_block, col_block_deformable,
                      CalcContactJacobianDeformableBlock(contact_data));
    ++row_block;
    ++col_block_deformable;
  }
  return builder.Build();
}

template <typename T>
MatrixX<T> DeformableRigidManager<T>::CalcContactJacobianDeformableBlock(
    const internal::DeformableContactData<T>& contact_data) const {
  MatrixX<T> Jc = MatrixX<T>::Zero(3 * contact_data.num_contact_points(),
                                   3 * contact_data.num_vertices_in_contact());
  constexpr int kNumVerticesInTetrahedron = 4;
  /* The mapping from vertex indexes of the deformable mesh to the permuted
   vertex indexes used in determining the column index in the contact jacobian.
  */
  const std::vector<int>& permuted_vertex_indexes =
      contact_data.permuted_vertex_indexes();
  DRAKE_DEMAND(deformable_model_ != nullptr);
  const std::vector<internal::ReferenceDeformableGeometry<T>>& ref_geometries =
      deformable_model_->reference_configuration_geometries();

  int contact_point_offset = 0;
  for (const auto& contact_pair : contact_data.contact_pairs()) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pair.contact_surface;
    const geometry::VolumeMesh<T>& reference_volume_mesh =
        ref_geometries[contact_pair.deformable_id].mesh();
    for (int ic = 0; ic < contact_surface.num_polygons(); ++ic) {
      const ContactPolygonData<T>& polygon_data =
          contact_surface.polygon_data(ic);
      /* The contribution to the contact velocity at contact point q from the
       deformable object A is R_CW * v_WAq, where
           v_WAq = ∑ⱼ bⱼ * v_WVᵢⱼ       (1).
       Here, bⱼ is the barycentric weight corresponding to the vertex iⱼ and
       v_WVᵢⱼ is the world frame velocity of the vertex iⱼ.

       Now, we want the deformable block of the contact jacobian Jc to be such
       that when multiplied with the deformable velocities on the right, it
       produces R_CW * v_WAq for the rows corresponding to the contact point
       q. From inspecting equation (1), the 3x3 block in the rows corresponding
       to the contact point q and the columns corresponding to vertex iⱼ has to
       be R_CW * bⱼ. */
      const Vector4<T>& barycentric_weights = polygon_data.b_centroid;
      const geometry::VolumeElement tet_element =
          reference_volume_mesh.element(polygon_data.tet_index);
      for (int j = 0; j < kNumVerticesInTetrahedron; ++j) {
        const int v = permuted_vertex_indexes[tet_element.vertex(j)];
        Jc.template block<3, 3>(3 * contact_point_offset, 3 * v) =
            contact_pair.R_CWs[ic].matrix() * barycentric_weights(j);
      }
      ++contact_point_offset;
    }
  }
  return Jc;
}

template <typename T>
MatrixX<T> DeformableRigidManager<T>::CalcContactJacobianRigidBlock(
    const systems::Context<T>& context,
    const internal::DeformableContactData<T>& contact_data) const {
  MatrixX<T> Jc = MatrixX<T>::Zero(3 * contact_data.num_contact_points(),
                                   this->plant().num_velocities());
  int contact_point_offset = 0;
  for (const auto& contact_pair : contact_data.contact_pairs()) {
    const DeformableContactSurface<T>& contact_surface =
        contact_pair.contact_surface;
    const int num_contact_points = contact_surface.num_polygons();
    if (num_contact_points == 0) {
      continue;
    }
    /* The contact points in world frame. */
    Matrix3X<T> p_WCs(3, num_contact_points);
    for (int i = 0; i < contact_surface.num_polygons(); ++i) {
      p_WCs.col(i) = contact_surface.polygon_data(i).centroid;
    }
    // TODO(xuchenhan-tri): The memory access pattern here is unfriendly for
    //  column major matrices. Consider switching to row major storage.
    /* Extract the contact jacobian block associated with the rigid geometry
     with `rigid_id`. */
    auto Jc_block =
        Jc.middleRows(3 * contact_point_offset, 3 * num_contact_points);
    const geometry::GeometryId rigid_id = contact_pair.rigid_id;
    const Frame<T>& body_frame =
        this->GetBodyFrameFromCollisionGeometry(rigid_id);
    this->internal_tree().CalcJacobianTranslationalVelocity(
        context, JacobianWrtVariable::kV, body_frame,
        this->plant().world_frame(), p_WCs, this->plant().world_frame(),
        this->plant().world_frame(), &Jc_block);
    /* Rotates to the contact frame at each contact point. */
    for (int i = 0; i < contact_pair.num_contact_points(); ++i) {
      /* The contact velocity at the contact point q between a deformable
       body A and a rigid body B is given by
          vc = v_CAq - v_CBq
              = R_CW * (v_WAq - v_WBq)
              = R_CW * (v_WAq - Jv_v_WBq * v)  Here v is rigid velocity dofs.
       From inspection, we see that the rigid block of the contact jacobian is
       -R_CW * Jv_v_WBq. */
      Jc_block.middleRows(3 * i, 3) =
          -contact_pair.R_CWs[i].matrix() * Jc_block.middleRows(3 * i, 3);
    }
    contact_point_offset += num_contact_points;
  }

  /* Sanity check that all rows of the contact jacobian has been written to. */
  DRAKE_DEMAND(3 * contact_point_offset == Jc.rows());
  return Jc;
}

template <typename T>
void DeformableRigidManager<T>::CalcContactPointData(
    const systems::Context<T>& context,
    ContactPointData* contact_point_data) const {
  DRAKE_DEMAND(contact_point_data != nullptr);

  /* Get the rigid-rigid and deformable-rigid contact info. */
  const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs =
      this->EvalDiscreteContactPairs(context);
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);

  const int num_rigid_contacts = rigid_contact_pairs.size();
  int num_deformable_contacts = 0;
  for (const auto& data : deformable_contact_data) {
    num_deformable_contacts += data.num_contact_points();
  }
  const int nc = num_rigid_contacts + num_deformable_contacts;
  contact_point_data->Resize(nc);

  /* Alias for convenience. */
  VectorX<T>& mu = contact_point_data->mu;
  VectorX<T>& phi0 = contact_point_data->phi0;
  VectorX<T>& stiffness = contact_point_data->stiffness;
  VectorX<T>& damping = contact_point_data->damping;

  /* First write the rigid contact data. */
  // TODO(xuchenhan-tri): Consider storing a combined friction value in the
  //  DiscreteContactPair.
  std::vector<CoulombFriction<double>> combined_friction_pairs =
      this->CalcCombinedFrictionCoefficients(context, rigid_contact_pairs);
  for (int i = 0; i < num_rigid_contacts; ++i) {
    mu[i] = combined_friction_pairs[i].dynamic_friction();
    phi0[i] = rigid_contact_pairs[i].phi0;
    stiffness[i] = rigid_contact_pairs[i].stiffness;
    damping[i] = rigid_contact_pairs[i].damping;
  }

  /* Then write the deformable contact data. */
  int contact_offset = num_rigid_contacts;
  /* Loop over deformable bodies. */
  for (const internal::DeformableContactData<T>&
           per_deformable_body_contact_data : deformable_contact_data) {
    /* For each deformable body, loop over rigid object in contact with the
     deformable body. */
    for (int i = 0; i < per_deformable_body_contact_data.num_contact_pairs();
         ++i) {
      const internal::DeformableRigidContactPair<T>&
          deformable_rigid_contact_pair =
              per_deformable_body_contact_data.contact_pairs()[i];
      const int nc_in_pair = deformable_rigid_contact_pair.num_contact_points();
      phi0.segment(contact_offset, nc_in_pair) = VectorX<T>::Map(
          per_deformable_body_contact_data.signed_distances(i).data(),
          nc_in_pair);
      mu.segment(contact_offset, nc_in_pair) =
          VectorX<T>::Ones(nc_in_pair) * deformable_rigid_contact_pair.friction;
      stiffness.segment(contact_offset, nc_in_pair) =
          VectorX<T>::Ones(nc_in_pair) *
          deformable_rigid_contact_pair.stiffness;
      damping.segment(contact_offset, nc_in_pair) =
          VectorX<T>::Ones(nc_in_pair) *
          deformable_rigid_contact_pair.dissipation;
      contact_offset += nc_in_pair;
    }
  }
}

template <typename T>
BlockSparseMatrix<T> DeformableRigidManager<T>::CalcContactTangentMatrix(
    const systems::Context<T>& context) const {
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);

  int num_deformable_body_in_contact = 0;
  for (const auto& contact_data : deformable_contact_data) {
    if (contact_data.num_contact_points() > 0) {
      ++num_deformable_body_in_contact;
    }
  }

  const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs =
      this->EvalDiscreteContactPairs(context);

  /* Return an empty tangent matrix if there is no contact. */
  if (rigid_contact_pairs.empty() && num_deformable_body_in_contact == 0) {
    return BlockSparseMatrix<T>();
  }

  // TODO(xuchenhan-tri): The mass matrix of the rigid dofs is treated as a
  //  single block. Exploit its branch-induced sparsity in the future.
  const int num_rigid_blocks = 1;
  const int num_deformable_blocks = num_deformable_body_in_contact;
  const int num_blocks = num_rigid_blocks + num_deformable_blocks;
  BlockSparseMatrixBuilder<T> builder(num_blocks, num_blocks, num_blocks);

  const int num_rigid_dofs = this->plant().num_velocities();
  MatrixX<T> M(num_rigid_dofs, num_rigid_dofs);
  this->plant().CalcMassMatrix(context, &M);
  /* The rigid dofs come before the deformable dofs in the contact formulation,
   so we put the rigid block on the top-left corner. */
  builder.PushBlock(0, 0, M);

  /* Now start building the deformable blocks. */
  int block_index = 1;
  DRAKE_DEMAND(deformable_model_ != nullptr);
  for (DeformableBodyIndex i(0); i < deformable_model_->num_bodies(); ++i) {
    /* Skip deformable bodies not in contact. */
    if (deformable_contact_data[i].num_contact_points() == 0) {
      continue;
    }
    const internal::SchurComplement<T>& tangent_matrix_schur_complement =
        EvalFreeMotionTangentMatrixSchurComplement(context, i);
    builder.PushBlock(block_index, block_index,
                      tangent_matrix_schur_complement.get_D_complement());
    ++block_index;
  }
  return builder.Build();
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionRigidVelocities(
    const systems::Context<T>& context, VectorX<T>* v_star) const {
  DRAKE_DEMAND(v_star != nullptr);
  /* First calculate all non-contact forces. */
  MultibodyForces<T> forces(this->plant());

  const multibody::internal::PositionKinematicsCache<T>& pc =
      this->plant().EvalPositionKinematics(context);
  const multibody::internal::VelocityKinematicsCache<T>& vc =
      this->plant().EvalVelocityKinematics(context);
  /* Compute forces applied by force elements. Note that this resets forces
   to empty so must come before other force calculations. */
  this->internal_tree().CalcForceElementsContribution(context, pc, vc, &forces);
  this->AddInForcesFromInputPorts(context, &forces);

  /* Perform the tip-to-base pass to compute the force bias terms needed by ABA.
   */
  const auto& tree_topology = this->internal_tree().get_topology();
  multibody::internal::ArticulatedBodyForceCache<T> aba_force_cache(
      tree_topology);
  this->internal_tree().CalcArticulatedBodyForceCache(context, forces,
                                                      &aba_force_cache);

  multibody::internal::AccelerationKinematicsCache<T> ac(tree_topology);
  this->internal_tree().CalcArticulatedBodyAccelerations(context,
                                                         aba_force_cache, &ac);

  /* Notice we are using a symplectic Euler scheme here. All forces are
   evaluated at the start of the time step to obtain the velocity, but the
   position update uses the post-contact velocity at the next time step. */
  const VectorX<T>& vdot0 = ac.get_vdot();
  const double dt = this->plant().time_step();
  const auto& x0 =
      context.get_discrete_state(this->multibody_state_index()).get_value();
  const auto& v0 = x0.bottomRows(this->plant().num_velocities());
  *v_star = v0 + dt * vdot0;
}

template <typename T>
void DeformableRigidManager<T>::CalcParticipatingFreeMotionVelocities(
    const systems::Context<T>& context, VectorX<T>* v_star) const {
  DRAKE_DEMAND(v_star != nullptr);
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);
  int num_deformable_vertices_in_contact = 0;
  for (const auto& contact_data : deformable_contact_data) {
    num_deformable_vertices_in_contact +=
        contact_data.num_vertices_in_contact();
  }
  const std::vector<DiscreteContactPair<T>>& rigid_contact_pairs =
      this->EvalDiscreteContactPairs(context);
  /* Return an empty velocity vector if there is no contact. */
  if (rigid_contact_pairs.empty() && num_deformable_vertices_in_contact == 0) {
    v_star->resize(0);
    return;
  }

  // TODO(xuchenhan-tri): Change the rigid velocities accordingly when the
  //  branch induced sparsity is introduced.
  /* For now, all rigid velocities are participating in contact if *any* contact
   exists. Put them in front of the deformable velocities to follow the same
   order as in CalcContactTangentMatrix(). */
  const int num_rigid_velocities = this->plant().num_velocities();
  const int num_deformable_velocities = num_deformable_vertices_in_contact * 3;
  v_star->resize(num_rigid_velocities + num_deformable_velocities);
  v_star->head(num_rigid_velocities) = EvalFreeMotionRigidVelocities(context);
  /* Now fill in the deformable participating velocities. */
  int v_star_offset = num_rigid_velocities;
  for (DeformableBodyIndex deformable_index(0);
       deformable_index < deformable_contact_data.size(); ++deformable_index) {
    const FemStateBase<T>& fem_state_star =
        EvalFreeMotionFemStateBase(context, deformable_index);
    const VectorX<T>& v_star_deformable = fem_state_star.qdot();
    const std::vector<int>& permuted_to_original_indexes =
        deformable_contact_data[deformable_index]
            .permuted_to_original_indexes();
    for (int vertex = 0;
         vertex <
         deformable_contact_data[deformable_index].num_vertices_in_contact();
         ++vertex) {
      /* For each participating deformable vertex, look up its free motion
       velocity through the mapping from the permuted index to the original
       index. */
      v_star->template segment<3>(v_star_offset) =
          v_star_deformable.template segment<3>(
              3 * permuted_to_original_indexes[vertex]);
      v_star_offset += 3;
    }
  }
  /* Sanity check that all entries in `v_star` has been filled. */
  DRAKE_DEMAND(v_star_offset == v_star->size());
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DeformableRigidManager);
