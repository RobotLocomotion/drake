#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"

#include <map>

#include "drake/multibody/contact_solvers/block_sparse_linear_operator.h"
#include "drake/multibody/fixed_fem/dev/inverse_spd_operator.h"
#include "drake/multibody/fixed_fem/dev/matrix_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace fem {

using multibody::contact_solvers::internal::BlockSparseLinearOperator;
using multibody::contact_solvers::internal::BlockSparseMatrix;
using multibody::contact_solvers::internal::BlockSparseMatrixBuilder;
using multibody::contact_solvers::internal::ContactSolverResults;
using multibody::contact_solvers::internal::InverseSpdOperator;
using multibody::contact_solvers::internal::PointContactData;
using multibody::contact_solvers::internal::SystemDynamicsData;
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
  // TODO(xuchenhan-tri): We have to manually keep the integration schemes in
  //  sync for free-motion solve and post-contact solve which is error prone.
  //  See issue #15620.
  /* We use the Newmark scheme with gamma = 1 and beta = 0.5 to advance states
   for all deformable bodies. */
  velocity_newmark_ = std::make_unique<internal::VelocityNewmarkScheme<T>>(
      this->plant().time_step(), 1, 0.5);
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

    const auto& next_fem_state_cache_entry = this->DeclareCacheEntry(
        fmt::format("FEM state for {} at the next time step",
                    deformable_body_id),
        systems::ValueProducer(
            *model_fem_state,
            std::function<void(const systems::Context<T>&, FemStateBase<T>*)>{
                [this, deformable_body_id](const systems::Context<T>& context,
                                           FemStateBase<T>* next_state) {
                  this->CalcNextFemStateBase(context, deformable_body_id,
                                             next_state);
                }}),
        {systems::System<T>::all_sources_ticket()});
    next_fem_state_cache_indexes_.emplace_back(
        next_fem_state_cache_entry.cache_index());

    /* Allocates and calculates the free-motion tangent matrix for the
     deformable body. */
    EigenSparseMatrix<T> model_tangent_matrix = {
        Eigen::SparseMatrix<T>(fem_model.num_dofs(), fem_model.num_dofs())};
    fem_model.SetTangentMatrixSparsityPattern(&(model_tangent_matrix.data));
    const auto& tangent_matrix_cache_entry = this->DeclareCacheEntry(
        fmt::format("Free motion FEM tangent matrix {}", deformable_body_id),
        systems::ValueProducer(model_tangent_matrix,
                               std::function<void(const systems::Context<T>&,
                                                  EigenSparseMatrix<T>*)>{
                                   [this, deformable_body_id](
                                       const systems::Context<T>& context,
                                       EigenSparseMatrix<T>* tangent_matrix) {
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

  const auto& contact_point_data_cache_entry = this->DeclareCacheEntry(
      "Contact point data",
      systems::ValueProducer(this,
                             &DeformableRigidManager<T>::CalcContactPointData),
      {systems::System<T>::xd_ticket()});
  contact_point_data_cache_index_ =
      contact_point_data_cache_entry.cache_index();

  /* Calculates the tangent matrix for the contact solver. */
  const auto& contact_tangent_matrix_cache_entry = this->DeclareCacheEntry(
      "Tangent matrix for contact",
      systems::ValueProducer(
          this, &DeformableRigidManager<T>::CalcContactTangentMatrix),
      {systems::System<T>::xd_ticket()});
  contact_tangent_matrix_cache_index_ =
      contact_tangent_matrix_cache_entry.cache_index();

  const auto& velocities_cache_entry = this->DeclareCacheEntry(
      "Velocities for all dofs",
      systems::ValueProducer(this, &DeformableRigidManager<T>::CalcVelocities),
      {systems::System<T>::xd_ticket()});
  velocities_cache_index_ = velocities_cache_entry.cache_index();

  const auto& free_motion_velocities_cache_entry = this->DeclareCacheEntry(
      "Free motion velocities for all dofs",
      systems::ValueProducer(
          this, &DeformableRigidManager<T>::CalcFreeMotionVelocities),
      {systems::System<T>::all_sources_ticket()});
  free_motion_velocities_cache_index_ =
      free_motion_velocities_cache_entry.cache_index();

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
          "Participating free motion velocities for contact (v_star)",
          systems::ValueProducer(
              std::function<void(const systems::Context<T>&, VectorX<T>*)>{
                  [this](const systems::Context<T>& context,
                         VectorX<T>* v_star) {
                    this->CalcFreeMotionParticipatingVelocities(context,
                                                                v_star);
                  }}),
          {systems::System<T>::xd_ticket(),
           free_motion_rigid_velocities_cache_entry.ticket()});
  participating_free_motion_velocities_cache_index_ =
      participating_free_motion_velocities_cache_entry.cache_index();

  const auto& participating_velocities_cache_entry = this->DeclareCacheEntry(
      "Participating velocities for contact (v0)",
      systems::ValueProducer(
          std::function<void(const systems::Context<T>&, VectorX<T>*)>{
              [this](const systems::Context<T>& context, VectorX<T>* v0) {
                this->CalcParticipatingVelocities(context, v0);
              }}),
      {systems::System<T>::xd_ticket()});
  participating_velocities_cache_index_ =
      participating_velocities_cache_entry.cache_index();

  const auto& two_way_coupled_contact_solver_results_cache_entry =
      this->DeclareCacheEntry(
          "Two-way coupled contact solver results",
          systems::ValueProducer(this,
                                 &DeformableRigidManager<
                                     T>::CalcTwoWayCoupledContactSolverResults),
          {systems::System<T>::all_sources_ticket()});
  two_way_coupled_contact_solver_results_cache_index_ =
      two_way_coupled_contact_solver_results_cache_entry.cache_index();

  const auto& deformable_contact_solver_results_cache_entry =
      this->DeclareCacheEntry(
          "Contact solver results for participating deformable dofs",
          systems::ValueProducer(
              this,
              &DeformableRigidManager<T>::CalcDeformableContactSolverResults),
          {two_way_coupled_contact_solver_results_cache_entry.ticket()});
  deformable_contact_solver_results_cache_index_ =
      deformable_contact_solver_results_cache_entry.cache_index();
}

template <typename T>
void DeformableRigidManager<T>::DoCalcContactSolverResults(
    const systems::Context<T>& context,
    ContactSolverResults<T>* rigid_results) const {
  DRAKE_DEMAND(rigid_results != nullptr);
  /* Extract the results related to the rigid dofs from the full two-way coupled
   deformable-rigid results. */
  const ContactSolverResults<T>& two_way_coupled_results =
      EvalTwoWayCoupledContactSolverResults(context);
  const int nc = two_way_coupled_results.vn.size();
  const int nv_rigid = this->plant().num_velocities();
  rigid_results->Resize(nv_rigid, nc);
  if (nc > 0) {
    /* If contact exists, extract the results for rigid dofs from the two-way
     coupled results. */
    rigid_results->v_next = two_way_coupled_results.v_next.head(nv_rigid);
    rigid_results->tau_contact =
        two_way_coupled_results.tau_contact.head(nv_rigid);
    rigid_results->fn = two_way_coupled_results.fn.head(nc);
    rigid_results->ft = two_way_coupled_results.ft.head(2 * nc);
    rigid_results->vn = two_way_coupled_results.vn.head(nc);
    rigid_results->vt = two_way_coupled_results.vt.head(2 * nc);
  } else {
    /* If no contact exists, the post-contact velocity is the same as the free
     motion velocity and the generalized contact forces are zero. */
    rigid_results->v_next = EvalFreeMotionRigidVelocities(context);
    rigid_results->tau_contact.setZero();
  }
}

template <typename T>
void DeformableRigidManager<T>::CalcTwoWayCoupledContactSolverResults(
    const systems::Context<T>& context,
    ContactSolverResults<T>* results) const {
  DRAKE_DEMAND(results != nullptr);
  DRAKE_DEMAND(contact_solver_ != nullptr);

  const int rigid_dofs = this->plant().num_velocities();
  /* Quick exit if there are no moving rigid or deformable objects. */
  if (rigid_dofs == 0 && deformable_model_->num_bodies() == 0) {
    results->Resize(0, 0);
    return;
  }

  /* Extract all information needed by the contact solver. */
  /* Point contact data. */
  const BlockSparseMatrix<T> Jc = CalcContactJacobian(context);
  const BlockSparseLinearOperator<T> Jc_op("Contact Jacobian", &Jc);
  const internal::ContactPointData<T>& point_data =
      EvalContactPointData(context);

  /* System dynamics data.*/
  const VectorX<T>& v_star = EvalFreeMotionParticipatingVelocities(context);
  const BlockSparseMatrix<T>& A = EvalContactTangentMatrix(context);
  // TODO(xuchenhan-tri): The inverse is currently calculated inefficiently.
  //  However, since the inverse tangent matrix won't be needed for the new
  //  contact solver. We do not try to optimize it right now. Use the matrix A
  //  instead of its inverse when the new contact solver is available.
  const InverseSpdOperator<T> A_inv_op("Tangent matrix inverse",
                                       A.MakeDenseMatrix());

  const SystemDynamicsData<T> dynamics_data(&A_inv_op, &v_star);
  const PointContactData<T> contact_data(&point_data.phi0, &Jc_op,
                                         &point_data.stiffness,
                                         &point_data.damping, &point_data.mu);
  const int nv = dynamics_data.num_velocities();
  const int nc = contact_data.num_contacts();

  results->Resize(nv, nc);
  /* Alejandro's experiment shows that v0 is no worse than v_star as an initial
   guess in general and is better in steady state. */
  const VectorX<T>& v0 = EvalParticipatingVelocities(context);
  contact_solver_->SolveWithGuess(this->plant().time_step(), dynamics_data,
                                  contact_data, v0, results);
}

template <typename T>
void DeformableRigidManager<T>::CalcDeformableContactSolverResults(
    const systems::Context<T>& context,
    ContactSolverResults<T>* deformable_results) const {
  DRAKE_DEMAND(deformable_results != nullptr);
  /* Extract the results related to the deformable dofs from the full two-way
   coupled deformable-rigid results. */
  const ContactSolverResults<T>& two_way_coupled_results =
      EvalTwoWayCoupledContactSolverResults(context);
  const std::vector<internal::DeformableContactData<T>>&
      deformable_contact_data = EvalDeformableRigidContact(context);

  /* Find the total number of deformable contacts and the number of deformable
   participating dofs. */
  int nc = 0;
  int nv_participating = 0;
  for (const auto& data : deformable_contact_data) {
    nc += data.num_contact_points();
    nv_participating += 3 * data.num_vertices_in_contact();
  }
  deformable_results->Resize(deformable_model_->NumDofs(), nc);
  /* Write to all per contact point results. The deformable-rigid contact point
   results are stored at the end of the two-way coupled results. */
  deformable_results->fn = two_way_coupled_results.fn.tail(nc);
  deformable_results->ft = two_way_coupled_results.ft.tail(2 * nc);
  deformable_results->vn = two_way_coupled_results.vn.tail(nc);
  deformable_results->vt = two_way_coupled_results.vt.tail(2 * nc);

  /* Calculate post-contact velocity and contact impulse for *all* deformable
   dofs using the results on participating deformable dofs. */
  const auto participating_deformable_velocities =
      two_way_coupled_results.v_next.tail(nv_participating);
  const auto participating_deformable_tau =
      two_way_coupled_results.tau_contact.tail(nv_participating);
  int dofs_offset = 0;
  int participating_dofs_offset = 0;
  for (DeformableBodyIndex body(0); body < deformable_model_->num_bodies();
       ++body) {
    const FemStateBase<T>& fem_state_star =
        EvalFreeMotionFemStateBase(context, body);
    const VectorX<T>& v_star = fem_state_star.qdot();
    const int body_num_dofs = v_star.size();
    const auto& body_contact_data = deformable_contact_data[body];
    if (body_contact_data.num_contact_points() == 0) {
      /* If the deformable body is not in contact, then the free motion velocity
      is the final velocity. */
      deformable_results->v_next.segment(dofs_offset, body_num_dofs) = v_star;
      deformable_results->tau_contact.segment(dofs_offset, body_num_dofs)
          .setZero();
    } else {
      const VectorX<T> permuted_v_star = internal::PermuteBlockVector<T>(
          v_star, body_contact_data.permuted_vertex_indexes());
      const int body_num_participating_dofs =
          3 * body_contact_data.num_vertices_in_contact();
      /* Calculate the velocity changes for participating dofs and store them
       in `participating_delta_v`. */
      const auto participating_v_star =
          permuted_v_star.head(body_num_participating_dofs);
      const auto participating_v = participating_deformable_velocities.segment(
          participating_dofs_offset, body_num_participating_dofs);
      const VectorX<T> participating_delta_v =
          participating_v - participating_v_star;
      /* Use Schur complement to calculate the velocity changes for
        non-participating dofs and store them in `nonparticipating_delta_v`. */
      const internal::SchurComplement<T>& schur_complement =
          EvalFreeMotionTangentMatrixSchurComplement(context, body);
      const VectorX<T> nonparticipating_delta_v =
          schur_complement.SolveForY(participating_delta_v);
      /* Calculate v = v* + dv for nonparticipating dofs. */
      const auto nonparticipating_v_star =
          permuted_v_star.tail(body_num_dofs - body_num_participating_dofs);
      const VectorX<T> nonparticipating_v =
          nonparticipating_v_star + nonparticipating_delta_v;
      /* We use `permuted_v` to store the velocity of the body i in permuted
       order at the next time step. */
      VectorX<T> permuted_v(body_num_dofs);
      permuted_v << participating_v, nonparticipating_v;
      /* Restore the velocities to their original order and write to
       `deformable_results`. */
      deformable_results->v_next.segment(dofs_offset, body_num_dofs) =
          internal::PermuteBlockVector<T>(
              permuted_v, body_contact_data.permuted_to_original_indexes());
      /* We use `permuted_tau` to store the contact impulse of the body i in
       permuted order at the next time step. */
      VectorX<T> permuted_tau = VectorX<T>::Zero(body_num_dofs);
      /* Extract `tau` for participating dofs. Non-participating dofs have zero
       `tau`. */
      permuted_tau.head(body_num_participating_dofs) =
          participating_deformable_tau.segment(participating_dofs_offset,
                                               body_num_participating_dofs);
      /* Restore the contact impulses to their original order and write to
       `deformable_results`. */
      deformable_results->tau_contact.segment(dofs_offset, body_num_dofs) =
          internal::PermuteBlockVector<T>(
              permuted_tau, body_contact_data.permuted_to_original_indexes());

      participating_dofs_offset += body_num_participating_dofs;
    }
    dofs_offset += body_num_dofs;
  }
  /* Sanity check that all dofs and participating dofs are accounted for. */
  DRAKE_DEMAND(dofs_offset == deformable_model_->NumDofs());
  DRAKE_DEMAND(participating_dofs_offset == nv_participating);
}

template <typename T>
void DeformableRigidManager<T>::DoCalcDiscreteValues(
    const systems::Context<T>& context,
    systems::DiscreteValues<T>* updates) const {
  /* Calculate the discrete state values for the rigid dofs. */
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

  /* Calculate the discrete state values for the deformable dofs. */
  const std::vector<systems::DiscreteStateIndex>& discrete_state_indexes =
      deformable_model_->discrete_state_indexes();
  for (DeformableBodyIndex body(0); body < discrete_state_indexes.size();
       ++body) {
    Eigen::VectorBlock<VectorX<T>> next_discrete_value =
        updates->get_mutable_value(discrete_state_indexes[body]);
    const int body_num_dofs = next_discrete_value.size() / 3;
    const FemStateBase<T>& next_state = EvalNextFemStateBase(context, body);
    next_discrete_value.head(body_num_dofs) = next_state.q();
    next_discrete_value.segment(body_num_dofs, body_num_dofs) =
        next_state.qdot();
    next_discrete_value.tail(body_num_dofs) = next_state.qddot();
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
void DeformableRigidManager<T>::CalcNextFemStateBase(
    const systems::Context<T>& context, DeformableBodyIndex body_index,
    FemStateBase<T>* fem_state) const {
  DRAKE_DEMAND(deformable_model_ != nullptr);
  /* Calculate the discrete state values for the deformable dofs. */
  const auto& deformable_contact_solver_results =
      EvalDeformableContactSolverResults(context);
  const VectorX<T>& deformable_velocities =
      deformable_contact_solver_results.v_next;
  // TODO(xuchenhan-tri): The dofs offset for a certain deformable body is
  // required throughout multiple methods in this class and should be
  // precomputed.
  int dofs_offset = 0;
  for (DeformableBodyIndex b(0); b < body_index; ++b) {
    dofs_offset += deformable_model_->fem_model(b).num_dofs();
  }
  const int body_num_dofs = deformable_model_->fem_model(body_index).num_dofs();
  const VectorX<T>& body_velocity =
      deformable_velocities.segment(dofs_offset, body_num_dofs);
  const FemStateBase<T>& state0 = EvalFemStateBase(context, body_index);
  velocity_newmark_->AdvanceOneTimeStep(state0, body_velocity, fem_state);
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionTangentMatrix(
    const systems::Context<T>& context, DeformableBodyIndex index,
    EigenSparseMatrix<T>* tangent_matrix) const {
  const FemStateBase<T>& free_motion_fem_state =
      EvalFreeMotionFemStateBase(context, index);
  const FemModelBase<T>& fem_model = deformable_model_->fem_model(index);
  fem_model.CalcTangentMatrix(free_motion_fem_state, &(tangent_matrix->data));
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
  const int num_rigid_block_cols = (this->plant().num_velocities() > 0) ? 1 : 0;
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

  for (const internal::DeformableContactData<T>& contact_data :
       deformable_contact_data) {
    /* Skip deformable bodies that are not in contact. */
    if (contact_data.num_contact_points() == 0) {
      continue;
    }
    if (num_rigid_block_cols > 0) {
      const int col_block_rigid = 0;
      builder.PushBlock(row_block, col_block_rigid,
                        CalcContactJacobianRigidBlock(context, contact_data));
    }
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

  /* Set columns corresponding to dofs under dirichlet boundary conditions to
   zero (if boundary conditions exist). We assume that the boundary conditions
   impose zero velocities. Otherwise, the kinematic relationship between dofs
   and contact velocities will also contain a bias term. */
  DeformableBodyIndex deformable_body_index =
      contact_data.deformable_body_index();
  /* Double check that the contact data isn't empty and there is indeed a
   deformable body associated with this contact. */
  DRAKE_DEMAND(contact_data.num_contact_points() > 0 &&
               deformable_body_index.is_valid());
  const DirichletBoundaryCondition<T>* bc =
      deformable_model_->fem_model(deformable_body_index)
          .dirichlet_boundary_condition();
  if (bc != nullptr) {
    const std::map<DofIndex, VectorX<T>>& bc_map = bc->get_bcs();
    const std::vector<int>& permuted_to_original_indexes =
        contact_data.permuted_to_original_indexes();
    for (int permuted_v = 0;
         permuted_v < contact_data.num_vertices_in_contact(); ++permuted_v) {
      const int v = permuted_to_original_indexes[permuted_v];
      for (int d = 0; d < 3; ++d) {
        const int dof_index = 3 * v + d;
        if (bc_map.find(DofIndex(dof_index)) != bc_map.end()) {
          const int permuted_dof_index = 3 * permuted_v + d;
          Jc.col(permuted_dof_index).setZero();
        }
      }
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

  /* Sanity check that all rows of the contact jacobian has been written to.
   */
  DRAKE_DEMAND(3 * contact_point_offset == Jc.rows());
  return Jc;
}

template <typename T>
void DeformableRigidManager<T>::CalcContactPointData(
    const systems::Context<T>& context,
    internal::ContactPointData<T>* contact_point_data) const {
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
  const int num_rigid_dofs = this->plant().num_velocities();
  const int num_rigid_blocks = (num_rigid_dofs == 0) ? 0 : 1;
  const int num_deformable_blocks = num_deformable_body_in_contact;
  const int num_blocks = num_rigid_blocks + num_deformable_blocks;
  BlockSparseMatrixBuilder<T> builder(num_blocks, num_blocks, num_blocks);

  if (num_rigid_blocks > 0) {
    MatrixX<T> M(num_rigid_dofs, num_rigid_dofs);
    this->plant().CalcMassMatrix(context, &M);
    /* The rigid dofs come before the deformable dofs in the contact
     formulation, so we put the rigid block on the top-left corner. */
    builder.PushBlock(0, 0, M);
  }

  /* Now start building the deformable blocks. */
  int block_index = num_rigid_blocks;
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

  /* Perform the tip-to-base pass to compute the force bias terms needed by
   * ABA.
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
void DeformableRigidManager<T>::CalcFreeMotionParticipatingVelocities(
    const systems::Context<T>& context,
    VectorX<T>* participating_v_star) const {
  const VectorX<T>& v_star = EvalFreeMotionVelocities(context);
  ExtractParticipatingVelocities(context, v_star, participating_v_star);
}

template <typename T>
void DeformableRigidManager<T>::CalcParticipatingVelocities(
    const systems::Context<T>& context, VectorX<T>* participating_v0) const {
  const VectorX<T>& v0 = EvalVelocities(context);
  ExtractParticipatingVelocities(context, v0, participating_v0);
}

template <typename T>
void DeformableRigidManager<T>::ExtractParticipatingVelocities(
    const systems::Context<T>& context, const VectorX<T>& v,
    VectorX<T>* participating_v) const {
  DRAKE_DEMAND(participating_v != nullptr);
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
    participating_v->resize(0);
    return;
  }

  // TODO(xuchenhan-tri): Change the rigid velocities accordingly when the
  //  branch induced sparsity is introduced.
  /* For now, all rigid velocities are participating in contact if *any*
   contact exists. Put them in front of the deformable velocities to follow
   the same order as in CalcContactTangentMatrix(). */
  const int num_rigid_velocities = this->plant().num_velocities();
  const int num_deformable_velocities = num_deformable_vertices_in_contact * 3;
  participating_v->resize(num_rigid_velocities + num_deformable_velocities);
  participating_v->head(num_rigid_velocities) = v.head(num_rigid_velocities);
  /* Now extract the deformable participating velocities. */
  int v_offset = num_rigid_velocities;
  int participating_v_offset = num_rigid_velocities;
  for (DeformableBodyIndex deformable_index(0);
       deformable_index < deformable_contact_data.size(); ++deformable_index) {
    const int num_dofs =
        deformable_model_->fem_model(deformable_index).num_dofs();
    /* The velocity of all dofs associated with the deformable body with index
     `deformable_index`. */
    const auto& v_body = v.segment(v_offset, num_dofs);
    const std::vector<int>& permuted_to_original_indexes =
        deformable_contact_data[deformable_index]
            .permuted_to_original_indexes();
    for (int vertex = 0;
         vertex <
         deformable_contact_data[deformable_index].num_vertices_in_contact();
         ++vertex) {
      /* For each participating deformable vertex, look up its (free motion)
       velocity through the mapping from the permuted index to the original
       index. */
      participating_v->template segment<3>(participating_v_offset) =
          v_body.template segment<3>(3 * permuted_to_original_indexes[vertex]);
      participating_v_offset += 3;
    }
    v_offset += num_dofs;
  }
  /* Sanity check that all entries in `participating_v` have been filled and
   all entries in `v` have been scanned through. */
  DRAKE_DEMAND(v_offset == v.size());
  DRAKE_DEMAND(participating_v_offset == participating_v->size());
}

template <typename T>
void DeformableRigidManager<T>::CalcVelocities(
    const systems::Context<T>& context, VectorX<T>* v) const {
  DRAKE_DEMAND(v != nullptr);
  const int num_rigid_velocities = this->plant().num_velocities();
  const int num_deformable_velocities = deformable_model_->NumDofs();
  v->resize(num_rigid_velocities + num_deformable_velocities);
  v->head(num_rigid_velocities) = this->plant().GetVelocities(context);
  int dofs_offset = num_rigid_velocities;
  for (DeformableBodyIndex deformable_index(0);
       deformable_index < deformable_model_->num_bodies(); ++deformable_index) {
    const VectorX<T>& deformable_v =
        EvalFemStateBase(context, deformable_index).qdot();
    v->segment(dofs_offset, deformable_v.size()) = deformable_v;
    dofs_offset += deformable_v.size();
  }
  /* Sanity check that all entries in `v` have been filled. */
  DRAKE_DEMAND(dofs_offset == v->size());
}

template <typename T>
void DeformableRigidManager<T>::CalcFreeMotionVelocities(
    const systems::Context<T>& context, VectorX<T>* v_star) const {
  DRAKE_DEMAND(v_star != nullptr);
  const int num_rigid_velocities = this->plant().num_velocities();
  const int num_deformable_velocities = deformable_model_->NumDofs();
  v_star->resize(num_rigid_velocities + num_deformable_velocities);
  v_star->head(num_rigid_velocities) = EvalFreeMotionRigidVelocities(context);
  int dofs_offset = num_rigid_velocities;
  for (DeformableBodyIndex deformable_index(0);
       deformable_index < deformable_model_->num_bodies(); ++deformable_index) {
    const VectorX<T>& deformable_v_star =
        EvalFreeMotionFemStateBase(context, deformable_index).qdot();
    v_star->segment(dofs_offset, deformable_v_star.size()) = deformable_v_star;
    dofs_offset += deformable_v_star.size();
  }
  /* Sanity check that all entries in `v_star` have been filled. */
  DRAKE_DEMAND(dofs_offset == v_star->size());
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DeformableRigidManager);
