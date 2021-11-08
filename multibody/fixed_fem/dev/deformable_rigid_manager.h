#pragma once

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/fixed_fem/dev/collision_objects.h"
#include "drake/multibody/fixed_fem/dev/deformable_contact_data.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_contact_pair.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/fixed_fem/dev/schur_complement.h"
#include "drake/multibody/fixed_fem/dev/velocity_newmark_scheme.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/discrete_update_manager.h"

namespace drake {
namespace multibody {
namespace fem {

namespace internal {
/* Struct to hold data (friction, signed distance-like value, stiffness, and
 damping) at each contact point in DeformableRigidManager. */
template <typename T>
struct ContactPointData {
  VectorX<T> mu;
  VectorX<T> phi0;
  VectorX<T> stiffness;
  VectorX<T> damping;
  void Resize(int size) {
    mu.resize(size);
    phi0.resize(size);
    stiffness.resize(size);
    damping.resize(size);
  }
};
}  // namespace internal

/** %DeformableRigidManager implements the interface in DiscreteUpdateManager
 and performs discrete update for deformable and rigid bodies with a two-way
 coupling scheme.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class DeformableRigidManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableRigidManager)

  /** Constructs a %DeformableRigidManager that takes ownership of the given
   `contact_solver` to solve for contacts.
   @pre contact_solver != nullptr. */
  DeformableRigidManager(
      std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
          contact_solver)
      : contact_solver_(std::move(contact_solver)) {
    DRAKE_DEMAND(contact_solver_ != nullptr);
  }

  /** Sets the given `contact_solver` as the solver that `this`
   %DeformableRigidManager uses to solve contact.
   @pre contact_solver != nullptr. */
  void SetContactSolver(
      std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
          contact_solver) {
    DRAKE_DEMAND(contact_solver != nullptr);
    contact_solver_ = std::move(contact_solver);
  }

  // TODO(xuchenhan-tri): Remove this method when SceneGraph owns all rigid and
  //  deformable geometries.
  // TODO(xuchenhan-tri): Reconcile the names of "deformable geometries" and
  //  "collision objects" when moving out of dev.
  /** Registers all the rigid collision geometries from the owning
   MultibodyPlant that are registered in the given SceneGraph into `this`
   %DeformableRigidManager. The registered rigid collision geometries will be
   used to generate rigid-deformable contact pairs to be used for the
   rigid-deformable two-way coupled contact solve. A common workflow to set up a
   simulation where deformable and rigid bodies interact with each other through
   contact looks like the following:
   ```
   // Set up a deformable model assciated with a MultibodyPlant.
   auto deformable_model = std::make_unique<DeformableModel<double>>(&plant);
   // Add deformable bodies to the model.
   deformable_model->RegisterDeformableBody(...);
   deformable_model->RegisterDeformableBody(...);
   // Done building the model. Move the DeformableModel into the MultibodyPlant.
   plant.AddPhysicalModel(std::move(deformable_model));
   // Register the plant as a source for scene graph for rigid geometries.
   plant.RegisterAsSourceForSceneGraph(&scene_graph);
   // Add rigid bodies.
   Parser parser(&plant, &scene_graph);
   parser.AddModelFromFile(...);
   parser.AddModelFromFile(...);
   // Done building the plant.
   plant.Finalize();
   // Use a DeformableRigidManager to perform the discrete updates.
   auto& deformable_rigid_manager = plant.SetDiscreteUpdateManager(
        std::make_unqiue<DeformableRigidManager<double>());
   // Register all rigid collision geometries at the manager.
   deformable_rigid_manager.RegisterCollisionObjects(scene_graph);
   ```
   @pre `This` %DeformableRigidManager is owned by a MultibodyPlant via the call
   MultibodyPlant::SetDiscreteUpdateManager().
   @pre The owning MultibodyPlant is registered as a source of the given
   `scene_graph`. */
  void RegisterCollisionObjects(
      const geometry::SceneGraph<T>& scene_graph) const;

 private:
  template <typename Scalar, int Options = 0, typename StorageIndex = int>
  /* Wrapper around Eigen::SparseMatrix to avoid non-type template parameters
   that trigger typename hasher to spew warning messages to the console in a
   simulation. */
  struct EigenSparseMatrix {
    using NonTypeTemplateParameter = std::integral_constant<int, Options>;
    Eigen::SparseMatrix<Scalar, Options, StorageIndex> data;
  };

  friend class DeformableRigidManagerTest;
  friend class DeformableRigidContactDataTest;
  friend class DeformableRigidDynamicsDataTest;
  friend class DeformableRigidContactSolverTest;

  // TODO(xuchenhan-tri): Implement CloneToDouble() and CloneToAutoDiffXd() and
  //  the corresponding is_cloneable methods.

  /* Implements DiscreteUpdateManager::ExtractModelInfo(). Verifies that
   exactly one DeformableModel is registered in the owning plant and
   sets up FEM solvers for deformable bodies. */
  void ExtractModelInfo() final;

  /* Make the FEM solvers that solve the deformable FEM models. */
  void MakeFemSolvers();

  // TODO(xuchenhan-tri): Remove this temporary geometry solutions when
  //  SceneGraph manages all deformable geometries.
  /* Registers the geometries in the DeformableModel in the owning
   MultibodyPlant at `this` DeformableRigidManager. */
  void RegisterDeformableGeometries();

  void DeclareCacheEntries() final;

  void DoCalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const final;

  /* Eval version of CalcTwoWayCoupledContactSolverResults(). */
  const contact_solvers::internal::ContactSolverResults<T>&
  EvalTwoWayCoupledContactSolverResults(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(two_way_coupled_contact_solver_results_cache_index_)
        .template Eval<contact_solvers::internal::ContactSolverResults<T>>(
            context);
  }

  /* Calculates the two-way coupled contact solver results with the results for
   rigid dofs before the results for participating deformable dofs. See class
   documentation of multibody::contact_solvers::internal::ContactSolver for a
   description of the contact problem being solved. */
  void CalcTwoWayCoupledContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const;

  /* Eval version of CalcDeformableContactSolverResults(). */
  const contact_solvers::internal::ContactSolverResults<T>&
  EvalDeformableContactSolverResults(const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(deformable_contact_solver_results_cache_index_)
        .template Eval<contact_solvers::internal::ContactSolverResults<T>>(
            context);
  }

  /* Calculates the contact solver results for the deformable dofs only. */
  void CalcDeformableContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* deformable_results)
      const;

  // TODO(xuchenhan-tri): Implement this once AccelerationKinematicsCache
  //  also caches acceleration for deformable dofs.
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {
    throw std::logic_error(
        "DoCalcAcclerationKinematicsCache() hasn't be implemented for "
        "DeformableRigidManager yet.");
  }

  void DoCalcDiscreteValues(const systems::Context<T>& context,
                            systems::DiscreteValues<T>* updates) const final;

  /* Eval version of CalcFemStateBase(). */
  const FemStateBase<T>& EvalFemStateBase(const systems::Context<T>& context,
                                          DeformableBodyIndex id) const {
    return this->plant()
        .get_cache_entry(fem_state_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

  /* Calculates the FEM state of the deformable body with index `id`. */
  void CalcFemStateBase(const systems::Context<T>& context,
                        DeformableBodyIndex id,
                        FemStateBase<T>* fem_state) const;

  /* Eval version of CalcFreeMotionFemStateBase(). */
  const FemStateBase<T>& EvalFreeMotionFemStateBase(
      const systems::Context<T>& context, DeformableBodyIndex id) const {
    return this->plant()
        .get_cache_entry(free_motion_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

  /* Calculates the free motion FEM state of the deformable body with index
   `id`. */
  void CalcFreeMotionFemStateBase(const systems::Context<T>& context,
                                  DeformableBodyIndex id,
                                  FemStateBase<T>* fem_state_star) const;

  /* Eval version of CalcNextFemStateBase(). */
  const FemStateBase<T>& EvalNextFemStateBase(
      const systems::Context<T>& context,
      DeformableBodyIndex body_index) const {
    return this->plant()
        .get_cache_entry(next_fem_state_cache_indexes_[body_index])
        .template Eval<FemStateBase<T>>(context);
  }

  /* Calculates the FEM state at the next time step for the deformable body with
   the given `body_index`. */
  void CalcNextFemStateBase(const systems::Context<T>& context,
                            DeformableBodyIndex body_index,
                            FemStateBase<T>* fem_state) const;

  /* Eval version of CalcFreeMotionTangentMatrix(). */
  const Eigen::SparseMatrix<T>& EvalFreeMotionTangentMatrix(
      const systems::Context<T>& context, DeformableBodyIndex index) const {
    return this->plant()
        .get_cache_entry(tangent_matrix_cache_indexes_[index])
        .template Eval<EigenSparseMatrix<T>>(context)
        .data;
  }

  /* Calculates the tangent matrix of the deformable body with the given `index`
   at free motion state. */
  void CalcFreeMotionTangentMatrix(const systems::Context<T>& context,
                                   DeformableBodyIndex index,
                                   EigenSparseMatrix<T>* tangent_matrix) const;

  /* Eval version of CalcFreeMotionTangentMatrixSchurComplement(). */
  const internal::SchurComplement<T>&
  EvalFreeMotionTangentMatrixSchurComplement(const systems::Context<T>& context,
                                             DeformableBodyIndex index) const {
    return this->plant()
        .get_cache_entry(tangent_matrix_schur_complement_cache_indexes_[index])
        .template Eval<internal::SchurComplement<T>>(context);
  }

  /* Calculates the Schur complement of the tangent matrix of the deformable
   body with the given `index`. More specifically, the tangent matrix of the
   deformable body is permuted to take the form M = [A, B; Bᵀ, D] where block A
   corresponds to dofs in contact and block D corresponds to dofs not in
   contact. The returned Schur complement provides information on
   M/D = A - BD⁻¹Bᵀ, which is guaranteed to be symmetric positive definite. */
  void CalcFreeMotionTangentMatrixSchurComplement(
      const systems::Context<T>& context, DeformableBodyIndex index,
      internal::SchurComplement<T>* schur_complement) const;

  // TODO(xuchenhan-tri): Remove this method when SceneGraph takes control of
  //  all geometries. SceneGraph should be responsible for obtaining the most
  //  up-to-date rigid body poses.
  /* Updates the world poses of all rigid collision geometries registered in
   `this` DeformableRigidManager. */
  void UpdateCollisionObjectPoses(const systems::Context<T>& context) const;

  // TODO(xuchenhan-tri): This method (or similar) should belong to SceneGraph
  //  when SceneGraph takes control of all geometries.
  /* Updates the vertex positions for all deformable meshes. */
  void UpdateDeformableVertexPositions(
      const systems::Context<T>& context) const;

  // TODO(xuchenhan-tri): Make proper distinction between id and index in
  //  variable names.
  /* Calculates the contact information for the contact pair consisting of the
   rigid body identified by `rigid_id` and the deformable body identified by
   `deformable_id`. */
  internal::DeformableRigidContactPair<T> CalcDeformableRigidContactPair(
      geometry::GeometryId rigid_id, DeformableBodyIndex deformable_id) const;

  /* Calculates and returns the DeformableContactData that contains information
   about all deformable-rigid contacts associated with the deformable body
   identified by `deformable_id`. */
  internal::DeformableContactData<T> CalcDeformableContactData(
      DeformableBodyIndex deformable_id) const;

  /* Eval version of CalcDeformableRigidContact(). */
  const std::vector<internal::DeformableContactData<T>>&
  EvalDeformableRigidContact(const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(deformable_contact_data_cache_index_)
        .template Eval<std::vector<internal::DeformableContactData<T>>>(
            context);
  }

  // TODO(xuchenhan-tri): Skip empty contact data altogether.
  /* Calculates all deforamble-rigid contacts and returns a vector of
   DeformableContactData in which the i-th entry contains contact information
   about the i-th deformable body against all rigid bodies. If the i-th
   deformable body is not in contact with any rigid body, then the i-th entry
   (data[i]) in the return value will have `data[i].num_contact_points() == 0`.
  */
  void CalcDeformableRigidContact(
      const systems::Context<T>& context,
      std::vector<internal::DeformableContactData<T>>* result) const;

  // TODO(xuchenhan-tri): Modify the description of the contact jacobian when
  //  sparsity for rigid dofs are exploited.
  /* Calculates and returns the contact jacobian as a block sparse matrix.
   As an illustrating example, consider a scene with n rigid bodies and m
   deformable bodies. Four of those deformable bodies are in contact with rigid
   geometries. Then the sparsity pattern of the contact jacobian looks like:
                                | RR               |
                                | RD0  D0          |
                                | RD1     D1       |
                                | RD2        D2    |
                                | RD3           D3 |
   where "RR" represents the rigid-rigid block, "RDi" represents the
   rigid-deformable block with respect to the rigid dofs for the i-th deformable
   body, and "Di" represents the rigid-deformable block with respect to the
   deformable dofs for the i-th deformable body.

   More specifically, the contact jacobian is ordered in the following way:
    1. The contact jacobian has 3 * (ncr + ncd) rows, where ncr is the number of
       contact points among rigid objects, and ncd is the number of contact
       points between deformable bodies and rigid bodies, i.e, the sum of
       DeformableContactData::num_contact_points() for all deformable bodies in
       contact.
    2. Rows 3*i, 3*i+1, and 3*i+2 correspond to the i-th rigid-rigid
       contact point for i = 0, ..., ncr-1. These contact points are ordered as
       given by the result of `EvalDiscreteContactPairs()`. Rows 3*(i+ncr),
       3*(i+ncr)+1, and 3*(i+ncr)+2 correspond to the i-th rigid-deformable
       contact points for i = 0, ..., ncd-1. These contact points are ordered as
       given by the result of `CalcDeformableRigidContact()`.
    3. The contact jacobian has nvr + 3 * nvd columns, where nvr is the number
       of rigid velocity degrees of freedom and nvd is the number of deformable
       vertices participating in contact. A vertex of a deformable body is said
       to be participating in contact if it is incident to a tetrahedron that
       contains a contact point.
    4. The first nvr columns of the contact jacobian correspond to the rigid
       velocity degrees of freedom. The last 3 * nvd columns correspond to the
       participating deformable velocity degrees of freedom. The participating
       deformable velocity dofs come in blocks. The i-th block corresponds to
       i-th deformable body and has number of dofs equal to 3 * number of
       participating vertices for deformable body i (see
       DeformableContactData::num_vertices_in_contact()). Within the i-th block,
       the 3*j, 3*j+1, and 3*j+2 velocity dofs belong to the j-th permuted
       vertex. (see DeformableContactData::permuted_vertex_indexes()). */
  multibody::contact_solvers::internal::BlockSparseMatrix<T>
  CalcContactJacobian(const systems::Context<T>& context) const;

  /* Given the contact data for a deformable body, calculates the contact
   jacobian for the contact points associated with that deformable body with
   respect to the deformable degrees of freedom participating in the contact. */
  MatrixX<T> CalcContactJacobianDeformableBlock(
      const internal::DeformableContactData<T>& contact_data) const;

  /* Given the contact data for a deformable body, calculates the contact
   jacobian for the contact points associated with that deformable body with
   respect to all rigid degrees of freedom. */
  MatrixX<T> CalcContactJacobianRigidBlock(
      const systems::Context<T>& context,
      const internal::DeformableContactData<T>& contact_data) const;

  /* Eval version of CalcContactPointData(). */
  const internal::ContactPointData<T>& EvalContactPointData(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(contact_point_data_cache_index_)
        .template Eval<internal::ContactPointData<T>>(context);
  }

  /* Calculates the combined friction, stiffness, damping, and penetration
   distance at all contact points. The way that the contact points are ordered
   in `contact_point_data` is directly correlated with the entries in the result
   of CalcContactJacobian(). In particular, the i-th entry in the
   `contact_point_data` corresponds to the contact point associated with the
   3*i, 3*i+1, and 3*i+2-th rows in the result of CalcContactJacobian(). */
  void CalcContactPointData(
      const systems::Context<T>& context,
      internal::ContactPointData<T>* contact_point_data) const;

  /* Eval version of CalcContactTangentMatrix(). */
  const contact_solvers::internal::BlockSparseMatrix<T>&
  EvalContactTangentMatrix(const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(contact_tangent_matrix_cache_index_)
        .template Eval<contact_solvers::internal::BlockSparseMatrix<T>>(
            context);
  }

  /* Calculates the tangent matrix used by the contact solver. The number and
   order of columns of the calculated tangent matrix is the same as the contact
   Jacobian matrix (see CalcContactJacobian()). The result is guaranteed to be
   symmetric positive definite. */
  contact_solvers::internal::BlockSparseMatrix<T> CalcContactTangentMatrix(
      const systems::Context<T>& context) const;

  /* Eval version of CalcFreeMotionRigidVelocities(). */
  const VectorX<T>& EvalFreeMotionRigidVelocities(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(free_motion_rigid_velocities_cache_index_)
        .template Eval<VectorX<T>>(context);
  }

  /* Calculates the free motion velocities for the rigid dofs. */
  void CalcFreeMotionRigidVelocities(const systems::Context<T>& context,
                                     VectorX<T>* v_star) const;

  /* Eval version of CalcFreeMotionParticipatingVelocities(). */
  const VectorX<T>& EvalFreeMotionParticipatingVelocities(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(participating_free_motion_velocities_cache_index_)
        .template Eval<VectorX<T>>(context);
  }

  /* Calculates the free motion velocities of the participating dofs used by the
   contact solver. See ExtractParticipatingVelocities() for definition of
   "participating". */
  void CalcFreeMotionParticipatingVelocities(
      const systems::Context<T>& context,
      VectorX<T>* participating_v_star) const;

  /* Eval version of CalcParticipatingVelocities(). */
  const VectorX<T>& EvalParticipatingVelocities(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(participating_velocities_cache_index_)
        .template Eval<VectorX<T>>(context);
  }

  /* Calculates the velocities of the participating dofs used by the contact
   solver at the previous time step. See ExtractParticipatingVelocities() for
   definition of "participating". */
  void CalcParticipatingVelocities(const systems::Context<T>& context,
                                   VectorX<T>* participating_v0) const;

  /* Extracts the velocities of the participating dofs used by the contact
   solver from the vector of all velocities `v`. A dof is considered as
   "participating" if
       1. it belongs to *any* rigid body and there exist either rigid-rigid
          contact or rigid-deformable contact, or,
       2. it belongs to a deformable body that participates in deformable-rigid
          contact (see DeformableContactData::permute_vertex_indexes()).
   The number and the order of the participating velocities are the same as the
   columns of the contact tangent matrix (see CalcContactTangentMatrix()). This
   method properly resizes the output argument `participating_v` so the caller
   doesn't have to resize it. */
  void ExtractParticipatingVelocities(const systems::Context<T>& context,
                                      const VectorX<T>& v,
                                      VectorX<T>* pariticipating_v) const;

  /* Eval version of CalcVelocities(). */
  const VectorX<T>& EvalVelocities(const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(velocities_cache_index_)
        .template Eval<VectorX<T>>(context);
  }

  /* Extracts the generalized velocities stored in context for both rigid and
   deformable dofs. */
  void CalcVelocities(const systems::Context<T>& context, VectorX<T>* v) const;

  /* Eval version of CalcFreeMotionVelocities(). */
  const VectorX<T>& EvalFreeMotionVelocities(
      const systems::Context<T>& context) const {
    return this->plant()
        .get_cache_entry(free_motion_velocities_cache_index_)
        .template Eval<VectorX<T>>(context);
  }

  /* Calculates the free motion velocities of all rigid and deformable dofs,
   with the rigid velocities coming before deformable velocities. */
  void CalcFreeMotionVelocities(const systems::Context<T>& context,
                                VectorX<T>* v_star) const;

  /* Given the GeometryId of a rigid collision geometry, returns the body frame
   of the collision geometry.
   @pre The collision geometry with the given `id` is already registered with
   `this` DeformableRigidManager. */
  const Frame<T>& GetBodyFrameFromCollisionGeometry(
      geometry::GeometryId id) const {
    BodyIndex body_index = this->geometry_id_to_body_index().at(id);
    const Body<T>& body = this->plant().get_body(body_index);
    return body.body_frame();
  }

  /* The deformable models being solved by `this` manager. */
  const DeformableModel<T>* deformable_model_{nullptr};
  /* Cached FEM state quantities. */
  std::vector<systems::CacheIndex> fem_state_cache_indexes_;
  std::vector<systems::CacheIndex> free_motion_cache_indexes_;
  std::vector<systems::CacheIndex> next_fem_state_cache_indexes_;
  std::vector<systems::CacheIndex> tangent_matrix_cache_indexes_;
  std::vector<systems::CacheIndex>
      tangent_matrix_schur_complement_cache_indexes_;
  /* Cached contact query results. */
  systems::CacheIndex deformable_contact_data_cache_index_;
  /* Cached contact point data. */
  systems::CacheIndex contact_point_data_cache_index_;
  /* Cached tangent matrix for contact. */
  systems::CacheIndex contact_tangent_matrix_cache_index_;
  /* Cached v0/v* for all dofs. */
  systems::CacheIndex velocities_cache_index_;
  systems::CacheIndex free_motion_velocities_cache_index_;
  /* Cached free motion velocities for rigid dofs. */
  systems::CacheIndex free_motion_rigid_velocities_cache_index_;
  /* Cached participating velocities for contact. */
  systems::CacheIndex participating_velocities_cache_index_;
  systems::CacheIndex participating_free_motion_velocities_cache_index_;
  /* Cached two-way coupled contact solver results. */
  systems::CacheIndex two_way_coupled_contact_solver_results_cache_index_;
  /* Cached contact solver results for deformable dofs only. */
  systems::CacheIndex deformable_contact_solver_results_cache_index_;

  /* Solvers for all deformable bodies. */
  std::vector<std::unique_ptr<FemSolver<T>>> fem_solvers_{};
  std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
      contact_solver_{nullptr};
  std::unique_ptr<internal::VelocityNewmarkScheme<T>> velocity_newmark_;

  /* Geometries temporarily managed by DeformableRigidManager. In the future,
   SceneGraph will manage all the geometries. */
  mutable std::vector<geometry::internal::DeformableVolumeMesh<T>>
      deformable_meshes_;
  mutable internal::CollisionObjects<T> collision_objects_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::DeformableRigidManager)
