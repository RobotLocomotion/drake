#pragma once

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/fixed_fem/dev/collision_objects.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/discrete_update_manager.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %DeformableRigidManager implements the interface in DiscreteUpdateManager
 and performs discrete update for deformable and rigid bodies with a two-way
 coupling scheme.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class DeformableRigidManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableRigidManager)

  DeformableRigidManager() = default;

  /** Sets the given `contact_solver` as the solver that `this`
    %DeformableRigidManager uses to solve contact. */
  void set_contact_solver(
      std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
          contact_solver) {
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
   auto& deformable_rigid_manager = plant.set_discrete_update_manager(
        std::make_unqiue<DeformableRigidManager<double>());
   // Register all rigid collision geometries at the manager.
   deformable_rigid_manager.RegisterCollisionObjects(scene_graph);
   ```
   @pre `This` %DeformableRigidManager is owned by a MultibodyPlant via the call
   MultibodyPlant::set_discrete_update_manager().
   @pre The owning MultibodyPlant is registered as a source of the given
   `scene_graph`. */
  void RegisterCollisionObjects(const geometry::SceneGraph<T>& scene_graph);

 private:
  friend class DeformableRigidManagerTest;

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

  void DeclareCacheEntries(MultibodyPlant<T>* plant) final;

  void DoCalcContactSolverResults(
      const systems::Context<T>& context,
      contact_solvers::internal::ContactSolverResults<T>* results) const final;

  /* Calculates all contact quantities needed by the contact solver and the
   TAMSI solver from the given `context` and `rigid_contact_pairs`.
   @pre All pointer parameters are non-null.
   @pre The size of `v` and `minus_tau` are equal to the number of rigid
        generalized velocities.
   @pre `M` is square and has rows and columns equal to the number of rigid
        generalized velocities.
   @pre `mu`, `phi`, `fn`, `stiffness`, `damping`, and `rigid_contact_pairs`
        have the same size. */
  void CalcContactQuantities(
      const systems::Context<T>& context,
      const std::vector<multibody::internal::DiscreteContactPair<T>>&
          rigid_contact_pairs,
      multibody::internal::ContactJacobians<T>* rigid_contact_jacobians,
      EigenPtr<VectorX<T>> v, EigenPtr<MatrixX<T>> M,
      EigenPtr<VectorX<T>> minus_tau, EigenPtr<VectorX<T>> mu,
      EigenPtr<VectorX<T>> phi, EigenPtr<VectorX<T>> fn,
      EigenPtr<VectorX<T>> stiffness, EigenPtr<VectorX<T>> damping) const;

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

  /* Evaluates the FEM state of the deformable body with index `id`. */
  const FemStateBase<T>& EvalFemStateBase(const systems::Context<T>& context,
                                          SoftBodyIndex id) const {
    return this->plant()
        .get_cache_entry(fem_state_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

  /* Evaluates the free motion FEM state of the deformable body with index `id`.
   */
  const FemStateBase<T>& EvalFreeMotionFemStateBase(
      const systems::Context<T>& context, SoftBodyIndex id) const {
    return this->plant()
        .get_cache_entry(free_motion_cache_indexes_[id])
        .template Eval<FemStateBase<T>>(context);
  }

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

  /* The deformable models being solved by `this` manager. */
  const DeformableModel<T>* deformable_model_{nullptr};
  /* Cached FEM state quantities. */
  std::vector<systems::CacheIndex> fem_state_cache_indexes_;
  std::vector<systems::CacheIndex> free_motion_cache_indexes_;
  /* Solvers for all deformable bodies. */
  std::vector<std::unique_ptr<FemSolver<T>>> fem_solvers_{};
  std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
      contact_solver_{nullptr};

  /* Geometries temporarily managed by DeformableRigidManager. In the future,
   SceneGraph will manage all the geometries. */
  mutable std::vector<geometry::VolumeMesh<T>> deformable_meshes_{};
  mutable internal::CollisionObjects<T> collision_objects_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager)
