#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/fem/force_density_field_base.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/scoped_name.h"

namespace drake {
namespace multibody {

/** The DeformableBody class represents a single deformable element within a
 MultibodyPlant. It encapsulates the mesh, physical properties, and
 finite-element model required to simulate deformable behavior. It manages:
   - Unique identification (DeformableBodyIndex, DeformableBodyId) and naming
   - Geometry association for collision and visualization
   - Construction of a FEM model with configurable constitutive parameters
   - Storage of reference vertex positions and system state indices
   - Application of boundary conditions and constraints
   - Registration and retrieval of external forces (including gravity)
   - Enabling/disabling of dynamics at runtime

 This class is not meant to be created by end users and it must be created
 exclusively by DeformableModel through DeformableModel::RegisterDeformableBody.
 @tparam_double_only */
template <typename T>
class DeformableBody final : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableBody);

  /** Returns this element's unique index. */
  DeformableBodyIndex index() const {
    return this->template index_impl<DeformableBodyIndex>();
  }

  /** Returns the unique body id. */
  DeformableBodyId body_id() const { return id_; }

  /** Returns the name of the body. */
  const std::string& name() const { return name_; }

  /** Returns scoped name of this body. Neither of the two pieces of the name
   will be empty (the scope name and the element name).
   @throws std::exception if this element is not associated with a
           MultibodyPlant. */
  ScopedName scoped_name() const;

  /** Returns the geometry id of the deformable geometry used to simulate this
   deformable body. */
  geometry::GeometryId geometry_id() const { return geometry_id_; }

  /** Returns physical parameters of this deformable body. */
  const fem::DeformableBodyConfig<T>& config() const { return config_; }

  /** Returns the number of degrees of freedom (DoFs) of this body. */
  int num_dofs() const { return fem_model_->num_dofs(); }

  /** Returns the reference positions of the vertices of the deformable body
   identified by the given `id`. The reference positions are the positions of
   the vertices of the mesh geometry representing the body at registration time,
   measured and expressed in the world frame. The reference positions are
   represented as a VectorX with 3N values where N is the number of vertices.
   The x-, y-, and z-positions (measured and expressed in the world frame) of
   the j-th vertex are 3j, 3j + 1, and 3j + 2 in the VectorX. */
  const VectorX<double>& reference_positions() const {
    return reference_positions_;
  }

  /** Returns the FemModel for this deformable body. */
  const fem::FemModel<T>& fem_model() const { return *fem_model_; }

  /** Returns all the external forces acting on this deformable body. */
  const std::vector<const ForceDensityFieldBase<T>*>& external_forces() const {
    return external_forces_;
  }

  /** Returns the index of the discrete state associated with this deformable
   body in the MultibodyPlant that owns the body. */
  systems::DiscreteStateIndex discrete_state_index() const {
    return discrete_state_index_;
  }

  /** Returns the index of the boolean parameter indicating whether this
   deformable body is enabled. */
  systems::AbstractParameterIndex is_enabled_parameter_index() const {
    return is_enabled_parameter_index_;
  }

  /** Returns the cache index for the FemState of this deformable body. */
  systems::CacheIndex fem_state_cache_index() const {
    return fem_state_cache_index_;
  }

  /** (Internal use only) Configures the parallelism that `this`
   %DeformableBody uses when opportunities for parallel computation arises. */
  void set_parallelism(Parallelism parallelism) {
    fem_model_->set_parallelism(parallelism);
  }

  /** Sets wall boundary conditions for this deformable body. All vertices of
   the mesh of the deformable body whose reference positions are inside the
   prescribed open half space are put under zero displacement boundary
   conditions. The open half space is defined by a plane with outward normal
   n_W. A vertex V is considered to be subject to the boundary condition if n̂ ⋅
   p_QV < 0 where Q is a point on the plane and n̂ is normalized n_W.
   @param[in] p_WQ  The position of a point Q on the plane in the world frame.
   @param[in] n_W   Outward normal to the half space expressed in the world
                    frame.
   @pre n_W.norm() > 1e-10.
   @warning Roundoff error may cause a point very near the defining plane to be
   mischaracterized as to which side of the plane it is on. */
  void SetWallBoundaryCondition(const Vector3<T>& p_WQ,
                                const Vector3<T>& n_W) const;

  /** Defines a fixed constraint between this deformable body and a rigid body
   B. Such a fixed constraint is modeled as distance holonomic constraints:

     p_PᵢQᵢ(q) = 0 for each constrained vertex Pᵢ

   where Pᵢ is the i-th vertex of the deformable body (A) under constraint and
   Qᵢ is a point rigidly affixed to the rigid body B. To specify the constraint,
   we put the reference mesh M of this body A in B's body frame with the given
   pose `X_BA` and prescribe a shape G with pose `X_BG` in B's body frame. All
   vertices Pᵢ in M that are inside (or on the surface of) G are subject to the
   fixed constraints with Qᵢ being coincident with Pᵢ when M is in pose X_BA.
   p_PᵢQᵢ(q) denotes the relative position of point Qᵢ with respect to point Pᵢ
   as a function of the configuration of the model q. Imposing this constraint
   forces Pᵢ and Qᵢ to be coincident for each vertex i of the deformable body
   specified to be under constraint.

   @param[in] body_B       The rigid body under constraint.
   @param[in] X_BA         The pose of this deformable body A's reference mesh
                           in B's body frame.
   @param[in] shape_G      The prescribed geometry shape, attached to rigid body
                           B, used to determine which vertices of this
                           deformable body A is under constraint.
   @param[in] X_BG         The fixed pose of the geometry frame of the given
                           `shape` in body B's frame.
   @returns the unique id of the newly added constraint.
   @throws std::exception unless `body_B` is registered with the same multibody
           tree owning this deformable body.
   @throws std::exception if no constraint is added (i.e. no vertex of the
           deformable body is inside the given `shape` with the given poses).
   @throws std::exception if this element is not associated with a
           MultibodyPlant. */
  MultibodyConstraintId AddFixedConstraint(
      const RigidBody<T>& body_B, const math::RigidTransform<double>& X_BA,
      const geometry::Shape& shape_G, const math::RigidTransform<double>& X_BG);

  /** Returns true if this deformable body is under any fixed constraint. */
  bool has_fixed_constraint() const { return !fixed_constraint_specs_.empty(); }

  /** (Internal use only) Returns a reference to the fixed constraints
   registered with this deformable body. */
  const std::vector<internal::DeformableRigidFixedConstraintSpec>&
  fixed_constraint_specs() const {
    return fixed_constraint_specs_;
  }

  /** Sets the vertex positions of this deformable body in the provided
   `context`.
   @param[in, out] context The context associated with the MultibodyPlant that
                           owns this body.
   @param[in] q            A 3×N matrix of vertex positions.

   @throws std::exception if any of the following conditions are met:
     1. `context` is nullptr.
     2. `context` does not belong to the MultibodyPlant that owns this body.
     3. The number of columns of `q` does not match the number of vertices of
        this body.
     4. `q` contains non-finite values. */
  void SetPositions(systems::Context<T>* context,
                    const Eigen::Ref<const Matrix3X<T>>& q) const;

  /** Sets the vertex velocities of this deformable body in the provided
   `context`.
   @param[in, out] context The context associated with the MultibodyPlant that
                           owns this body.
   @param[in] v            A 3×N matrix of vertex velocities.

   @throws std::exception if any of the following conditions are met:
     1. `context` is nullptr.
     2. `context` does not belong to the MultibodyPlant that owns this body.
     3. The number of columns of `v` does not match the number of vertices of
        this body.
     4. `v` contains non-finite values. */
  void SetVelocities(systems::Context<T>* context,
                     const Eigen::Ref<const Matrix3X<T>>& v) const;

  /** Sets the vertex positions and velocities of this deformable body in the
   provided `context`.
   @param[in, out] context The context associated with the MultibodyPlant that
                           owns this body.
   @param[in] q            A 3×N matrix of vertex positions.
   @param[in] v            A 3×N matrix of vertex velocities.

   @throws std::exception if any of the following conditions are met:
     1. `context` is nullptr.
     2. `context` does not belong to the MultibodyPlant that owns this body.
     3. The number of columns of `q` or `v` does not match the number of
        vertices of this body.
     4. `q` or `v` contains non-finite values. */
  void SetPositionsAndVelocities(systems::Context<T>* context,
                                 const Eigen::Ref<const Matrix3X<T>>& q,
                                 const Eigen::Ref<const Matrix3X<T>>& v) const;

  /** Copies out the matrix of vertex positions for this deformable body in the
   provided `context`.

   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval q          A 3×N matrix containing the positions of all vertices of
                      the body.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Matrix3X<T> GetPositions(const systems::Context<T>& context) const;

  /** Copies out the matrix of vertex velocities for this deformable body in the
   provided `context`.

   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval v          A 3×N matrix containing the velocities of all vertices of
                      the body.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Matrix3X<T> GetVelocities(const systems::Context<T>& context) const;

  /** Copies out the matrix of vertex positions and velocities for this
   deformable body in the provided `context`. The first N columns are the
   positions and the next N columns are the velocities.

   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @return A 3x2N matrix containing the positions and velocities of all
           vertices of the body.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Matrix3X<T> GetPositionsAndVelocities(
      const systems::Context<T>& context) const;

  /** @return true if this deformable body is enabled.
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant that owns this body. */
  bool is_enabled(const systems::Context<T>& context) const;

  /** Disables this deformable body in the given context. Disabling a deformable
   body sets its vertex velocities and accelerations to zero and freezes its
   vertex positions. A disabled deformable body is not subject to any constraint
   (e.g. frictional contact constraint or fixed constraint); it does not move
   under the influence of external forces (e.g. gravity); and it does not
   necessarily satisfy the prescribed boundary condition (if any). On the flip
   side, a disabled deformable body does not affect the dynamics of other
   bodies, even if the collision between the disabled body's geometry and other
   geometries is not filtered. Effectively, the physics of the deformable body
   stop being computed. The deformable body can be enabled by calling Enable().
   Calling Disable() on a body which is already disabled has no effect.
   @see Enable().
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant that owns this body.
   @throw std::exception if context is null. */
  void Disable(systems::Context<T>* context) const;

  /** Enables this deformable body in the given context. Calling Enable() on a
   body which is already enabled has no effect.
   @see Disable().
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant that owns this body.
   @throw std::exception if context is null. */
  void Enable(systems::Context<T>* context) const;

  /** Sets the default pose of the simulated geometry (in its reference
   configuration) in the world frame W.
   @param[in] X_WD The default pose of the simulated geometry in the
   world frame W. */
  void set_default_pose(const math::RigidTransform<double>& X_WD) {
    X_WD_ = X_WD;
  }

  /** Returns the default pose of the simulated geometry (in its reference
   configuration) in the world frame W. This returns pose last set by
   set_default_pose(), or the pose of the geometry in the world frame W when the
   body is registered if set_default_pose() has not been called. */
  const math::RigidTransform<double>& get_default_pose() const { return X_WD_; }

  /** Calculates the body's center of mass position in world frame W.
   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval p_WBcm_W the body's center of mass position, measured and expressed
   in the world frame W.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Vector3<T> CalcCenterOfMassPositionInWorld(
      const systems::Context<T>& context) const;

  /** Calculates the body's center of mass translational velocity in world frame
   W.
   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval v_WScm_W Scm's translational velocity in frame W, expressed in W,
   where Scm is the center of mass of this body.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Vector3<T> CalcCenterOfMassTranslationalVelocityInWorld(
      const systems::Context<T>& context) const;

  /** Using an angular momentum analogy, calculates an "effective" angular
   velocity for this body about its center of mass, measured and expressed in
   the world frame W. The effective angular velocity is computed using an
   angular momentum equation that assumes the body is a rigid body (albeit we
   know it is deformable).

        H_WBcm_W = I_BBcm_W * w_WBcm_W

   for which when solved for w_WBcm_W gives

        w_WBcm_W = inverse(I_BBcm_W) * H_WBcm_W

   where H_WBcm_W is the body's angular momentum about its center of mass Bcm
   measured and expressed in the world frame W.
   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval w_WBcm_W the body's effective angular velocity about Bcm, measured
   and expressed in the world frame W.
   @throws std::exception if `context` does not belong to the MultibodyPlant
   that owns this body. */
  Vector3<T> CalcEffectiveAngularVelocity(
      const systems::Context<T>& context) const;

 private:
  template <typename U>
  friend class DeformableModel;
  template <typename U>
  friend class DeformableBody;

  /* Private constructor exposed only to DeformableModel.
   @param index           Unique DeformableBodyIndex
   @param id              Unique DeformableBodyId
   @param name            Name of the body
   @param geometry_id     GeometryId of the simulated geometry.
   @param model_instance  ModelInstanceIndex for this body.
   @param mesh_G          The simulated volume mesh in the geometry's frame.
   @param X_WG            The pose of the mesh in the world frame.
   @param config          Physical parameters of this body.
   @param weights         The integrator weights for the deformable body used to
                          combine the stiffness, damping, and mass matrices to
                          form the tangent matrix. */
  DeformableBody(DeformableBodyIndex index, DeformableBodyId id,
                 std::string name, geometry::GeometryId geometry_id,
                 ModelInstanceIndex model_instance,
                 const geometry::VolumeMesh<double>& mesh_G,
                 const math::RigidTransform<double>& X_WG,
                 const fem::DeformableBodyConfig<T>& config,
                 const Vector3<double>& weights);

  /* Creates a deep copy of this DeformableBody. Called only in DeformableModel
   to support cloning DeformableModel. */
  std::unique_ptr<DeformableBody<double>> CloneToDouble() const;

  /* Private setter accessible to DeformableModel. */
  void set_discrete_state_index(systems::DiscreteStateIndex index) {
    discrete_state_index_ = index;
  }

  /* Private setter accessible to DeformableModel. */
  void set_is_enabled_parameter_index(systems::AbstractParameterIndex index) {
    is_enabled_parameter_index_ = index;
  }

  /* Private setter accessible to DeformableModel. Sets the external forces
   acting on this body, including the gravity force and other user-defined
   external forces.
   @param[in] external_forces  User-defined external forces shared by all
                               deformable bodies.
   @param[in] gravity          The acceleration of the gravity vector in m/s².
  */
  void SetExternalForces(
      const std::vector<std::unique_ptr<ForceDensityFieldBase<T>>>&
          external_forces,
      const Vector3<T>& gravity);

  /* Builds a FEM model for this body with linear tetrahedral elements and a
   single quadrature point. The reference positions as well as the connectivity
   of the elements are given by `mesh`, and physical properties of the body are
   given by `config`. */
  template <typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModel(const geometry::VolumeMesh<double>& mesh,
                             const fem::DeformableBodyConfig<T>& config,
                             const Vector3<double>& weights);

  /* Helper used by BuildLinearVolumetricModel() to make the proper fem_model_
   instantiation. The model type is inferred from the template argument Model,
   while the number of element subdivisions (as needed at compile time) is
   selected from config.element_subdivision_count(). */
  template <template <class> class Model, typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  SelectSubdivisionAndBuildVolumetricModel(
      const geometry::VolumeMesh<double>& mesh,
      const fem::DeformableBodyConfig<T>& config,
      const Vector3<double>& weights);

  /* Helper called by SelectSubdivisionAndBuildVolumetricModel() to instantiate
   the appropriate fem_model_ based on template parameters Model and num_subd.
   */
  template <template <class> class Model, int num_subd, typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModelHelper(const geometry::VolumeMesh<double>& mesh,
                                   const fem::DeformableBodyConfig<T>& config,
                                   const Vector3<double>& weights);

  void DoSetTopology() final {
    /* No-op because deformable bodies are not part of the MultibodyTree
     topology. */
  }

  void DoDeclareDiscreteState(
      internal::MultibodyTreeSystem<T>* tree_system) final;

  /* Sets the default state of this deformable body. This is called by
   DeformableModel::SetDefaultState. */
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const;

  /* Returns the default positions of the vertices of the deformable body. This
   provides the positions of the registered mesh posed in the default pose,
   measured and expressed in the world frame. */
  VectorX<T> CalcDefaultPositions() const;

  void DoDeclareParameters(internal::MultibodyTreeSystem<T>* tree_system) final;

  void DoDeclareCacheEntries(
      internal::MultibodyTreeSystem<T>* tree_system) final;

  /* Private helper to populate FemState from discrete state values. */
  void CalcFemStateFromDiscreteValues(const systems::Context<T>& context,
                                      fem::FemState<T>* fem_state) const;

  /* NOTE: If a new data member is added to this list, it would need to be
   cloned accordingly in CloneToDouble(). */
  DeformableBodyId id_{};
  std::string name_;
  geometry::GeometryId geometry_id_{};
  /* The mesh of the deformable geometry (in its reference configuration) in
   its geometry frame. */
  geometry::VolumeMesh<double> mesh_G_;
  /* The pose of the deformable geometry (in its reference configuration) at
   registration in the world frame. */
  math::RigidTransform<double> X_WG_;
  /* The default pose of the deformable geometry (in its reference
   configuration) in the world frame. */
  math::RigidTransform<double> X_WD_;
  fem::DeformableBodyConfig<T> config_;
  /* The vertex positions of the deformable body in its reference
   configuration measured and expressed in the world frame. */
  VectorX<double> reference_positions_;
  copyable_unique_ptr<fem::FemModel<T>> fem_model_;
  systems::DiscreteStateIndex discrete_state_index_{};
  systems::CacheIndex fem_state_cache_index_{};
  systems::AbstractParameterIndex is_enabled_parameter_index_{};
  std::vector<internal::DeformableRigidFixedConstraintSpec>
      fixed_constraint_specs_;
  /* External forces and constraints. */
  /* Owned gravity force. */
  copyable_unique_ptr<ForceDensityFieldBase<T>> gravity_force_;
  /* All external forces affecting this body (including the owned gravity). */
  std::vector<const ForceDensityFieldBase<T>*> external_forces_;
};

}  // namespace multibody
}  // namespace drake
