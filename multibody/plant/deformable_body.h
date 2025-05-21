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
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/deformable_ids.h"
#include "drake/multibody/plant/force_density_field.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {

// TODO(xuchenhan-tri): Derive from MultibodyElement.
template <typename T>
class DeformableBody {
 public:
  /** Returns this element's unique index. */
  DeformableBodyIndex index() const { return index_; }

  /** Returns the unique body id. */
  DeformableBodyId body_id() const { return id_; }

  /** Returns the name of the body. */
  const std::string& name() const { return name_; }

  /** Returns the geometry id of the deformable geometry used to simulate this
   deformable body. */
  geometry::GeometryId geometry_id() const { return geometry_id_; }

  /** Returns the model instance index of this deformable body. */
  ModelInstanceIndex model_instance() const { return model_instance_; }

  /** Returns physical parameters of this deformable body. */
  const fem::DeformableBodyConfig<T>& config() const { return config_; }

  /** Returns the number of degrees of freedom (DoFs) of this body. */
  int num_dofs() const { return fem_model_->num_dofs(); }

  /** Returns the reference positions of the vertices of the deformable body
   identified by the given `id`.
   The reference positions are represented as a VectorX with 3N values where N
   is the number of vertices. The x-, y-, and z-positions (measured and
   expressed in the world frame) of the j-th vertex are 3j, 3j + 1, and 3j + 2
   in the VectorX. */
  const VectorX<double>& reference_positions() const {
    return reference_positions_;
  }

  /** Returns the FemModel for this deformable body. */
  const fem::FemModel<T>& fem_model() const { return *fem_model_; }

  /** Returns all the external forces acting on this deformable body. */
  const std::vector<const ForceDensityField<T>*>& external_forces() const {
    return external_forces_;
  }

  systems::DiscreteStateIndex discrete_state_index() const {
    return discrete_state_index_;
  }

  systems::AbstractParameterIndex is_enabled_parameter_index() const {
    return is_enabled_parameter_index_;
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
   @warning Be aware of round-off errors in floating computations when placing a
   vertex very close to the plane defining the half space. */
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
   @param[in] shape        The prescribed geometry shape, attached to rigid body
                           B, used to determine which vertices of this
                           deformable body A is under constraint.
   @param[in] X_BG         The fixed pose of the geometry frame of the given
                           `shape` in body B's frame.
   @returns the unique id of the newly added constraint.
   @throws std::exception unless `body_B` is registered with the same multibody
           plant owning this deformable model.
   @throws std::exception if Finalize() has been called on the multibody plant
           owning this deformable body.
   @throws std::exception if no constraint is added (i.e. no vertex of the
           deformable body is inside the given `shape` with the given poses). */
  MultibodyConstraintId AddFixedConstraint(
      const RigidBody<T>& body_B, const math::RigidTransform<double>& X_BA,
      const geometry::Shape& shape, const math::RigidTransform<double>& X_BG);

  /** Returns true if and only if this deformable body is under any fixed
   * constraint. */
  bool has_fixed_constraint() const { return !fixed_constraint_specs_.empty(); }

  /** (Internal use only) Returns a reference to the all fixed constraints
   registered with the deformable body with the given `id`. */
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
     4. `q` contains non-finite values.
     5. `Finalize()` has not been called on the MultibodyPlant that owns this
        body. */
  void SetPositions(systems::Context<T>* context,
                    const Eigen::Ref<const Matrix3X<T>>& q) const;

  /** Returns the matrix of vertex positions for this deformable body in the
   provided `context`.

   @param[in] context The context associated with the MultibodyPlant that owns
                      this body.
   @retval q          A 3×N matrix containing the positions of all vertices of
                      the body.
   @throws std::exception if any of the following conditions are met:
     1. `context` does not belong to the MultibodyPlant that owns this body.
     2. `Finalize()` has not been called on the MultibodyPlant that owns this
        body. */
  Matrix3X<T> GetPositions(const systems::Context<T>& context) const;

  /** @return true if and only if this deformable body is enabled.
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

 private:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DeformableBody)

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
   @param plant           Pointer to the MultibodyPlant that owns this
                          DeformableBody.
   @pre `plant` is not nullptr.
   @pre `plant` is not finalized. */
  DeformableBody(DeformableBodyIndex index, DeformableBodyId id,
                 std::string name, geometry::GeometryId geometry_id,
                 ModelInstanceIndex model_instance,
                 const geometry::VolumeMesh<double>& mesh_G,
                 const math::RigidTransform<double>& X_WG,
                 const fem::DeformableBodyConfig<T>& config,
                 const MultibodyPlant<T>* plant);

  std::unique_ptr<DeformableBody<double>> CloneToDouble(
      const MultibodyPlant<double>* plant) const {
    if constexpr (!std::is_same_v<T, double>) {
      /* A none double body shouldn't exist in the first place. */
      DRAKE_UNREACHABLE();
    } else {
      auto clone = std::unique_ptr<DeformableBody<double>>(
          new DeformableBody<double>(*this));
      clone->plant_ = plant;
      return clone;
    }
  }

  /* Private setter accessible to DeformableModel. */
  void set_discrete_state_index(systems::DiscreteStateIndex index) {
    discrete_state_index_ = index;
  }

  /* Private setter accessible to DeformableModel. */
  void set_is_enabled_parameter_index(systems::AbstractParameterIndex index) {
    is_enabled_parameter_index_ = index;
  }

  /* Private setter accessible to DeformableModel.
   @param[in] external_forces  External forces shared by all deformable bodies.
   @param[in] gravity          The acceleration of the gravity vector in m/s².
  */
  void SetExternalForces(
      const std::vector<std::unique_ptr<ForceDensityField<T>>>& external_forces,
      const Vector3<T>& gravity);

  /* Builds a FEM model for this body with linear tetrahedral elements and a
   single quadrature point. The reference positions as well as the connectivity
   of the elements are given by `mesh`, and physical properties of the body are
   given by `config`. */
  template <typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModel(const geometry::VolumeMesh<double>& mesh,
                             const fem::DeformableBodyConfig<T>& config);

  /* Helper for BuildLinearVolumetricModel templated on constitutive model. */
  template <template <class> class Model, typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModelHelper(const geometry::VolumeMesh<double>& mesh,
                                   const fem::DeformableBodyConfig<T>& config);

  DeformableBodyIndex index_{};
  DeformableBodyId id_{};
  std::string name_;
  geometry::GeometryId geometry_id_{};
  ModelInstanceIndex model_instance_{};
  /* The mesh of the deformable geometry (in its reference configuration) in its
   geometry frame. */
  geometry::VolumeMesh<double> mesh_G_;
  /* The pose of the deformable geometry (in its reference configuration) in the
   world frame. */
  math::RigidTransform<double> X_WG_;
  fem::DeformableBodyConfig<T> config_;
  const MultibodyPlant<T>* plant_{};
  VectorX<double> reference_positions_;
  copyable_unique_ptr<fem::FemModel<T>> fem_model_;
  systems::DiscreteStateIndex discrete_state_index_{};
  systems::AbstractParameterIndex is_enabled_parameter_index_{};
  std::vector<internal::DeformableRigidFixedConstraintSpec>
      fixed_constraint_specs_;
  /* External forces and constraints. */
  /* Owned gravity force. */
  copyable_unique_ptr<ForceDensityField<T>> gravity_force_;
  /* All external forces affecting this body (including the owned gravity). */
  std::vector<const ForceDensityField<T>*> external_forces_;
};

}  // namespace multibody
}  // namespace drake
