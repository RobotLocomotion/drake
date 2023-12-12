#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/identifier.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/fem/fem_model.h"
#include "drake/multibody/plant/constraint_specs.h"
#include "drake/multibody/plant/deformable_ids.h"
#include "drake/multibody/plant/force_density_field.h"
#include "drake/multibody/plant/physical_model.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

/** DeformableModel implements the interface in PhysicalModel and provides the
 functionalities to specify deformable bodies. Unlike rigid bodies, the shape of
 deformable bodies can change in a simulation. Each deformable body is modeled
 as a volumetric mesh with persisting topology, changing vertex positions, and
 an approximated signed distance field. A finite element model is built for each
 registered deformable body that is used to evaluate the dynamics of the body.
 @experimental
 @tparam_double_only */
template <typename T>
class DeformableModel final : public multibody::PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableModel)

  /** Constructs a DeformableModel to be owned by the given MultibodyPlant.
   @pre plant != nullptr.
   @pre Finalize() has not been called on `plant`. */
  explicit DeformableModel(MultibodyPlant<T>* plant);

  /** Returns the number of deformable bodies registered with this
   DeformableModel. */
  int num_bodies() const { return reference_positions_.size(); }

  // TODO(xuchenhan-tri): Document the minimal requirement on the geometry
  //  instance. For example, it must have a friction proximity property to be
  //  simulated with an MbP that involves contact.
  // TODO(xuchenhan-tri): Consider allowing registering deformable bodies with
  //  non-world frames.
  /** Registers a deformable body in `this` DeformableModel with the given
   GeometryInstance. The body is represented in the world frame and simulated
   with FEM with linear elements and a first order quadrature rule that
   integrates linear functions exactly. See FemModel for details. Returns a
   unique identifier for the added geometry.
   @param[in] geometry_instance  The geometry to be registered with the model.
   @param[in] config             The physical properties of deformable body.
   @param[in] resolution_hint    The parameter that guides the level of mesh
                                 refinement of the deformable geometry. It has
                                 length units (in meters) and roughly
                                 corresponds to a typical edge length in the
                                 resulting mesh for a primitive shape.
   @pre resolution_hint > 0.
   @throws std::exception if Finalize() has been called on the multibody plant
   owning this deformable model. */
  DeformableBodyId RegisterDeformableBody(
      std::unique_ptr<geometry::GeometryInstance> geometry_instance,
      const fem::DeformableBodyConfig<T>& config, double resolution_hint);

  // TODO(xuchenhan-tri): Consider pulling PosedHalfSpace out of internal
  // namespace and use it here.
  /** Sets wall boundary conditions for the body with the given `id`. All
   vertices of the mesh of the deformable body whose reference positions are
   inside the prescribed open half space are put under zero displacement
   boundary conditions. The open half space is defined by a plane with outward
   normal n_W. A vertex V is considered to be subject to the boundary condition
   if n̂ ⋅ p_QV < 0 where Q is a point on the plane and n̂ is normalized n_W.
   @param[in] id    The body to be put under boundary condition.
   @param[in] p_WQ  The position of a point Q on the plane in the world frame.
   @param[in] n_W   Outward normal to the half space expressed in the world
                    frame.
   @pre n_W.norm() > 1e-10.
   @warning Be aware of round-off errors in floating computations when placing a
   vertex very close to the plane defining the half space.
   @throws std::exception if Finalize() has been called on the multibody plant
   owning this deformable model or if no deformable body with the given `id` has
   been registered in this model. */
  void SetWallBoundaryCondition(DeformableBodyId id, const Vector3<T>& p_WQ,
                                const Vector3<T>& n_W);

  /** Defines a fixed constraint between a deformable body A and a rigid body B.
   Such a fixed constraint is modeled as distance holonomic constraints:

     p_PᵢQᵢ(q) = 0 for each constrained vertex Pᵢ

   where Pᵢ is the i-th vertex of the deformable body under constraint and Qᵢ is
   a point rigidly affixed to the rigid body B. To specify the constraint, we
   put the reference mesh M of the deformable body A in B's body frame with the
   given pose `X_BA` and prescribe a shape G with pose `X_BG` in B's body
   frame. All vertices Pᵢ in M that are inside (or on the surface of) G are
   subject to the fixed constraints with Qᵢ being coincident with Pᵢ when M is
   in pose X_BA. p_PᵢQᵢ(q) denotes the relative position of point Qᵢ with
   respect to point Pᵢ as a function of the configuration of the model q.
   Imposing this constraint forces Pᵢ and Qᵢ to be coincident for each vertex i
   of the deformable body specified to be under constraint.

   @param[in] body_A_id    The unique id of the deformable body under the fixed
                           constraint.
   @param[in] body_B       The rigid body under constraint.
   @param[in] X_BA         The pose of deformable body A's reference mesh in B's
                           body frame
   @param[in] shape        The prescribed geometry shape, attached to rigid body
                           B, used to determine which vertices of the deformable
                           body A is under constraint.
   @param[in] X_BG         The fixed pose of the geometry frame of the given
                           `shape` in body B's frame.
   @returns the unique id of the newly added constraint.
   @throws std::exception if no deformable body with the given `body_A_id`
           has been registered.
   @throws std::exception unless `body_B` is registered with the same multibody
           plant owning this deformable model.
   @throws std::exception if shape is not supported by
           QueryObject::ComputeSignedDistanceToPoint(). Currently, supported
           shapes include Box, Capsule, Cylinder, Ellipsoid, HalfSpace, and
           Sphere.
   @throws std::exception if Finalize() has been called on the multibody plant
           owning this deformable model.
   @throws std::exception if no constraint is added (i.e. no vertex of the
           deformable body is inside the given `shape` with the given poses). */
  MultibodyConstraintId AddFixedConstraint(
      DeformableBodyId body_A_id, const RigidBody<T>& body_B,
      const math::RigidTransform<double>& X_BA, const geometry::Shape& shape,
      const math::RigidTransform<double>& X_BG);

  /** Returns the discrete state index of the deformable body identified by the
   given `id`.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet.
   or if no deformable body with the given `id` has been registered in this
   model. */
  systems::DiscreteStateIndex GetDiscreteStateIndex(DeformableBodyId id) const;

  /** Registers an external force density field that applies external force to
   all deformable bodies.
   @throws std::exception if Finalize() has been called on the multibody plant
           owning this deformable model. */
  void AddExternalForce(std::unique_ptr<ForceDensityField<T>> external_force);

  // TODO(xuchenhan-tri): We should allow instrospecting external forces
  // pre-finalize. Currently we add gravity forces at finalize time (instead of
  // immediately after a deformable body is registered) because when gravity is
  // modified via MbP's API, there's no easy way to propagate that information
  // to deformable models.
  /** Returns the force density fields acting on the deformable body with the
   given `id`.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet.
   or if no deformable body with the given `id` has been registered in this
   model. */
  const std::vector<const ForceDensityField<T>*>& GetExternalForces(
      DeformableBodyId id) const;

  /** Returns the FemModel for the body with `id`.
   @throws exception if no deformable body with `id` is registered with `this`
   %DeformableModel. */
  const fem::FemModel<T>& GetFemModel(DeformableBodyId id) const;

  // TODO(xuchenhan-tri): The use of T over double is not well-reasoned.
  //  Consider whether T is really necessary when we support autodiff in
  //  deformable simulations.
  /** Returns the reference positions of the vertices of the deformable body
   identified by the given `id`.
   The reference positions are represented as a VectorX with 3N values where N
   is the number of vertices. The x-, y-, and z-positions (measured and
   expressed in the world frame) of the j-th vertex are 3j, 3j + 1, and 3j + 2
   in the VectorX.
   @throws std::exception if no deformable body with the given `id` has been
   registered in this model. */
  const VectorX<T>& GetReferencePositions(DeformableBodyId id) const;

  /** Returns the DeformableBodyId of the body with the given body index.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet
   or if index is larger than or equal to the total number of registered
   deformable bodies. */
  DeformableBodyId GetBodyId(DeformableBodyIndex index) const;

  /** (Internal) Returns the DeformableBodyIndex of the body with the given id.
   This function is for internal bookkeeping use only. Most users should use
   DeformableBodyId instead.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet
   or if no body with the given `id` has been registered. */
  DeformableBodyIndex GetBodyIndex(DeformableBodyId id) const;

  /** Returns the GeometryId of the geometry associated with the body with the
   given `id`.
   @throws std::exception if no body with the given `id` has been registered. */
  geometry::GeometryId GetGeometryId(DeformableBodyId id) const;

  /** Returns the DeformableBodyId associated with the given `geometry_id`.
   @throws std::exception if the given `geometry_id` does not correspond to a
   deformable body registered with this model. */
  DeformableBodyId GetBodyId(geometry::GeometryId geometry_id) const;

  /** (Internal use only) Returns the true iff the deformable body with the
   given `id` has constraints associated with it. */
  bool HasConstraint(DeformableBodyId id) const {
    return body_id_to_constraint_ids_.count(id) > 0;
  }

  /** (Internal use only) Returns the fixed constraint specification
   corresponding to the given `id`.
   @throws if `id` is not a valid identifier for a fixed constraint. */
  const internal::DeformableRigidFixedConstraintSpec& fixed_constraint_spec(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(fixed_constraint_specs_.count(id) > 0);
    return fixed_constraint_specs_.at(id);
  }

  /** (Internal use only) Returns a reference to the all ids of fixed
   constraints registered with the deformable body with the given `id`. */
  const std::vector<MultibodyConstraintId>& fixed_constraint_ids(
      DeformableBodyId id) const {
    return body_id_to_constraint_ids_.at(id);
  }

  /** Returns the output port of the vertex positions for all registered
   deformable bodies.
   @throws std::exception if MultibodyPlant::Finalize() has not been called yet.
  */
  const systems::OutputPort<T>& vertex_positions_port() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return plant_->get_output_port(vertex_positions_port_index_);
  }

 private:
  PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant() const final {
    return PhysicalModelPointerVariant<T>(this);
  }

  // TODO(xuchenhan-tri): Implement CloneToDouble() and CloneToAutoDiffXd()
  // and the corresponding is_cloneable methods.

  void DoDeclareSystemResources(MultibodyPlant<T>* plant) final;

  /* Builds a FEM model for the body with `id` with linear tetrahedral elements
   and a single quadrature point. The reference positions as well as the
   connectivity of the elements are given by `mesh`, and physical properties
   such as the material model of the body are given by `config`.
   @throws exception if an FEM model corresponding to `id` already exists. */
  void BuildLinearVolumetricModel(DeformableBodyId id,
                                  const geometry::VolumeMesh<double>& mesh,
                                  const fem::DeformableBodyConfig<T>& config);

  template <template <class, int> class Model>
  void BuildLinearVolumetricModelHelper(
      DeformableBodyId id, const geometry::VolumeMesh<double>& mesh,
      const fem::DeformableBodyConfig<T>& config);

  /* Copies the vertex positions of all deformable bodies to the output port
   value which is guaranteed to be of type GeometryConfigurationVector. */
  void CopyVertexPositions(const systems::Context<T>& context,
                           AbstractValue* output) const;

  /* Helper to throw a useful message if a deformable body with the given `id`
   doesn't exist. */
  void ThrowUnlessRegistered(const char* source_method,
                             DeformableBodyId id) const;

  /* The MultibodyPlant that owns `this` DeformableModel. */
  MultibodyPlant<T>* plant_{nullptr};
  /* The positions of each vertex of deformable body at reference configuration.
   */
  std::unordered_map<DeformableBodyId, VectorX<T>> reference_positions_;
  /* The discrete state indexes for all deformable bodies. */
  std::unordered_map<DeformableBodyId, systems::DiscreteStateIndex>
      discrete_state_indexes_;
  std::unordered_map<DeformableBodyId, geometry::GeometryId>
      body_id_to_geometry_id_;
  std::unordered_map<geometry::GeometryId, DeformableBodyId>
      geometry_id_to_body_id_;
  std::unordered_map<DeformableBodyId, std::unique_ptr<fem::FemModel<T>>>
      fem_models_;
  /*The collection all external forces. */
  std::vector<std::unique_ptr<ForceDensityField<T>>> force_densities_;
  /* body_index_to_force_densities_[i] is the collection of pointers to external
   forces applied to body i. */
  std::vector<std::vector<const ForceDensityField<T>*>>
      body_index_to_force_densities_;
  std::unordered_map<DeformableBodyId, std::vector<MultibodyConstraintId>>
      body_id_to_constraint_ids_;
  /* Only used pre-finalize. Empty post-finalize. */
  std::unordered_map<DeformableBodyId, T> body_id_to_density_prefinalize_;
  std::unordered_map<DeformableBodyId, DeformableBodyIndex> body_id_to_index_;
  std::vector<DeformableBodyId> body_ids_;
  std::map<MultibodyConstraintId, internal::DeformableRigidFixedConstraintSpec>
      fixed_constraint_specs_;
  systems::OutputPortIndex vertex_positions_port_index_;
};

}  // namespace multibody
}  // namespace drake
