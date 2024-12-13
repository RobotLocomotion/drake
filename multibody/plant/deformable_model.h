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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableModel);

  // TODO(xuchenhan-tri): The prerequisite isn't very precise. It's ok to call
  // the constructor in the middle of finalizing a plant, as long as this
  // DeformableModel has a chance to declare the system resources it needs.
  // Consider making the constructor private and only allow construction via
  // plant.AddDeformableModel().
  /** (Internal only) Constructs a DeformableModel to be owned by the given
   MultibodyPlant. This constructor is only intended to be called internally by
   %MultibodyPlant.
   @pre plant != nullptr.
   @pre Finalize() has not been called on `plant`. */
  explicit DeformableModel(MultibodyPlant<T>* plant);

  ~DeformableModel() final;

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
   @throws std::exception if `this` %DeformableModel is not of scalar type
   double.
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
   @throws std::exception if `this` %DeformableModel is not of scalar type
           double.
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
   @throws std::exception if `this` %DeformableModel is not of scalar type
           double.
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

  // TODO(xuchenhan-tri): filter collisions for the disabled deformable body's
  // geometry.
  /** Disables the deformable body with the given `id` in the given context.
   Disabling a deformable body sets its vertex velocities and accelerations to
   zero and freezes its vertex positions. A disabled deformable body is not
   subject to any constraint (e.g. frictional contact constraint or fixed
   constraint); it does not move under the influence of external forces
   (e.g. gravity); and it does not necessarily satisfy the prescribed boundary
   condition (if any). On the flip side, a disabled deformable body does not
   affect the dynamics of other bodies, even if the collision between the
   disabled body's geometry and other geometries is not filtered. Effectively,
   the physics of the deformable body stop being computed. The deformable body
   can be enabled by calling Enable(). Calling Disable() on a body which is
   already disabled has no effect.
   @see Enable().
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant associated with this %DeformableModel.
   @throw std::exception if a deformable body with the given id is not
   registered.
   @throw std::exception if context is null. */
  void Disable(DeformableBodyId id, systems::Context<T>* context) const;

  /** Enables the deformable body with the given `id` in the given context.
   Calling Enable() on a body which is already enabled has no effect.
   @see Disable().
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant associated with this %DeformableModel.
   @throw std::exception if a deformable body with the given id is not
   registered.
   @throw std::exception if context is null. */
  void Enable(DeformableBodyId id, systems::Context<T>* context) const;

  /** @return true if and only if the deformable body with the given id is
   enabled.
   @throw std::exception if the passed in context isn't compatible with the
   MultibodyPlant associated with this %DeformableModel.
   @throw std::exception if a deformable body with the given id is not
   registered. */
  bool is_enabled(DeformableBodyId id,
                  const systems::Context<T>& context) const {
    ThrowUnlessRegistered(__func__, id);
    this->plant().ValidateContext(context);
    return context.get_parameters().template get_abstract_parameter<bool>(
        is_enabled_parameter_indexes_.at(id));
  }

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
    return body_id_to_constraint_ids_.contains(id);
  }

  /** (Internal use only) Returns the fixed constraint specification
   corresponding to the given `id`.
   @throws if `id` is not a valid identifier for a fixed constraint. */
  const internal::DeformableRigidFixedConstraintSpec& fixed_constraint_spec(
      MultibodyConstraintId id) const {
    DRAKE_THROW_UNLESS(fixed_constraint_specs_.contains(id));
    return fixed_constraint_specs_.at(id);
  }

  /** (Internal use only) Returns a reference to the all ids of fixed
   constraints registered with the deformable body with the given `id`. */
  const std::vector<MultibodyConstraintId>& fixed_constraint_ids(
      DeformableBodyId id) const {
    return body_id_to_constraint_ids_.at(id);
  }

  /** Returns the output port index of the vertex positions port for all
   registered deformable bodies.
   @throws std::exception if called before `DeclareSceneGraphPorts()` is called.
  */
  systems::OutputPortIndex configuration_output_port_index() const {
    DRAKE_DEMAND(configuration_output_port_index_.is_valid());
    return configuration_output_port_index_;
  }

  /** Returns true if there's no deformable body or external force registered to
   `this` %DeformableModel. */
  bool is_empty() const {
    return body_ids_.empty() && force_densities_.empty();
  }

  bool is_cloneable_to_double() const final { return true; }

  /** Returns true if and only if this %DeformableModel is empty. */
  bool is_cloneable_to_autodiff() const final { return is_empty(); }

  /** Returns true if and only if this %DeformableModel is empty. */
  bool is_cloneable_to_symbolic() const final { return is_empty(); }

 private:
  /* Allow different specializations to access each other's private data for
   scalar conversion. */
  template <typename U>
  friend class DeformableModel;

  PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant() const final {
    return PhysicalModelPointerVariant<T>(this);
  }

  std::unique_ptr<PhysicalModel<double>> CloneToDouble(
      MultibodyPlant<double>* plant) const final;

  /* Since %DeformableModel is only cloneable to AutoDiffXd if the model is
   empty, the clone simply returns an empty model. */
  std::unique_ptr<PhysicalModel<AutoDiffXd>> CloneToAutoDiffXd(
      MultibodyPlant<AutoDiffXd>* plant) const final;

  /* Since %DeformableModel is only cloneable to symbolic if the model is
   empty, the clone simply returns an empty model. */
  std::unique_ptr<PhysicalModel<symbolic::Expression>> CloneToSymbolic(
      MultibodyPlant<symbolic::Expression>* plant) const final;

  void DoDeclareSystemResources() final;

  void DoDeclareSceneGraphPorts() final;

  /* Builds a FEM model for the body with `id` with linear tetrahedral elements
   and a single quadrature point. The reference positions as well as the
   connectivity of the elements are given by `mesh`, and physical properties
   such as the material model of the body are given by `config`.
   @throws exception if an FEM model corresponding to `id` already exists. */
  template <typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModel(DeformableBodyId id,
                             const geometry::VolumeMesh<double>& mesh,
                             const fem::DeformableBodyConfig<T>& config);

  template <template <class> class Model, typename T1 = T>
  typename std::enable_if_t<std::is_same_v<T1, double>, void>
  BuildLinearVolumetricModelHelper(DeformableBodyId id,
                                   const geometry::VolumeMesh<double>& mesh,
                                   const fem::DeformableBodyConfig<T>& config);

  /* Copies the vertex positions of all deformable bodies to the output port
   value which is guaranteed to be of type GeometryConfigurationVector. */
  void CopyVertexPositions(const systems::Context<T>& context,
                           AbstractValue* output) const;

  /* Helper to throw a useful message if a deformable body with the given `id`
   doesn't exist. */
  void ThrowUnlessRegistered(const char* function_name,
                             DeformableBodyId id) const;

  /* Helper to throw a useful message if the given `function_name` is called on
   a DeformableModel that doesn't have scalar type double. */
  void ThrowIfNotDouble(const char* function_name) const;

  /* Data members. WARNING: if you add a field here be sure to update
   CloneToDouble() to make sure all fields are copied. */
  /* The positions of each vertex of deformable body at reference configuration.
   */
  std::unordered_map<DeformableBodyId, VectorX<T>> reference_positions_;
  /* The discrete state indexes for all deformable bodies. */
  std::unordered_map<DeformableBodyId, systems::DiscreteStateIndex>
      discrete_state_indexes_;
  /* System parameter index for the enable states for each deformable body. */
  std::unordered_map<DeformableBodyId, systems::AbstractParameterIndex>
      is_enabled_parameter_indexes_;
  std::unordered_map<DeformableBodyId, geometry::GeometryId>
      body_id_to_geometry_id_;
  std::unordered_map<geometry::GeometryId, DeformableBodyId>
      geometry_id_to_body_id_;
  std::unordered_map<DeformableBodyId, std::unique_ptr<fem::FemModel<T>>>
      fem_models_;
  /* The collection all external forces. */
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
  systems::OutputPortIndex configuration_output_port_index_;
};

}  // namespace multibody
}  // namespace drake
