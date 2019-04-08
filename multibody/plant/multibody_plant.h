#pragma once

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/random.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/contact_jacobians.h"
#include "drake/multibody/plant/contact_results.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/implicit_stribeck_solver.h"
#include "drake/multibody/plant/implicit_stribeck_solver_results.h"
#include "drake/multibody/tree/force_element.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/weld_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {

/// @cond
// Helper macro to throw an exception within methods that should not be called
// post-finalize.
#define DRAKE_MBP_THROW_IF_FINALIZED() ThrowIfFinalized(__func__)

// Helper macro to throw an exception within methods that should not be called
// pre-finalize.
#define DRAKE_MBP_THROW_IF_NOT_FINALIZED() ThrowIfNotFinalized(__func__)
/// @endcond

/// %MultibodyPlant is a Drake system framework representation (see
/// systems::System) for the model of a physical system consisting of a
/// collection of interconnected bodies.  See @ref multibody for an overview of
/// concepts/notation.
///
/// %MultibodyPlant provides a user-facing API to:
///
/// - add bodies, joints, force elements, and constraints,
/// - register geometries to a provided SceneGraph instance,
/// - create and manipulate its Context,
/// - perform Context-dependent computational queries.
///
/// @section equations_of_motion System dynamics
///
/// @cond
/// TODO(amcastro-tri): Update this documentation to include:
///   - Input actuation and ports and connection to the B matrix.
///   - Externally applied forces and ports to apply them.
///   - Bilateral constraints.
///   - Unilateral constraints and contact.
/// @endcond
///
/// The state of a multibody system `x = [q; v]` is given by its generalized
/// positions vector q, of size `nq` (see num_positions()), and by its
/// generalized velocities vector v, of size `nv` (see num_velocities()).
/// As a Drake System, %MultibodyPlant implements the governing equations for a
/// multibody dynamical system in the form `ẋ = f(t, x, u)` with t being the
/// time and u the input vector of actuation forces. The governing equations for
/// the dynamics of a multibody system modeled with %MultibodyPlant are
/// [Featherstone 2008, Jain 2010]: <pre>
///          q̇ = N(q)v
///   (1)    M(q)v̇ + C(q, v)v = tau
/// </pre>
/// where `M(q)` is the mass matrix of the multibody system, `C(q, v)v`
/// corresponds to the bias term containing Coriolis and gyroscopic effects and
/// `N(q)` is the kinematic coupling matrix describing the relationship between
/// the rate of change of the generalized coordinates and the generalized
/// velocities, [Seth 2010]. N(q) is an `nq x nv` matrix.
/// The vector `tau ∈ ℝⁿᵛ` on the right hand side of Eq. (1) corresponds to
/// generalized forces applied on the system. These can include externally
/// applied body forces, constraint forces, and contact forces.
///
/// @section sdf_loading Loading models from SDF files
///
/// Drake has the capability of loading multibody models from SDF and URDF
/// files.  Consider the example below which loads an acrobot model:
/// @code
///   MultibodyPlant<T> acrobot;
///   SceneGraph<T> scene_graph;
///   Parser parser(&acrobot, &scene_graph);
///   const std::string relative_name =
///     "drake/multibody/benchmarks/acrobot/acrobot.sdf";
///   const std::string full_name = FindResourceOrThrow(relative_name);
///   parser.AddModelFromFile(full_name);
/// @endcode
/// As in the example above, for models including visual geometry, collision
/// geometry or both, the user must specify a SceneGraph for geometry handling.
/// You can find a full example of the LQR controlled acrobot in
/// examples/multibody/acrobot/run_lqr.cc.
///
/// AddModelFromFile() can be invoked multiple times on the same plant in order
/// to load multiple model instances.  Other methods are available on Parser
/// such as AddAllModelsFromFile() which allows creating model instances per
/// each `<model>` tag found in the file. Please refer to each of these
/// methods' documentation for further details.
///
/// @section adding_elements Adding modeling elements
///
/// @cond
/// TODO(amcastro-tri): Update this section to add force elements and
/// constraints.
/// @endcond
///
/// Clients of a %MultibodyPlant can add multibody elements with the following
/// methods:
///
/// - Bodies: AddRigidBody().
/// - Joints: AddJoint().
///
/// All modeling elements **must** be added pre-finalize.
///
/// @section geometry_registration Registering geometry with a SceneGraph
///
/// %MultibodyPlant users can register geometry with a SceneGraph for
/// essentially two purposes; a) visualization and, b) contact modeling.
/// @cond
/// // TODO(SeanCurtis-TRI): update this comment as the number of SceneGraph
/// // roles changes.
/// @endcond
/// Before any geometry registration takes place, a user **must** first make a
/// call to RegisterAsSourceForSceneGraph() in order to register the
/// %MultibodyPlant as a client of a SceneGraph instance, point at which the
/// plant will have assigned a valid geometry::SourceId.
/// At Finalize(), %MultibodyPlant will declare input/output ports as
/// appropriate to communicate with the SceneGraph instance on which
/// registrations took place. All geometry registration **must** be performed
/// pre-finalize.
///
/// If %MultibodyPlant registers geometry with a SceneGraph via calls to
/// RegisterCollisionGeometry(), an input port for geometric queries will be
/// declared at Finalize() time, see get_geometry_query_input_port(). Users must
/// connect this input port to the output port for geometric queries of the
/// SceneGraph used for registration, which can be obtained with
/// SceneGraph::get_query_output_port().
/// In summary, if %MultibodyPlant registers collision geometry, the setup
/// process will include:
///
/// 1. Call to RegisterAsSourceForSceneGraph().
/// 2. Calls to RegisterCollisionGeometry(), as many as needed.
/// 3. Call to Finalize(), user is done specifying the model.
/// 4. Connect SceneGraph::get_query_output_port() to
///    get_geometry_query_input_port().
///
/// Refer to the documentation provided in each of the methods above for further
/// details.
///
/// @section finalize_stage Finalize() stage
///
/// Once the user is done adding modeling elements and registering geometry, a
/// call to Finalize() must be performed. This call will:
///
/// - Build the underlying MultibodyTree topology, see MultibodyTree::Finalize()
///   for details,
/// - declare the plant's state,
/// - declare the plant's input and output ports,
/// - declare input and output ports for communication with a SceneGraph.
///
/// @cond
/// TODO(amcastro-tri): Consider making the actual geometry registration with GS
/// AFTER Finalize() so that we can tell if there are any bodies welded to the
/// world to which we could just assign anchored geometry instead of dynamic
/// geometry. This is an optimization and the API, and pre/post-finalize
/// conditions should not change.
/// @endcond
///
/// <h3> References </h3>
///
/// - [Featherstone 2008] Featherstone, R., 2008.
///     Rigid body dynamics algorithms. Springer.
/// - [Jain 2010] Jain, A., 2010.
///     Robot and multibody dynamics: analysis and algorithms.
///     Springer Science & Business Media.
/// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
///     Minimal formulation of joint motion for biomechanisms.
///     Nonlinear dynamics, 62(1), pp.291-303.
///
/// @tparam T Must be one of drake's default scalar types.
///
/// @ingroup systems
template <typename T>
class MultibodyPlant : public internal::MultibodyTreeSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)

  /// Default constructor creates a plant with a single "world" body.
  /// Therefore, right after creation, num_bodies() returns one.
  /// @param[in] time_step
  ///   An optional parameter indicating whether `this` plant is modeled as a
  ///   continuous system (`time_step = 0`) or as a discrete system with
  ///   periodic updates of period `time_step > 0`. @default 0.0.
  /// @throws std::exception if `time_step` is negative.
  explicit MultibodyPlant(double time_step = 0);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  MultibodyPlant(const MultibodyPlant<U>& other)
      : internal::MultibodyTreeSystem<T>(
            systems::SystemTypeTag<multibody::MultibodyPlant>{},
            other.internal_tree().template CloneToScalar<T>(),
            other.is_discrete()) {
    DRAKE_THROW_UNLESS(other.is_finalized());
    time_step_ = other.time_step_;
    // Copy of all members related with geometry registration.
    source_id_ = other.source_id_;
    body_index_to_frame_id_ = other.body_index_to_frame_id_;
    frame_id_to_body_index_ = other.frame_id_to_body_index_;
    geometry_id_to_body_index_ = other.geometry_id_to_body_index_;
    geometry_id_to_visual_index_ = other.geometry_id_to_visual_index_;
    geometry_id_to_collision_index_ = other.geometry_id_to_collision_index_;
    default_coulomb_friction_ = other.default_coulomb_friction_;
    visual_geometries_ = other.visual_geometries_;
    collision_geometries_ = other.collision_geometries_;
    if (geometry_source_is_registered())
      DeclareSceneGraphPorts();

    // MultibodyTree::CloneToScalar() already called MultibodyTree::Finalize()
    // on the new MultibodyTree on U. Therefore we only Finalize the plant's
    // internals (and not the MultibodyTree).
    FinalizePlantOnly();
  }

  /// Returns the number of Frame objects in this model.
  /// Frames include body frames associated with each of the bodies,
  /// including the _world_ body. This means the minimum number of frames is
  /// one.
  int num_frames() const {
    return internal_tree().num_frames();
  }

  /// Returns the number of bodies in the model, including the "world" body,
  /// which is always part of the model.
  /// @see AddRigidBody().
  int num_bodies() const {
    return internal_tree().num_bodies();
  }

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const {
    return internal_tree().num_joints();
  }

  /// Returns the number of joint actuators in the model.
  /// @see AddJointActuator().
  int num_actuators() const {
    return internal_tree().num_actuators();
  }

  /// Returns the number of ForceElement objects.
  /// @see AddForceElement().
  int num_force_elements() const {
    return internal_tree().num_force_elements();
  }

  /// Returns the number of model instances in the model.
  /// @see AddModelInstance().
  int num_model_instances() const {
    return internal_tree().num_model_instances();
  }

  /// Returns the size of the generalized position vector `q` for this model.
  int num_positions() const { return internal_tree().num_positions(); }

  /// Returns the size of the generalized position vector `q` for a specific
  /// model instance.
  int num_positions(ModelInstanceIndex model_instance) const {
    return internal_tree().num_positions(model_instance);
  }

  /// Returns the size of the generalized velocity vector `v` for this model.
  int num_velocities() const { return internal_tree().num_velocities(); }

  /// Returns the size of the generalized velocity vector `v` for a specific
  /// model instance.
  int num_velocities(ModelInstanceIndex model_instance) const {
    return internal_tree().num_velocities(model_instance);
  }

  // N.B. The state in the Context may at some point contain values such as
  // integrated power and other discrete states, hence the specific name.
  /// Returns the size of the multibody system state vector `x = [q; v]`. This
  /// will be num_positions() plus num_velocities().
  int num_multibody_states() const { return internal_tree().num_states(); }

  /// Returns the total number of actuated degrees of freedom.
  /// That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs() const { return internal_tree().num_actuated_dofs(); }

  /// Returns the total number of actuated degrees of freedom for a specific
  /// model instance.  That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs(ModelInstanceIndex model_instance) const {
    return internal_tree().num_actuated_dofs(model_instance);
  }

  /// @name Position and velocity state component accessors and mutators.
  /// Various methods for accessing and mutating `[q; v]`, where `q` is the
  /// vector of generalized positions and `v` is the vector of generalized
  /// velocities, or some portion thereof (e.g., only `v`).
  /// @{

  /// Returns a const vector reference containing the vector
  /// `[q; v]` with `q` the vector of generalized positions and
  /// `v` the vector of generalized velocities.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositionsAndVelocities(
      const systems::Context<T>& context) const {
    return internal_tree().GetPositionsAndVelocities(context);
  }

  /// Returns the vector `[q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities for model instance `model_instance`.
  /// @throws std::exception if the `context` does not correspond to the context
  /// for a multibody model or `model_instance` is invalid.
  /// @note returns a dense vector of dimension `q.size() + v.size()` associated
  ///          with `model_instance` in O(`q.size()`) time.
  VectorX<T> GetPositionsAndVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetPositionsAndVelocities(context, model_instance);
  }

  /// (Advanced) Returns a mutable vector containing the vector `[q; v]`
  /// of the model with `q` the vector of generalized positions and `v` the
  /// vector of generalized velocities (**see warning**).
  /// @warning You should use SetPositionsAndVelocities() instead of this method
  ///          unless you are fully aware of the interactions with the caching
  ///          mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositionsAndVelocities(
      systems::Context<T>* context) const {
    return internal_tree().GetMutablePositionsAndVelocities(context);
  }

  /// Gets the pose of a given `body` in the world frame W.
  /// @note In general getting the pose of a body in the model would involve
  /// solving the kinematics. This method allows us to simplify this process
  /// when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  math::RigidTransform<T> GetFreeBodyPose(const systems::Context<T>& context,
                                          const Body<T>& body) const {
    return internal_tree().GetFreeBodyPoseOrThrow(context, body);
  }

  /// Sets `context` to store the pose `X_WB` of a given `body` B in the world
  /// frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyPose(systems::Context<T>* context, const Body<T>& body,
                       const math::RigidTransform<T>& X_WB) const {
    internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context);
  }

  /// Sets `state` to store the pose `X_WB` of a given `body` B in the world
  /// frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  /// @pre `state` comes from this MultibodyPlant.
  void SetFreeBodyPose(
      const systems::Context<T>& context, systems::State<T>* state,
      const Body<T>& body, const math::RigidTransform<T>& X_WB) const {
    CheckValidState(state);
    internal_tree().SetFreeBodyPoseOrThrow(body, X_WB, context, state);
  }

  /// Sets `context` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodySpatialVelocity(
      systems::Context<T>* context, const Body<T>& body,
      const SpatialVelocity<T>& V_WB) const {
    internal_tree().SetFreeBodySpatialVelocityOrThrow(body, V_WB, context);
  }

  /// Sets `state` to store the spatial velocity `V_WB` of a given `body` B in
  /// the world frame W, for a given `context` of `this` model.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  /// @pre `state` comes from this MultibodyPlant.
  void SetFreeBodySpatialVelocity(
      const systems::Context<T>& context, systems::State<T>* state,
      const Body<T>& body, const SpatialVelocity<T>& V_WB) const {
    CheckValidState(state);
    internal_tree().SetFreeBodySpatialVelocityOrThrow(
        body, V_WB, context, state);
  }

  // TODO(sammy-tri) We should also be able to set the default pose of a free
  // body.  See https://github.com/RobotLocomotion/drake/issues/10713

  /// Sets the distribution used by SetRandomState() to populate the
  /// x-y-z `position` component of the floating-base state.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomPositionDistribution(
      const Body<T>& body, const Vector3<symbolic::Expression>& position) {
    this->mutable_tree().SetFreeBodyRandomPositionDistributionOrThrow(body,
                                                                      position);
  }

  /// Sets the distribution used by SetRandomState() to populate the
  /// rotation component of the floating-base state.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomRotationDistribution(
      const Body<T>& body,
      const Eigen::Quaternion<symbolic::Expression>& rotation) {
    this->mutable_tree().SetFreeBodyRandomRotationDistributionOrThrow(
        body, rotation);
  }

  /// Sets the distribution used by SetRandomState() to populate the
  /// rotation component of the floating-base state using uniformly random
  /// rotations.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::exception if called pre-finalize.
  void SetFreeBodyRandomRotationDistributionToUniform(const Body<T>& body);

  /// Sets all generalized positions and velocities from the given vector
  /// [q; v].
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `q_v` is not equal to `num_positions() + num_velocities()`.
  void SetPositionsAndVelocities(
      systems::Context<T>* context, const VectorX<T>& q_v) const {
    DRAKE_DEMAND(q_v.size() == (num_positions() + num_velocities()));
    internal_tree().GetMutablePositionsAndVelocities(context) = q_v;
  }

  /// Sets generalized positions and velocities from the given vector
  /// [q; v] for the specified model instance.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_v` is not equal to
  /// `num_positions(model_instance) + num_velocities(model_instance)`.
  void SetPositionsAndVelocities(
      systems::Context<T>* context, ModelInstanceIndex model_instance,
      const VectorX<T>& q_v) const {
    internal_tree().SetPositionsAndVelocities(model_instance, q_v, context);
  }

  /// Returns a const vector reference containing the vector of
  /// generalized positions.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<const VectorX<T>> GetPositions(
      const systems::Context<T>& context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetPositionsAndVelocities() as a VectorX<T> so that we can
    // call head() on it.
    return GetPositionsAndVelocities(context).nestedExpression().head(
        num_positions());
  }

  /// Returns an vector containing the generalized positions (`q`) for the
  /// given model instance.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  /// @note returns a dense vector of dimension `q.size()` associated with
  ///          `model_instance` in O(`q.size()`) time.
  VectorX<T> GetPositions(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetPositionsFromArray(
        model_instance, GetPositions(context));
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized positions (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetPositions() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or if it does not
  /// correspond to the context for a multibody model.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositions(
      systems::Context<T>* context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call head() on it.
    return internal_tree().GetMutablePositionsAndVelocities(context)
        .nestedExpression().head(num_positions());
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized positions (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetPositions() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `state` is nullptr or if the context does
  ///         not correspond to the context for a multibody model.
  /// @pre `state` comes from this MultibodyPlant.
  Eigen::VectorBlock<VectorX<T>> GetMutablePositions(
      const systems::Context<T>& context, systems::State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidState(state));
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call head() on it.
    return internal_tree()
        .GetMutablePositionsAndVelocities(context, state)
        .nestedExpression()
        .head(num_positions());
  }

  /// Sets all generalized positions from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `q` is not equal to `num_positions()`.
  void SetPositions(systems::Context<T>* context, const VectorX<T>& q) const {
    GetMutablePositions(context) = q;
  }

  /// Sets the positions for a particular model instance from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`.
  void SetPositions(
      systems::Context<T>* context,
      ModelInstanceIndex model_instance, const VectorX<T>& q_instance) const {
    Eigen::VectorBlock<VectorX<T>> q = GetMutablePositions(context);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// Sets the positions for a particular model instance from the given vector.
  /// @throws std::exception if the `state` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `q_instance` is not equal to
  /// `num_positions(model_instance)`.
  /// @pre `state` comes from this MultibodyPlant.
  void SetPositions(const systems::Context<T>& context,
                    systems::State<T>* state, ModelInstanceIndex model_instance,
                    const VectorX<T>& q_instance) const {
    CheckValidState(state);
    Eigen::VectorBlock<VectorX<T>> q = GetMutablePositions(context, state);
    internal_tree().SetPositionsInArray(model_instance, q_instance, &q);
  }

  /// Returns a const vector reference containing the generalized velocities.
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  Eigen::VectorBlock<const VectorX<T>> GetVelocities(
      const systems::Context<T>& context) const {
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetPositionsAndVelocities() as a VectorX<T> so that we can
    // call tail() on it.
    return GetPositionsAndVelocities(context).nestedExpression().tail(
        num_velocities());
  }

  /// Returns a vector containing the generalized velocities (`v`) for
  /// the given model instance.
  /// @throws std::exception if the `context` does not
  /// correspond to the context for a multibody model.
  /// @note returns a dense vector of dimension `v.size()` associated with
  ///          `model_instance` in O(`v.size()`) time.
  VectorX<T> GetVelocities(
      const systems::Context<T>& context,
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetVelocitiesFromArray(
        model_instance, GetVelocities(context));
  }

  /// (Advanced) Returns a mutable vector reference containing the vector
  /// of generalized velocities (**see warning**).
  /// @note This method returns a reference to existing data, exhibits constant
  ///       i.e., O(1) time complexity, and runs very quickly.
  /// @warning You should use SetVelocities() instead of this method
  ///          unless you are fully aware of the possible interactions with the
  ///          caching mechanism (see @ref dangerous_get_mutable).
  /// @throws std::exception if the `context` is nullptr or the context does
  /// not correspond to the context for a multibody model.
  /// @pre `state` comes from this MultibodyPlant.
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      const systems::Context<T>& context, systems::State<T>* state) const {
    DRAKE_ASSERT_VOID(CheckValidState(state));
    // Note: the nestedExpression() is necessary to treat the VectorBlock<T>
    // returned from GetMutablePositionsAndVelocities() as a VectorX<T> so that
    // we can call tail() on it.
    return internal_tree()
        .GetMutablePositionsAndVelocities(context, state)
        .nestedExpression()
        .tail(num_velocities());
  }

  /// See GetMutableVelocities() method above.
  Eigen::VectorBlock<VectorX<T>> GetMutableVelocities(
      systems::Context<T>* context) const {
    return GetMutableVelocities(*context, &context->get_mutable_state());
  }

  /// Sets all generalized velocities from the given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, or if the length of
  /// `v` is not equal to `num_velocities()`.
  void SetVelocities(systems::Context<T>* context, const VectorX<T>& v) const {
    GetMutableVelocities(context) = v;
  }

  /// Sets the generalized velocities for a particular model instance from the
  /// given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`.
  /// @pre `state` comes from this MultibodyPlant.
  void SetVelocities(
      const systems::Context<T>& context, systems::State<T>* state,
      ModelInstanceIndex model_instance, const VectorX<T>& v_instance) const {
    CheckValidState(state);
    Eigen::VectorBlock<VectorX<T>> v = GetMutableVelocities(context, state);
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, &v);
  }

  /// Sets the generalized velocities for a particular model instance from the
  /// given vector.
  /// @throws std::exception if the `context` is nullptr, if the context does
  /// not correspond to the context for a multibody model, if the model instance
  /// index is invalid, or if the length of `v_instance` is not equal to
  /// `num_velocities(model_instance)`.
  void SetVelocities(
      systems::Context<T>* context,
      ModelInstanceIndex model_instance, const VectorX<T>& v_instance) const {
    Eigen::VectorBlock<VectorX<T>> v = GetMutableVelocities(context);
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, &v);
  }
  /// @}
  // end multibody state accessors.

  /// @name Adding new multibody elements
  /// %MultibodyPlant users will add modeling elements like bodies,
  /// joints, force elements, constraints, etc, using one of these methods.
  /// Once a user is done adding __all__ modeling elements, the Finalize()
  /// method **must** be called before invoking any %MultibodyPlant service to
  /// perform computations.
  /// An attempt to call any of these methods **after** a call to Finalize() on
  /// the plant, will result on an exception being thrown. See Finalize() for
  /// details.
  /// @{

  /// Creates a rigid body with the provided name and spatial inertia.  This
  /// method returns a constant reference to the body just added, which will
  /// remain valid for the lifetime of `this` %MultibodyPlant.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   ModelInstanceIndex model_instance = plant.AddModelInstance("instance");
  ///   const RigidBody<T>& body =
  ///     plant.AddRigidBody("BodyName", model_instance, spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   @p model_instance. See HasBodyNamed(), Body::name().
  /// @param[in] model_instance
  ///   A model instance index which this body is part of.
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, ModelInstanceIndex model_instance,
      const SpatialInertia<double>& M_BBo_B) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    const RigidBody<T>& body = this->mutable_tree().AddRigidBody(
        name, model_instance, M_BBo_B);
    // Each entry of visual_geometries_, ordered by body index, contains a
    // std::vector of geometry ids for that body. The emplace_back() below
    // resizes visual_geometries_ to store the geometry ids for the body we
    // just added.
    // Similarly for the collision_geometries_ vector.
    DRAKE_DEMAND(visual_geometries_.size() == body.index());
    visual_geometries_.emplace_back();
    DRAKE_DEMAND(collision_geometries_.size() == body.index());
    collision_geometries_.emplace_back();
    return body;
  }

  /// Creates a rigid body with the provided name and spatial inertia.  This
  /// method returns a constant reference to the body just added, which will
  /// remain valid for the lifetime of `this` %MultibodyPlant.  The body will
  /// use the default model instance
  /// (@ref model_instance "more on model instances").
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // ... Code to define spatial_inertia, a SpatialInertia<T> object ...
  ///   const RigidBody<T>& body =
  ///     plant.AddRigidBody("BodyName", spatial_inertia);
  /// @endcode
  ///
  /// @param[in] name
  ///   A string that identifies the new body to be added to `this` model. A
  ///   std::runtime_error is thrown if a body named `name` already is part of
  ///   the model in the default model instance. See HasBodyNamed(),
  ///   Body::name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this`
  ///   %MultibodyPlant, computed about the body frame origin `Bo` and expressed
  ///   in the body frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  /// @throws std::logic_error if additional model instances have been created
  ///                          beyond the world and default instances.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_B) {
    if (num_model_instances() != 2) {
      throw std::logic_error(
          "This model has more model instances than the default.  Please "
          "call AddRigidBody with an explicit model instance.");
    }

    return AddRigidBody(name, default_model_instance(), M_BBo_B);
  }

  /// This method adds a Frame of type `FrameType<T>`. For more information,
  /// please see the corresponding constructor of `FrameType`.
  /// @tparam FrameType Template which will be instantiated on `T`.
  /// @param frame Unique pointer frame instance.
  /// @returns A constant reference to the new Frame just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  template <template<typename> class FrameType>
  const FrameType<T>& AddFrame(std::unique_ptr<FrameType<T>> frame) {
    return this->mutable_tree().AddFrame(std::move(frame));
  }

  /// This method adds a Joint of type `JointType` between two bodies.
  /// For more information, see the below overload of `AddJoint<>`, and the
  /// related `MultibodyTree::AddJoint<>` method.
  template <template<typename Scalar> class JointType>
  const JointType<T>& AddJoint(std::unique_ptr<JointType<T>> joint) {
    static_assert(std::is_convertible<JointType<T>*, Joint<T>*>::value,
                  "JointType must be a sub-class of Joint<T>.");
    return this->mutable_tree().AddJoint(std::move(joint));
  }

  /// This method adds a Joint of type `JointType` between two bodies.
  /// The two bodies connected by this Joint object are referred to as the
  /// _parent_ and _child_ bodies. Although the terms _parent_ and _child_ are
  /// sometimes used synonymously to describe the relationship between inboard
  /// and outboard bodies in multibody models, this usage is wholly unrelated
  /// and implies nothing about the inboard-outboard relationship between the
  /// bodies.
  /// As explained in the Joint class's documentation, in Drake we define a
  /// frame F attached to the parent body P with pose `X_PF` and a frame M
  /// attached to the child body B with pose `X_BM`. This method helps creating
  /// a joint between two bodies with fixed poses `X_PF` and `X_BM`.
  /// Refer to the Joint class's documentation for more details.
  ///
  /// @param name
  ///   A string that uniquely identifies the new joint to be added to `this`
  ///   model. A std::runtime_error is thrown if a joint named `name` already is
  ///   part of the model. See HasJointNamed(), Joint::name().
  /// @param[in] parent
  ///   The parent body connected by the new joint.
  /// @param[in] X_PF
  ///   The fixed pose of frame F attached to the parent body, measured in
  ///   the frame P of that body. `X_PF` is an optional parameter; empty curly
  ///   braces `{}` imply that frame F **is** the same body frame P. If instead
  ///   your intention is to make a frame F with pose `X_PF` equal to the
  ///   identity pose, provide `RigidTransform<double>::Identity()` as your
  ///   input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame M with pose `X_BM` equal to the
  ///   identity pose, provide `RigidTransform<double>::Identity()` as your
  ///   input.
  /// @param[in] args
  ///   Zero or more parameters provided to the constructor of the new joint. It
  ///   must be the case that
  ///   `JointType<T>(
  ///   const std::string&, const Frame<T>&, const Frame<T>&, args)` is a valid
  ///   constructor.
  /// @tparam JointType The type of the Joint to add.
  /// @returns A constant reference to the new joint just added, of type
  ///   `JointType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyPlant. It will remain valid for the lifetime of `this`
  ///   %MultibodyPlant.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // Code to define bodies serving as the joint's parent and child bodies.
  ///   const RigidBody<double>& body_1 =
  ///     plant.AddRigidBody("Body1", SpatialInertia<double>(...));
  ///   const RigidBody<double>& body_2 =
  ///     plant.AddRigidBody("Body2", SpatialInertia<double>(...));
  ///   // Body 1 serves as parent, Body 2 serves as child.
  ///   // Define the pose X_BM of a frame M rigidly atached to child body B.
  ///   const RevoluteJoint<double>& elbow =
  ///     plant.AddJoint<RevoluteJoint>(
  ///       "Elbow",                /* joint name */
  ///       body_1,                 /* parent body */
  ///       {},                     /* frame F IS the parent body frame P */
  ///       body_2,                 /* child body, the pendulum */
  ///       X_BM,                   /* pose of frame M in the body frame B */
  ///       Vector3d::UnitZ());     /* revolute axis in this case */
  /// @endcode
  ///
  /// @throws std::exception if `this` %MultibodyPlant already contains a joint
  /// with the given `name`.  See HasJointNamed(), Joint::name().
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template <template <typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name, const Body<T>& parent,
      const optional<math::RigidTransform<double>>& X_PF, const Body<T>& child,
      const optional<math::RigidTransform<double>>& X_BM, Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    return this->mutable_tree().template AddJoint<JointType>(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
  }

  /// Adds a new force element model of type `ForceElementType` to `this`
  /// %MultibodyPlant.  The arguments to this method `args` are forwarded to
  /// `ForceElementType`'s constructor.
  /// @param[in] args
  ///   Zero or more parameters provided to the constructor of the new force
  ///   element. It must be the case that
  ///   `JointType<T>(args)` is a valid constructor.
  /// @tparam ForceElementType The type of the ForceElement to add.
  /// This method can only be called once for elements of type
  /// UniformGravityFieldElement. That is, gravity can only be specified once.
  /// @returns A constant reference to the new ForceElement just added, of type
  ///   `ForceElementType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyPlant. It will remain valid for the lifetime of `this`
  ///   %MultibodyPlant.
  /// @see The ForceElement class's documentation for further details on how a
  /// force element is defined.
  template<template<typename Scalar> class ForceElementType, typename... Args>
#ifdef DRAKE_DOXYGEN_CXX
  const ForceElementType<T>&
#else
  typename std::enable_if<!std::is_same<
      ForceElementType<T>,
      UniformGravityFieldElement<T>>::value, const ForceElementType<T>&>::type
#endif
  AddForceElement(Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    return this->mutable_tree().template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // SFINAE overload for ForceElementType = UniformGravityFieldElement.
  // This allow us to keep track of the gravity field parameters.
  // TODO(amcastro-tri): This specialization pattern leads to difficult to
  // mantain indirection layers between MBP/MBT and can cause difficult to find
  // bugs, see #11051. It is bad practice and should removed, see #11080.
  template <template <typename Scalar> class ForceElementType, typename... Args>
  typename std::enable_if<
      std::is_same<ForceElementType<T>, UniformGravityFieldElement<T>>::value,
      const ForceElementType<T>&>::type
  AddForceElement(Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    DRAKE_DEMAND(!gravity_field_.has_value());
    // We save the force element so that we can grant users access to it for
    // gravity field specific queries.
    gravity_field_ = &this->mutable_tree()
                          .template AddForceElement<UniformGravityFieldElement>(
                              std::forward<Args>(args)...);
    return *gravity_field_.value();
  }
#endif

  /// Creates and adds a JointActuator model for an actuator acting on a given
  /// `joint`.
  /// This method returns a constant reference to the actuator just added, which
  /// will remain valid for the lifetime of `this` plant.
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new actuator to be added to `this`
  ///   model. A std::runtime_error is thrown if an actuator with the same name
  ///   already exists in the model. See HasJointActuatorNamed().
  /// @param[in] joint
  ///   The Joint to be actuated by the new JointActuator.
  /// @returns A constant reference to the new JointActuator just added, which
  /// will remain valid for the lifetime of `this` plant.
  /// @throws std::exception if `joint.num_velocities() > 1` since for now we
  /// only support actuators for single dof joints.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint) {
    DRAKE_THROW_UNLESS(joint.num_velocities() == 1);
    return this->mutable_tree().AddJointActuator(name, joint);
  }

  /// Creates a new model instance.  Returns the index for the model
  /// instance.
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new instance to be added to `this`
  ///   model. An exception is thrown if an instance with the same name
  ///   already exists in the model. See HasModelInstanceNamed().
  ModelInstanceIndex AddModelInstance(const std::string& name) {
    return this->mutable_tree().AddModelInstance(name);
  }

  /// Welds frames A and B with relative pose `X_AB`. That is, the pose of
  /// frame B in frame A is fixed, with value `X_AB`.
  /// The call to this method creates and adds a new WeldJoint to the model.
  /// The new WeldJoint is named as: A.name() + "_welds_to_" + B.name().
  /// @returns a constant reference to the WeldJoint welding frames A and B.
  const WeldJoint<T>& WeldFrames(const Frame<T>& A, const Frame<T>& B,
                                 const math::RigidTransform<double>& X_AB =
                                     math::RigidTransform<double>::Identity());
  /// @}

  /// @name Querying for multibody elements by name
  /// These methods allow a user to query whether a given multibody element is
  /// part of this plant's model. These queries can be performed at any time
  /// during the lifetime of a %MultibodyPlant model, i.e. there is no
  /// restriction on whether they must be called before or after Finalize().
  /// That is, these queries can be performed while new multibody elements are
  /// being added to the model.
  /// @{

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant.
  /// @see AddRigidBody().
  ///
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  bool HasBodyNamed(const std::string& name) const {
    return internal_tree().HasBodyNamed(name);
  }

  /// @returns `true` if a body named `name` was added to the %MultibodyPlant
  /// in @p model_instance.
  /// @see AddRigidBody().
  ///
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasBodyNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasBodyNamed(name, model_instance);
  }

  /// @returns `true` if a frame named `name` was added to the model.
  /// @see AddFrame().
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  bool HasFrameNamed(const std::string& name) const {
    return internal_tree().HasFrameNamed(name);
  }

  /// @returns `true` if a frame named `name` was added to @p model_instance.
  /// @see AddFrame().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasFrameNamed(const std::string& name,
                     ModelInstanceIndex model_instance) const {
    return internal_tree().HasFrameNamed(name, model_instance);
  }

  /// @returns `true` if a joint named `name` was added to this model.
  /// @see AddJoint().
  /// @throws std::logic_error if the joint name occurs in multiple model
  /// instances.
  bool HasJointNamed(const std::string& name) const {
    return internal_tree().HasJointNamed(name);
  }

  /// @returns `true` if a joint named `name` was added to @p model_instance.
  /// @see AddJoint().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointNamed(name, model_instance);
  }

  /// @returns `true` if an actuator named `name` was added to this model.
  /// @see AddJointActuator().
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  bool HasJointActuatorNamed(const std::string& name) const {
    return internal_tree().HasJointActuatorNamed(name);
  }

  /// @returns `true` if an actuator named `name` was added to
  /// @p model_instance.
  /// @see AddJointActuator().
  /// @throws std::exception if @p model_instance is not valid for this model.
  bool HasJointActuatorNamed(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().HasJointActuatorNamed(name, model_instance);
  }

  /// @returns `true` if a model instance named `name` was added to this model.
  /// @see AddModelInstance().
  bool HasModelInstanceNamed(const std::string& name) const {
    return internal_tree().HasModelInstanceNamed(name);
  }
  /// @}

  /// @name Retrieving multibody elements by name
  /// These methods allow a user to retrieve a reference to a multibody element
  /// by its name. An exception is thrown if there is no element with the
  /// requested name.

  /// These queries can be performed at any time during the lifetime of a
  /// %MultibodyPlant, i.e. there is no restriction on whether they must
  /// be called before or after Finalize(). This implies that these queries can
  /// be performed while new multibody elements are being added to the model.
  ///
  /// If the named element is present in more than one model instance and a
  /// model instance is not explicitly specified, std::logic_error is thrown.
  ///
  /// @{

  /// Returns a constant reference to a body that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const Body<T>& GetBodyByName(const std::string& name) const {
    return internal_tree().GetBodyByName(name);
  }

  /// Returns a constant reference to the body that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @see HasBodyNamed() to query if there exists a body in `this`
  /// %MultibodyPlant with a given specified name.
  const Body<T>& GetBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetBodyByName(name, model_instance);
  }

  /// Returns a list of body indices associated with `model_instance`.
  std::vector<BodyIndex> GetBodyIndices(ModelInstanceIndex model_instance)
  const {
    return internal_tree().GetBodyIndices(model_instance);
  }

  /// Returns a list of joint indices associated with `model_instance`.
  std::vector<JointIndex> GetJointIndices(ModelInstanceIndex model_instance)
  const {
    return internal_tree().GetJointIndices(model_instance);
  }

  /// Returns a constant reference to a frame that is identified by the
  /// string `name` in `this` model.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::logic_error if the frame name occurs in multiple model
  /// instances.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(const std::string& name) const {
    return internal_tree().GetFrameByName(name);
  }

  /// Returns a constant reference to the frame that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no frame with the requested name.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasFrameNamed() to query if there exists a frame in `this` model with
  /// a given specified name.
  const Frame<T>& GetFrameByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetFrameByName(name, model_instance);
  }

  /// Returns a constant reference to a rigid body that is identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the body name occurs in multiple model
  /// instances.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(const std::string& name) const {
    return internal_tree().GetRigidBodyByName(name);
  }

  /// Returns a constant reference to the rigid body that is uniquely identified
  /// by the string `name` in @p model_instance.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @throws std::logic_error if the requested body is not a RigidBody.
  /// @throws std::runtime_error if @p model_instance is not valid for this
  ///         model.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const RigidBody<T>& GetRigidBodyByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetRigidBodyByName(name, model_instance);
  }

  /// Returns a constant reference to a joint that is identified
  /// by the string `name` in `this` %MultibodyPlant.  If the optional
  /// template argument is supplied, then the returned value is downcast to
  /// the specified `JointType`.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::logic_error if the named joint is not of type `JointType` or
  /// if there is no Joint with that name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointNamed() to query if there exists a joint in `this`
  /// %MultibodyPlant with a given specified name.
  template <template <typename> class JointType = Joint>
  const JointType<T>& GetJointByName(
      const std::string& name,
      optional<ModelInstanceIndex> model_instance = nullopt) const {
    return internal_tree().template GetJointByName<JointType>(
        name, model_instance);
  }

  /// A version of GetJointByName that returns a mutable reference.
  /// @see GetJointByName.
  template <template <typename> class JointType = Joint>
  JointType<T>& GetMutableJointByName(
      const std::string& name,
      optional<ModelInstanceIndex> model_instance = nullopt) {
    return this->mutable_tree().template GetMutableJointByName<JointType>(
        name, model_instance);
  }

  /// Returns a constant reference to an actuator that is identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::logic_error if the actuator name occurs in multiple model
  /// instances.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name) const {
    return internal_tree().GetJointActuatorByName(name);
  }

  /// Returns a constant reference to the actuator that is uniquely identified
  /// by the string `name` and @p model_instance in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @throws std::exception if @p model_instance is not valid for this model.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` %MultibodyPlant with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name, ModelInstanceIndex model_instance) const {
    return internal_tree().GetJointActuatorByName(name, model_instance);
  }

  /// Returns the index to the model instance that is uniquely identified
  /// by the string `name` in `this` %MultibodyPlant.
  /// @throws std::logic_error if there is no instance with the requested name.
  /// @see HasModelInstanceNamed() to query if there exists an instance in
  /// `this` %MultibodyPlant with a given specified name.
  ModelInstanceIndex GetModelInstanceByName(const std::string& name) const {
    return internal_tree().GetModelInstanceByName(name);
  }
  /// @}

  /// @name Model instance accessors
  /// Many of this class's methods expect vectors of tree state or
  /// joint actuator inputs which encompass the entire tree.  Methods
  /// in this section are convenience accessors for the portion of
  /// those vectors which apply to a single model instance only.
  /// @{

  /// Given the actuation values `u_instance` for all actuators in
  /// `model_instance`, this method sets the actuation vector u for the entire
  /// model to which this actuator belongs to. This method throws
  /// an exception if the size of `u_instance` is not equal to the number of
  /// degrees of freedom of all of the actuated joints in `model_instance`.
  /// @param[in] u_instance Actuation values for the actuators. It must be of
  ///   size equal to the number of degrees of freedom of all of the actuated
  ///   joints in `model_instance`.
  /// @param[out] u
  ///   The vector containing the actuation values for the entire model.
  void SetActuationInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& u_instance,
      EigenPtr<VectorX<T>> u) const {
    internal_tree().SetActuationInArray(model_instance, u_instance, u);
  }

  /// Returns a vector of generalized positions for `model_instance` from a
  /// vector `q_array` of generalized positions for the entire model
  /// model.  This method throws an exception if `q` is not of size
  /// MultibodyPlant::num_positions().
  VectorX<T> GetPositionsFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q) const {
    return internal_tree().GetPositionsFromArray(model_instance, q);
  }

  /// Sets the vector of generalized positions for `model_instance` in
  /// `q` using `q_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `q` is not of size
  /// MultibodyPlant::num_positions() or `q_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`.
  void SetPositionsInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& q_instance,
      EigenPtr<VectorX<T>> q) const {
    internal_tree().SetPositionsInArray(model_instance, q_instance, q);
  }

  /// Returns a vector of generalized velocities for `model_instance` from a
  /// vector `v` of generalized velocities for the entire MultibodyPlant
  /// model.  This method throws an exception if the input array is not of size
  /// MultibodyPlant::num_velocities().
  VectorX<T> GetVelocitiesFromArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v) const {
    return internal_tree().GetVelocitiesFromArray(model_instance, v);
  }

  /// Sets the vector of generalized velocities for `model_instance` in
  /// `v` using `v_instance`, leaving all other elements in the array
  /// untouched. This method throws an exception if `v` is not of size
  /// MultibodyPlant::num_velocities() or `v_instance` is not of size
  /// `MultibodyPlant::num_positions(model_instance)`.
  void SetVelocitiesInArray(
      ModelInstanceIndex model_instance,
      const Eigen::Ref<const VectorX<T>>& v_instance,
      EigenPtr<VectorX<T>> v) const {
    internal_tree().SetVelocitiesInArray(model_instance, v_instance, v);
  }

  /// @}

  /// @name Accessing the state
  /// @{

  /// Sets `context` to store the pose `X_WB` of a given `body` B in the world
  /// frame W.
  /// @param[in] context
  ///   The context to store the pose `X_WB` of `body_B`.
  /// @param[in] body_B
  ///   The body B corresponding to the pose `X_WB` to be stored in `context`.
  /// @retval X_WB
  ///   The pose of body frame B in the world frame W.
  /// @note In general setting the pose and/or velocity of a body in the model
  /// would involve a complex inverse kinematics problem. This method allows us
  /// to simplify this process when we know the body is free in space.
  /// @throws std::exception if `body` is not a free body in the model.
  /// @throws std::logic_error if called pre-finalize.
  void SetFreeBodyPoseInWorldFrame(
      systems::Context<T>* context,
      const Body<T>& body, const math::RigidTransform<T>& X_WB) const;

  /// Updates `context` to store the pose `X_FB` of a given `body` B in a frame
  /// F.
  /// Frame F must be anchored, meaning that it is either directly welded to the
  /// world frame W or, more generally, that there is a kinematic path between
  /// frame F and the world frame W that only includes weld joints.
  /// @throws std::logic_error if called pre-finalize.
  /// @throws std::logic_error if frame F is not anchored to the world.
  void SetFreeBodyPoseInAnchoredFrame(
      systems::Context<T>* context,
      const Frame<T>& frame_F, const Body<T>& body,
      const math::RigidTransform<T>& X_FB) const;

  /// Computes the relative transform `X_AB(q)` from a frame B to a frame A, as
  /// a function of the generalized positions q of the model.
  /// That is, the position `p_AQ` of a point Q measured and expressed in
  /// frame A can be computed from the position `p_BQ` of this point measured
  /// and expressed in frame B using the transformation `p_AQ = X_AB⋅p_BQ`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @param[in] frame_A
  ///   The target frame A in the computed relative transform `X_AB`.
  /// @param[in] frame_B
  ///   The source frame B in the computed relative transform `X_AB`.
  /// @retval X_AB
  ///   The relative transform from frame B to frame A, such that
  ///   `p_AQ = X_AB⋅p_BQ`.
  math::RigidTransform<T> CalcRelativeTransform(
      const systems::Context<T>& context, const Frame<T>& frame_A,
      const Frame<T>& frame_B) const {
    return internal_tree().CalcRelativeTransform(context, frame_A, frame_B);
  }

  /// Given the positions `p_BQi` for a set of points `Qi` measured and
  /// expressed in a frame B, this method computes the positions `p_AQi(q)` of
  /// each point `Qi` in the set as measured and expressed in another frame A,
  /// as a function of the generalized positions q of the model.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q of the model.
  /// @param[in] frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿᵖ` with
  ///   `np` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B.
  /// @param[in] frame_A
  ///   The frame A in which it is desired to compute the positions `p_AQi` of
  ///   each point `Qi` in the set.
  /// @param[out] p_AQi
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame A. The output `p_AQi` **must** have the same size as
  ///   the input `p_BQi` or otherwise this method aborts. That is `p_AQi`
  ///   **must** be in `ℝ³ˣⁿᵖ`.
  ///
  /// @note Both `p_BQi` and `p_AQi` must have three rows. Otherwise this
  /// method will throw a std::runtime_error exception. This method also throws
  /// a std::runtime_error exception if `p_BQi` and `p_AQi` differ in the number
  /// of columns.
  void CalcPointsPositions(
      const systems::Context<T>& context,
      const Frame<T>& frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const {
    return internal_tree().CalcPointsPositions(
        context, frame_B, p_BQi, frame_A, p_AQi);
  }

  /// Evaluate the pose `X_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[in] body_B
  ///   The body B for which the pose is requested.
  /// @retval X_WB
  ///   The pose of body frame B in the world frame W.
  /// @throws std::exception if Finalize() was not called on `this` model or if
  /// `body_B` does not belong to this model.
  const math::RigidTransform<T>& EvalBodyPoseInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const {
    return internal_tree().EvalBodyPoseInWorld(context, body_B);
  }

  /// Evaluate the spatial velocity `V_WB` of a body B in the world frame W.
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @param[in] body_B
  ///   The body B for which the spatial velocity is requested.
  /// @returns V_WB
  ///   The spatial velocity of body frame B in the world frame W.
  /// @throws std::exception if Finalize() was not called on `this` model or if
  /// `body_B` does not belong to this model.
  const SpatialVelocity<T>& EvalBodySpatialVelocityInWorld(
      const systems::Context<T>& context,
      const Body<T>& body_B) const {
    return internal_tree().EvalBodySpatialVelocityInWorld(context, body_B);
  }

  /// Given a list of points with fixed position vectors `p_FP` in a frame
  /// F, (that is, their time derivative `DtF(p_FP)` in frame F is zero),
  /// this method computes the geometric Jacobian `Jv_WFp` defined by:
  /// <pre>
  ///   v_WP(q, v) = Jv_WFp(q)⋅v
  /// </pre>
  /// where `v_WP(q, v)` is the translational velocity of point `P` in the
  /// world frame W and q and v are the vectors of generalized position and
  /// velocity, respectively.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_F
  ///   The positions `p_FP` of each point in the input set are measured and
  ///   expressed in this frame F and are constant (fixed) in this frame.
  /// @param[in] p_FP_list
  ///   A matrix with the fixed position of a set of points `P` measured and
  ///   expressed in `frame_F`.
  ///   Each column of this matrix contains the position vector `p_FP` for a
  ///   point `P` measured and expressed in frame F. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] p_WP_list
  ///   The output positions of each point `P` now measured and expressed in
  //    the world frame W. These positions are computed in the process of
  ///   computing the geometric Jacobian `J_WP` and therefore external storage
  ///   must be provided.
  ///   The output `p_WP_list` **must** have the same size as the input set
  ///   `p_FP_list` or otherwise this method throws a
  ///   std::runtime_error exception. That is `p_WP_list` **must** be in
  ///   `ℝ³ˣⁿᵖ`.
  /// @param[out] Jv_WFp
  ///   The geometric Jacobian `Jv_WFp(q)`, function of the generalized
  ///   positions q only. This Jacobian relates the translational velocity
  ///   `v_WP` of each point `P` in the input set by: <pre>
  ///     v_WP(q, v) = Jv_WFp(q)⋅v
  ///   </pre>
  ///   so that `v_WP` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `P` in the same order they were given in the
  ///   input set. Therefore `J_WFp` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_WFp` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  ///
  /// @throws std::exception if the output `p_WP_list` is nullptr or does not
  ///  have the same size as the input array `p_FP_list`.
  /// @throws std::exception if `Jv_WFp` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jv_WFp` for details.
  // TODO(amcastro-tri): provide the Jacobian-times-vector operation, since for
  // most applications it is all we need and it is more efficient to compute.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_FP_list,
      EigenPtr<MatrixX<T>> p_WP_list, EigenPtr<MatrixX<T>> Jv_WFp) const {
    return internal_tree().CalcPointsGeometricJacobianExpressedInWorld(
        context, frame_F, p_FP_list, p_WP_list, Jv_WFp);
  }

  /// Computes the bias term `b_WFp` associated with the translational
  /// acceleration `a_WFp` of a point `P` instantaneously moving with a frame F.
  /// That is, the translational acceleration of point `P` can be computed as:
  /// <pre>
  ///   a_WFp = Jv_WFp(q)⋅v̇ + b_WFp(q, v)
  /// </pre>
  /// where `b_WFp = J̇v_WFp(q, v)⋅v`.
  ///
  /// This method computes `b_WFp` for each point `P` in `p_FP_list` defined by
  /// its position `p_FP` in `frame_F`.
  ///
  /// @see CalcPointsGeometricJacobianExpressedInWorld() to compute the
  /// geometric Jacobian `Jv_WFp(q)`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q and generalized velocities v.
  /// @param[in] frame_F
  ///   Points `P` in the list instantaneously move with this frame.
  /// @param[in] p_FP_list
  ///   A matrix with the fixed position of a list of points `P` measured and
  ///   expressed in `frame_F`.
  ///   Each column of this matrix contains the position vector `p_FP` for a
  ///   point `P` measured and expressed in frame F. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the list.
  /// @returns b_WFp
  ///   The bias term, function of the generalized positions q and the
  ///   generalized velocities v as stored in `context`.
  ///   The returned vector has size `3⋅np`, with np the number of points in
  ///   `p_FP_list`, and concatenates the bias terms for each point `P` in the
  ///   list in the same order they are specified on input.
  ///
  /// @throws std::exception if `p_FP_list` does not have 3 rows.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  VectorX<T> CalcBiasForPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F,
      const Eigen::Ref<const MatrixX<T>>& p_FP_list) const {
    return internal_tree().CalcBiasForPointsGeometricJacobianExpressedInWorld(
        context, frame_F, p_FP_list);
  }

  // TODO(eric.cousineau): Reduce duplicate text between overloads.
  /// This is a variant to compute the geometric Jacobian `Jv_WFp` for a list of
  /// points `P` moving with `frame_F`, given that we know the position `p_WP`
  /// of each point in the list measured and expressed in the world frame W. The
  /// geometric Jacobian `Jv_WFp` is defined such that: <pre>
  ///   v_WP(q, v) = Jv_WFp(q)⋅v
  /// </pre>
  /// where `v_WP(q, v)` is the translational velocity of point `P` in the
  /// world frame W and q and v are the vectors of generalized position and
  /// velocity, respectively. Since the spatial velocity of each
  /// point `P` is linear in the generalized velocities, the geometric
  /// Jacobian `Jv_WFp` is a function of the generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_F
  ///   Points `P` in the list instantaneously move with this frame.
  /// @param[in] p_WP_list
  ///   A matrix with the fixed position of a list of points `P` measured and
  ///   expressed in the world frame W.
  ///   Each column of this matrix contains the position vector `p_WP` for a
  ///   point `P` measured and expressed in the world frame W. Therefore this
  ///   input matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the list.
  /// @param[out] Jv_WFp
  ///   The geometric Jacobian `Jv_WFp(q)`, function of the generalized
  ///   positions q only. This Jacobian relates the translational velocity
  ///   `v_WP` of each point `P` in the input list by: <pre>
  ///     `v_WP(q, v) = Jv_WFp(q)⋅v`
  ///   </pre>
  ///   so that `v_WP` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `P` in the same order they were given in the
  ///   input list. Therefore `J_WP` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `J_WP` **must**
  ///   have size `3⋅np x nv` or this method throws a std::runtime_error
  ///   exception.
  ///
  /// @throws std::exception if `Jv_WFp` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jv_WFp` for details.
  // TODO(amcastro-tri): provide the Jacobian-times-vector operation, since for
  // most applications it is all we need and it is more efficient to compute.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_WP_list,
      EigenPtr<MatrixX<T>> Jv_WFp) const {
    return internal_tree().CalcPointsGeometricJacobianExpressedInWorld(
        context, frame_F, p_WP_list, Jv_WFp);
  }

  /// Given a list of points with fixed position vectors `p_FP` in a frame
  /// F, (that is, their time derivative `DtF(p_FP)` in frame F is zero),
  /// this method computes the analytical Jacobian `Jq_WFp(q)`.
  /// The analytical Jacobian `Jq_WFp(q)` is defined by: <pre>
  ///   Jq_WFp(q) = d(p_WFp(q))/dq
  /// </pre>
  /// where `p_WFp(q)` is the position of point P, which moves with frame F, in
  /// the world frame W.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_F
  ///   The positions `p_FP` of each point in the input set are measured and
  ///   expressed in this frame F and are constant (fixed) in this frame.
  /// @param[in] p_FP_list
  ///   A matrix with the fixed position of a set of points `P` measured and
  ///   expressed in `frame_F`.
  ///   Each column of this matrix contains the position vector `p_FP` for a
  ///   point `P` measured and expressed in frame F. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] p_WP_list
  ///   The output positions of each point `P` now measured and expressed in
  //    the world frame W. These positions are computed in the process of
  ///   computing the geometric Jacobian `J_WP` and therefore external storage
  ///   must be provided.
  ///   The output `p_WP_list` **must** have the same size as the input set
  ///   `p_FP_list` or otherwise this method throws a
  ///   std::runtime_error exception. That is `p_WP_list` **must** be in
  ///   `ℝ³ˣⁿᵖ`.
  /// @param[out] Jq_WFp
  ///   The analytical Jacobian `Jq_WFp(q)`, function of the generalized
  ///   positions q only.
  ///   We stack the positions of each point P in the world frame W into a
  ///   column vector p_WFp = [p_WFp1; p_WFp2; ...] of size 3⋅np, with np
  ///   the number of points in p_FP_list. Then the analytical Jacobian is
  ///   defined as: <pre>
  ///     Jq_WFp(q) = ∇(p_WFp(q))
  ///   </pre>
  ///   with `∇(⋅)` the gradient operator with respect to the generalized
  ///   positions q. Therefore `Jq_WFp` is a matrix of size `3⋅np x nq`, with
  ///   `nq` the number of generalized positions. On input, matrix `Jq_WFp`
  ///   **must** have size `3⋅np x nq` or this method throws a
  ///   std::runtime_error exception.
  ///
  /// @throws std::exception if the output `p_WP_list` is nullptr or does not
  /// have the same size as the input array `p_FP_list`.
  /// @throws std::exception if `Jq_WFp` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jq_WFp` for details.
  // TODO(amcastro-tri): provide the Jacobian-times-vector operation, since for
  // most applications it is all we need and it is more efficient to compute.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcPointsAnalyticalJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const MatrixX<T>>& p_FP_list,
      EigenPtr<MatrixX<T>> p_WP_list, EigenPtr<MatrixX<T>> Jq_WFp) const {
    internal_tree().CalcPointsAnalyticalJacobianExpressedInWorld(
        context, frame_F, p_FP_list, p_WP_list, Jq_WFp);
  }

  /// Given a frame `Fp` defined by shifting a frame F from its origin `Fo` to
  /// a new origin `P`, this method computes the geometric Jacobian `Jv_WFp`
  /// for frame `Fp`. The new origin `P` is specified by the position vector
  /// `p_FP` in frame F. The frame geometric Jacobian `Jv_WFp` is defined by:
  /// <pre>
  ///   V_WFp(q, v) = Jv_WFp(q)⋅v
  /// </pre>
  /// where `V_WFp(q, v)` is the spatial velocity of frame `Fp` measured and
  /// expressed in the world frame W and q and v are the vectors of generalized
  /// position and velocity, respectively.
  /// The geometric Jacobian `Jv_WFp(q)` is a function of the generalized
  /// coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_F
  ///   The position `p_FP` of frame `Fp` is measured and expressed in this
  ///   frame F.
  /// @param[in] p_FP
  ///   The (fixed) position of the origin `P` of frame `Fp` as measured and
  ///   expressed in frame F.
  /// @param[out] Jv_WFp
  ///   The geometric Jacobian `Jv_WFp(q)`, function of the generalized
  ///   positions q only. This Jacobian relates to the spatial velocity `V_WFp`
  ///   of frame `Fp` by: <pre>
  ///     V_WFp(q, v) = Jv_WFp(q)⋅v
  ///   </pre>
  ///   Therefore `Jv_WFp` is a matrix of size `6 x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `Jv_WFp` **must**
  ///   have size `6 x nv` or this method throws an exception. The top rows of
  ///   this matrix (which can be accessed with Jv_WFp.topRows<3>()) is the
  ///   Jacobian `Hw_WFp` related to the angular velocity of `Fp` in W by
  ///   `w_WFp = Hw_WFp⋅v`. The bottom rows of this matrix (which can be
  ///   accessed with Jv_WFp.bottomRows<3>()) is the Jacobian `Hv_WFp` related
  ///   to the translational velocity of the origin `P` of frame `Fp` in W by
  ///   `v_WFpo = Hv_WFp⋅v`. This ordering is consistent with the internal
  ///   storage of the SpatialVelocity class. Therefore the following operations
  ///   results in a valid spatial velocity: <pre>
  ///     SpatialVelocity<double> Jv_WFp_times_v(Jv_WFp * v);
  ///   </pre>
  ///
  /// @throws std::exception if `J_WFp` is nullptr or if it is not of size
  ///   `6 x nv`.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcFrameGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const Vector3<T>>& p_FP,
      EigenPtr<MatrixX<T>> Jv_WFp) const {
    internal_tree().CalcFrameGeometricJacobianExpressedInWorld(
        context, frame_F, p_FP, Jv_WFp);
  }

  /// Computes the geometric Jacobian for a point moving with a given frame.
  /// Consider a point P instantaneously moving with a frame B with position
  /// `p_BP` in that frame. Frame `Bp` is the frame defined by shifting frame B
  /// with origin at `Bo` to a new origin at point P. The spatial
  /// velocity `V_ABp_E` of frame `Bp` measured in a frame A and expressed in a
  /// frame E relates to the generalized velocities of the system by the
  /// geometric Jacobian `Jv_ABp_E(q)` by: <pre>
  ///   V_ABp_E(q, v) = Jv_ABp_E(q)⋅v
  /// </pre>
  /// This method computes the geometric Jacobian `Jv_ABp_E(q)`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_B
  ///   The position `p_BP` of point P is measured and expressed in this frame.
  /// @param[in] p_BP
  ///   The (fixed) position of the origin `P` of frame `Bp` as measured and
  ///   expressed in frame B.
  /// @param[in] frame_A
  ///   The second frame in which the spatial velocity `V_ABp` is measured and
  ///   expressed.
  /// @param[in] frame_E
  ///   Frame in which the velocity V_ABp_E is expressed.
  /// @param[out] Jv_ABp_E
  ///   The geometric Jacobian `Jv_ABp_E(q)`, function of the generalized
  ///   positions q only. This Jacobian relates to the spatial velocity
  ///   `V_ABp_E` of frame `Bp` in A and expressed in E by: <pre>
  ///     V_ABp_E(q, v) = Jv_ABp_E(q)⋅v
  ///   </pre>
  ///   Therefore `Jv_ABp_E` is a matrix of size `6 x nv`, with `nv`
  ///   the number of generalized velocities. On input, matrix `Jv_ABp_E`
  ///   **must** have size `6 x nv` or this method throws an exception.
  ///   Given a `6 x nv` spatial Jacobian Jv, let Jvr be the `3 x nv`
  ///   rotational part (top 3 rows) and Jvt be the translational part
  ///   (bottom 3 rows). These can be obtained as follows: <pre>
  ///     Jvr_ABp = Jv_ABp.topRows<3>();
  ///     Jvt_ABp = Jv_ABp.bottomRows<3>();
  ///   </pre>
  ///   This ordering is consistent with the internal storage of the
  ///   SpatialVelocity class. Therefore the following operations results in
  ///   a valid spatial velocity: <pre>
  ///     SpatialVelocity<double> V_ABp(Jv_ABp * v);
  ///   </pre>
  ///
  /// @throws std::exception if `J_ABp` is nullptr or if it is not of size
  ///   `6 x nv`.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  void CalcRelativeFrameGeometricJacobian(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BP,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Jv_ABp_E) const {
    return internal_tree().CalcRelativeFrameGeometricJacobian(
        context, frame_B, p_BP, frame_A, frame_E, Jv_ABp_E);
  }

  /// Given a frame `Fp` defined by shifting a frame F from its origin `Fo` to
  /// a new origin `P`, this method computes the bias term `Ab_WFp` associated
  /// with the spatial acceleration `A_WFp` a frame `Fp` instantaneously
  /// moving with a frame F at a fixed position `p_FP`.
  /// That is, the spatial acceleration of frame `Fp` can be computed as:
  /// <pre>
  ///   A_WFp = Jv_WFp(q)⋅v̇ + Ab_WFp(q, v)
  /// </pre>
  /// where `Ab_WFp(q, v) = J̇v_WFp(q, v)⋅v`.
  ///
  /// @see CalcFrameGeometricJacobianExpressedInWorld() to compute the
  /// geometric Jacobian `Jv_WFp(q)`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q and generalized velocities v.
  /// @param[in] frame_F
  ///   The position `p_FP` of frame `Fp` is measured and expressed in this
  ///   frame F.
  /// @param[in] p_FP
  ///   The (fixed) position of the origin `P` of frame `Fp` as measured and
  ///   expressed in frame F.
  /// @returns Ab_WFp
  ///   The bias term, function of the generalized positions q and the
  ///   generalized velocities v as stored in `context`.
  ///   The returned vector is of size 6, with the first three elements related
  ///   to the bias in angular acceleration and the with the last three elements
  ///   related to the bias in translational acceleration.
  /// @note SpatialAcceleration(Ab_WFp) defines a valid SpatialAcceleration.
  // TODO(amcastro-tri): Rework this method as per issue #10155.
  Vector6<T> CalcBiasForFrameGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_F, const Eigen::Ref<const Vector3<T>>& p_FP) const {
    return internal_tree().CalcBiasForFrameGeometricJacobianExpressedInWorld(
        context, frame_F, p_FP);
  }

  /// Computes the Jacobian of spatial velocity for a frame instantaneously
  /// moving with a specified frame in the model. Consider a point P
  /// instantaneously moving with a frame B with position `p_BP` in that frame.
  /// Frame `Bp` is the frame defined by shifting frame B with origin at `Bo` to
  /// a new origin at point P. The spatial velocity `V_ABp_E` of frame `Bp`
  /// measured in a frame A and expressed in a frame E can be expressed as:
  /// <pre>
  ///   V_ABp_E(q, w) = Jw_ABp_E(q)⋅w
  /// </pre>
  /// where w represents
  ///   * the time derivative of the generalized position vector q̇, if
  ///     `with_respect_to` is JacobianWrtVariable::kQDot.
  ///   * the generalized velocity vector v, if `with_respect_to` is
  ///     JacobianWrtVariable::kV.
  ///
  /// This method computes `Jw_ABp_E(q)`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] with_respect_to
  ///   Enum indicating whether `Jw_ABp_E` converts generalized velocities or
  ///   time-derivatives of generalized positions to spatial velocities.
  /// @param[in] frame_B
  ///   The position `p_BP` of point P is measured and expressed in this frame.
  /// @param[in] p_BP
  ///   The (fixed) position of the origin `P` of frame `Bp` as measured and
  ///   expressed in frame B.
  /// @param[in] frame_A
  ///   The second frame in which the spatial velocity `V_ABp` is measured.
  /// @param[in] frame_E
  ///   Frame in which the velocity V_ABp_E, and therefore the Jacobian Jw_ABp_E
  ///   is expressed.
  /// @param[out] Jw_ABp_E
  ///   The Jacobian `Jw_ABp_E(q)`, function of the generalized
  ///   positions q only. This Jacobian relates to the spatial velocity
  ///   `V_ABp_E` of frame `Bp` in `A` and expressed in `E` by: <pre>
  ///     V_ABp_E(q, w) = Jw_ABp_E(q)⋅w </pre>
  ///   Therefore `Jw_ABp_E` is a matrix of size `6 x nz`, where `nz` is the
  ///   number of elements in w. On input, matrix `Jv_ABp_E` **must** have size
  ///   `6 x nz` or this method throws an exception. Given a `6 x nz` Jacobian
  ///   J, let Jr be the `3 x nz` rotational part (top 3 rows) and Jt be the
  ///   translational part (bottom 3 rows). These can be obtained as follows:
  ///   ```
  ///     Jr_ABp_E = Jw_ABp_E.topRows<3>();
  ///     Jt_ABp_E = Jw_ABp_E.bottomRows<3>();
  ///   ```
  ///   This ordering is consistent with the internal storage of the
  ///   SpatialVelocity class. Therefore the following operations results in
  ///   a valid spatial velocity: <pre>
  ///     SpatialVelocity<double> V_ABp(Jw_ABp * w); </pre>
  ///
  /// @throws std::exception if `Jw_ABp_E` is nullptr or if it is not of size
  ///   `6 x nz`.
  void CalcJacobianSpatialVelocity(
      const systems::Context<T>& context,
      JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BP,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Jw_ABp_E) const {
    return internal_tree().CalcJacobianSpatialVelocity(
        context, with_respect_to, frame_B, p_BP, frame_A, frame_E, Jw_ABp_E);
  }

  /// Returns a frame B's angular velocity Jacobian in a frame A with respect
  /// to "speeds" 𝑠, where 𝑠 is either q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of
  /// generalized positions) or v ≜ [v₁ ... vₖ]ᵀ (generalized velocities).
  /// When a frame B's angular velocity `w_AB` in a frame A is characterized by
  /// speeds 𝑠, B's angular velocity Jacobian in A with respect to 𝑠 is
  /// <pre>
  ///      Js_w_AB = [ ∂(w_AB)/∂𝑠₁,  ...  ∂(w_AB)/∂𝑠ₙ ]    (n is j or k)
  /// </pre>
  /// B's angular velocity in A is linear in 𝑠₁, ... 𝑠ₙ and can be written
  /// `w_AB = Js_w_AB ⋅ 𝑠`  where 𝑠 is [𝑠₁ ... 𝑠ₙ]ᵀ.
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `Js_w_AB` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame B in `w_AB` (B's angular velocity in A).
  /// @param[in] frame_A The frame A in `w_AB` (B's angular velocity in A).
  /// @param[in] frame_E The frame in which `w_AB` is expressed on input and
  /// the frame in which the Jacobian `Js_w_AB` is expressed on output.
  /// @param[out] Js_w_AB_E Frame B's angular velocity Jacobian in frame A with
  /// respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).  The previous definition shows `Js_w_AB_E` is
  /// a matrix of size `3 x n`, where n is the number of elements in 𝑠.
  /// @throws std::exception if `Js_w_AB_E` is nullptr or not of size `3 x n`.
  void CalcJacobianAngularVelocity(const systems::Context<T>& context,
                                   const JacobianWrtVariable with_respect_to,
                                   const Frame<T>& frame_B,
                                   const Frame<T>& frame_A,
                                   const Frame<T>& frame_E,
                                   EigenPtr<MatrixX<T>> Js_w_AB_E) const {
    return internal_tree().CalcJacobianAngularVelocity(
        context, with_respect_to, frame_B, frame_A, frame_E, Js_w_AB_E);
  }

  /// Return a point's translational velocity Jacobian in a frame A with respect
  /// to "speeds" 𝑠, where 𝑠 is either q̇ ≜ [q̇₁ ... q̇ⱼ]ᵀ (time-derivatives of
  /// generalized positions) or v ≜ [v₁ ... vₖ]ᵀ (generalized velocities).
  /// For a point Bp of (fixed/welded to) a frame B whose translational velocity
  /// `v_ABp` in a frame A is characterized by speeds 𝑠, Bp's velocity Jacobian
  /// in A with respect to 𝑠 is defined as
  /// <pre>
  ///      Js_v_ABp = [ ∂(v_ABp)/∂𝑠₁,  ...  ∂(v_ABp)/∂𝑠ₙ ]    (n is j or k)
  /// </pre>
  /// Point Bp's velocity in A is linear in 𝑠₁, ... 𝑠ₙ and can be written
  /// `v_ABp = Js_v_ABp ⋅ 𝑠`  where 𝑠 is [𝑠₁ ... 𝑠ₙ]ᵀ.
  ///
  /// @param[in] context The state of the multibody system.
  /// @param[in] with_respect_to Enum equal to JacobianWrtVariable::kQDot or
  /// JacobianWrtVariable::kV, indicating whether the Jacobian `Js_v_ABp` is
  /// partial derivatives with respect to 𝑠 = q̇ (time-derivatives of generalized
  /// positions) or with respect to 𝑠 = v (generalized velocities).
  /// @param[in] frame_B The frame on which point Bp is fixed/welded.
  /// @param[in] p_BoBp_B The position vector from Bo (frame_B's origin) to
  ///   point Bp (which is regarded as fixed to B), expressed in frame B.
  /// @param[in] frame_A The frame that measures `v_ABp` (Bp's velocity in A).
  /// @param[in] frame_E The frame in which `v_ABp` is expressed on input and
  /// the frame in which the Jacobian `Js_v_ABp` is expressed on output.
  /// @param[out] Js_v_ABp_E Point Bp's velocity Jacobian in frame A with
  /// respect to speeds 𝑠 (which is either q̇ or v), expressed in frame E.
  /// The Jacobian is a function of only generalized positions q (which are
  /// pulled from the context).  The previous definition shows `Js_v_ABp_E` is
  /// a matrix of size `3 x n`, where n is the number of elements in 𝑠.
  /// @throws std::exception if `Js_v_ABp_E` is nullptr or not of size `3 x n`.
  void CalcJacobianTranslationalVelocity(
      const systems::Context<T>& context, JacobianWrtVariable with_respect_to,
      const Frame<T>& frame_B, const Eigen::Ref<const Vector3<T>>& p_BoBp_B,
      const Frame<T>& frame_A, const Frame<T>& frame_E,
      EigenPtr<MatrixX<T>> Js_v_ABp_E) const {
    return internal_tree().CalcJacobianTranslationalVelocity(
        context, with_respect_to, frame_B, p_BoBp_B, frame_A, frame_E,
        Js_v_ABp_E);
  }

  /// Given the state of this model in `context` and a known vector
  /// of generalized accelerations `known_vdot`, this method computes the
  /// spatial acceleration `A_WB` for each body as measured and expressed in the
  /// world frame W.
  ///
  /// @param[in] context
  ///   The context containing the state of this model.
  /// @param[in] known_vdot
  ///   A vector with the generalized accelerations for the full model.
  /// @param[out] A_WB_array
  ///   A pointer to a valid, non nullptr, vector of spatial accelerations
  ///   containing the spatial acceleration `A_WB` for each body. It must be of
  ///   size equal to the number of bodies in the model. On output,
  ///   entries will be ordered by BodyIndex.
  /// @throws std::exception if A_WB_array is not of size `num_bodies()`.
  void CalcSpatialAccelerationsFromVdot(
      const systems::Context<T>& context,
      const VectorX<T>& known_vdot,
      std::vector<SpatialAcceleration<T>>* A_WB_array) const;

  /// Given the state of this model in `context` and a known vector
  /// of generalized accelerations `vdot`, this method computes the
  /// set of generalized forces `tau` that would need to be applied in order to
  /// attain the specified generalized accelerations.
  /// Mathematically, this method computes: <pre>
  ///   tau = M(q)v̇ + C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the model's mass matrix, `C(q, v)v` is the bias
  /// term containing Coriolis and gyroscopic effects and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the geometric Jacobian `J_WB(q)` which maps generalized velocities
  /// into body B spatial velocity as `V_WB = J_WB(q)v`.
  /// This method does not compute explicit expressions for the mass matrix nor
  /// for the bias term, which would be of at least `O(n²)` complexity, but it
  /// implements an `O(n)` Newton-Euler recursive algorithm, where n is the
  /// number of bodies in the model. The explicit formation of the
  /// mass matrix `M(q)` would require the calculation of `O(n²)` entries while
  /// explicitly forming the product `C(q, v) * v` could require up to `O(n³)`
  /// operations (see [Featherstone 1987, §4]), depending on the implementation.
  /// The recursive Newton-Euler algorithm is the most efficient currently known
  /// general method for solving inverse dynamics [Featherstone 2008].
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] known_vdot
  ///   A vector with the known generalized accelerations `vdot` for the full
  ///   model. Use the provided Joint APIs in order to access entries into this
  ///   array.
  /// @param[in] external_forces
  ///   A set of forces to be applied to the system either as body spatial
  ///   forces `Fapp_Bo_W` or generalized forces `tau_app`, see MultibodyForces
  ///   for details.
  ///
  /// @returns the vector of generalized forces that would need to be applied to
  /// the mechanical system in order to achieve the desired acceleration given
  /// by `known_vdot`.
  VectorX<T> CalcInverseDynamics(
      const systems::Context<T>& context,
      const VectorX<T>& known_vdot,
      const MultibodyForces<T>& external_forces) const {
    return internal_tree().CalcInverseDynamics(
        context, known_vdot, external_forces);
  }

  /// Computes the combined force contribution of ForceElement objects in the
  /// model. A ForceElement can apply forces as a spatial force per body or as
  /// generalized forces, depending on the ForceElement model.
  /// ForceElement contributions are a function of the state and time only.
  /// The output from this method can immediately be used as input to
  /// CalcInverseDynamics() to include the effect of applied forces by force
  /// elements.
  ///
  /// @param[in] context
  ///   The context containing the state of this model.
  /// @param[out] forces
  ///   A pointer to a valid, non nullptr, multibody forces object. On output
  ///   `forces` will store the forces exerted by all the ForceElement
  ///   objects in the model.
  /// @throws std::exception if `forces` is null or not compatible with this
  ///   model, per MultibodyForces::CheckInvariants().
  void CalcForceElementsContribution(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  /// Computes and returns the total potential energy stored in `this` multibody
  /// model for the configuration given by `context`.
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @returns The total potential energy stored in `this` multibody model.
  T CalcPotentialEnergy(const systems::Context<T>& context) const {
    return internal_tree().CalcPotentialEnergy(context);
  }

  /// Computes and returns the power generated by conservative forces in the
  /// multibody model. This quantity is defined to be positive when the
  /// potential energy is decreasing. In other words, if `U(q)` is the potential
  /// energy as defined by CalcPotentialEnergy(), then the conservative power,
  /// `Pc`, is `Pc = -U̇(q)`.
  ///
  /// @see CalcPotentialEnergy()
  T CalcConservativePower(const systems::Context<T>& context) const {
    return internal_tree().CalcConservativePower(context);
  }

  /// Computes the bias term `C(q, v)v` containing Coriolis and gyroscopic
  /// effects of the multibody equations of motion: <pre>
  ///   M(q)v̇ + C(q, v)v = tau_app + ∑ J_WBᵀ(q) Fapp_Bo_W
  /// </pre>
  /// where `M(q)` is the multibody model's mass matrix and `tau_app` consists
  /// of a vector applied generalized forces. The last term is a summation over
  /// all bodies in the model where `Fapp_Bo_W` is an applied spatial force on
  /// body B at `Bo` which gets projected into the space of generalized forces
  /// with the geometric Jacobian `J_WB(q)` which maps generalized velocities
  /// into body B spatial velocity as `V_WB = J_WB(q)v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q and the generalized velocities v.
  /// @param[out] Cv
  ///   On output, `Cv` will contain the product `C(q, v)v`. It must be a valid
  ///   (non-null) pointer to a column vector in `ℛⁿ` with n the number of
  ///   generalized velocities (num_velocities()) of the model.
  ///   This method aborts if Cv is nullptr or if it does not have the
  ///   proper size.
  void CalcBiasTerm(
      const systems::Context<T>& context, EigenPtr<VectorX<T>> Cv) const {
    internal_tree().CalcBiasTerm(context, Cv);
  }

  /// Computes the generalized forces `tau_g(q)` due to gravity as a function
  /// of the generalized positions `q` stored in the input `context`.
  /// The vector of generalized forces due to gravity `tau_g(q)` is defined such
  /// that it appears on the right hand side of the equations of motion together
  /// with any other generalized forces, like so:
  /// <pre>
  ///   Mv̇ + C(q, v)v = tau_g(q) + tau_app
  /// </pre>
  /// where `tau_app` includes any other generalized forces applied on the
  /// system.
  ///
  /// @param[in] context
  ///   The context storing the state of the model.
  /// @returns tau_g
  ///   A vector containing the generalized forces due to gravity.
  ///   The generalized forces are consistent with the vector of
  ///   generalized velocities `v` for `this` so that
  ///   the inner product `v⋅tau_g` corresponds to the power applied by the
  ///   gravity forces on the mechanical system. That is, `v⋅tau_g > 0`
  ///   corresponds to potential energy going into the system, as either
  ///   mechanical kinetic energy, some other potential energy, or heat, and
  ///   therefore to a decrease of the gravitational potential energy.
  VectorX<T> CalcGravityGeneralizedForces(
      const systems::Context<T>& context) const {
    return internal_tree().CalcGravityGeneralizedForces(context);
  }

  // Preserve access to base overload from this class.
  using systems::System<T>::MapVelocityToQDot;

  /// Transforms generalized velocities v to time derivatives `qdot` of the
  /// generalized positions vector `q` (stored in `context`). `v` and `qdot`
  /// are related linearly by `q̇ = N(q)⋅v`.
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `q̇ = N(q)⋅v`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] v
  ///   A vector of of generalized velocities for this model.
  ///   This method aborts if v is not of size num_velocities().
  /// @param[out] qdot
  ///   A valid (non-null) pointer to a vector in `ℝⁿ` with n being the number
  ///   of generalized positions in this model,
  ///   given by `num_positions()`. This method aborts if `qdot` is nullptr
  ///   or if it is not of size num_positions().
  ///
  /// @see MapQDotToVelocity()
  /// @see Mobilizer::MapVelocityToQDot()
  void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const {
    return internal_tree().MapVelocityToQDot(context, v, qdot);
  }

  // Preserve access to base overload from this class.
  using systems::System<T>::MapQDotToVelocity;

  /// Transforms the time derivative `qdot` of the generalized positions vector
  /// `q` (stored in `context`) to generalized velocities `v`. `v` and `q̇`
  /// are related linearly by `q̇ = N(q)⋅v`. Although `N(q)` is not
  /// necessarily square, its left pseudo-inverse `N⁺(q)` can be used to
  /// invert that relationship without residual error, provided that `qdot` is
  /// in the range space of `N(q)` (that is, if it *could* have been produced as
  /// `q̇ = N(q)⋅v` for some `v`).
  /// Using the configuration `q` stored in the given `context` this method
  /// calculates `v = N⁺(q)⋅q̇`.
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[in] qdot
  ///   A vector containing the time derivatives of the generalized positions.
  ///   This method aborts if `qdot` is not of size num_positions().
  /// @param[out] v
  ///   A valid (non-null) pointer to a vector in `ℛⁿ` with n the number of
  ///   generalized velocities. This method aborts if v is nullptr or if it
  ///   is not of size num_velocities().
  ///
  /// @see MapVelocityToQDot()
  /// @see Mobilizer::MapQDotToVelocity()
  void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const {
    internal_tree().MapQDotToVelocity(context, qdot, v);
  }

  /// This method allows users to map the state of `this` model, x, into a
  /// vector of selected state xₛ with a given preferred ordering.
  /// The mapping, or selection, is returned in the form of a selector matrix
  /// Sx such that `xₛ = Sx⋅x`. The size nₛ of xₛ is always smaller or equal
  /// than the size of the full state x. That is, a user might be interested in
  /// only a given portion of the full state x.
  ///
  /// This selection matrix is particularly useful when adding PID control
  /// on a portion of the state, see systems::controllers::PidController.
  ///
  /// A user specifies the preferred order in xₛ via `user_to_joint_index_map`.
  /// The selected state is built such that selected positions are followed
  /// by selected velocities, as in `xₛ = [qₛ, vₛ]`.
  /// The positions in qₛ are a concatenation of the positions for each joint
  /// in the order they appear in `user_to_joint_index_map`. That is, the
  /// positions for `user_to_joint_index_map[0]` are first, followed by the
  /// positions for `user_to_joint_index_map[1]`, etc. Similarly for the
  /// selected velocities vₛ.
  ///
  /// @throws std::logic_error if there are repeated indexes in
  /// `user_to_joint_index_map`.
  // TODO(amcastro-tri): consider having an extra `free_body_index_map`
  // so that users could also re-order free bodies if they wanted to.
  MatrixX<double> MakeStateSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const {
    return internal_tree().MakeStateSelectorMatrix(user_to_joint_index_map);
  }

  /// This method allows user to map a vector `uₛ` containing the actuation
  /// for a set of selected actuators into the vector u containing the actuation
  /// values for `this` full model.
  /// The mapping, or selection, is returned in the form of a selector matrix
  /// Su such that `u = Su⋅uₛ`. The size nₛ of uₛ is always smaller or equal
  /// than the size of the full vector of actuation values u. That is, a user
  /// might be interested in only a given subset of actuators in the model.
  ///
  /// This selection matrix is particularly useful when adding PID control
  /// on a portion of the state, see systems::controllers::PidController.
  ///
  /// A user specifies the preferred order in uₛ via
  /// `user_to_actuator_index_map`. The actuation values in uₛ are a
  /// concatenation of the values for each actuator in the order they appear in
  /// `user_to_actuator_index_map`.
  /// The full vector of actuation values u is ordered by JointActuatorIndex.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointActuatorIndex>& user_to_actuator_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(
        user_to_actuator_index_map);
  }

  /// Alternative signature to build an actuation selector matrix `Su` such
  /// that `u = Su⋅uₛ`, where u is the vector of actuation values for the full
  /// model (ordered by JointActuatorIndex) and uₛ is a vector of actuation
  /// values for the actuators acting on the joints listed by
  /// `user_to_joint_index_map`. It is assumed that all joints referenced by
  /// `user_to_joint_index_map` are actuated.
  /// See MakeActuatorSelectorMatrix(const std::vector<JointActuatorIndex>&) for
  /// details.
  /// @throws std::logic_error if any of the joints in
  /// `user_to_joint_index_map` does not have an actuator.
  MatrixX<double> MakeActuatorSelectorMatrix(
      const std::vector<JointIndex>& user_to_joint_index_map) const {
    return internal_tree().MakeActuatorSelectorMatrix(user_to_joint_index_map);
  }

  /// Returns a vector of size `num_positions()` containing the lower position
  /// limits for every generalized position coordinate. These include joint and
  /// floating base coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::logic_error if called pre-finalize.
  VectorX<double> GetPositionLowerLimits() const {
    return internal_tree().GetPositionLowerLimits();
  }

  /// Upper limit analog of GetPositionsLowerLimits(), where any unbounded or
  /// unspecified limits will be +infinity.
  /// @see GetPositionLowerLimits() for more information.
  VectorX<double> GetPositionUpperLimits() const {
    return internal_tree().GetPositionUpperLimits();
  }

  /// Returns a vector of size `num_velocities()` containing the lower velocity
  /// limits for every generalized velocity coordinate. These include joint and
  /// floating base coordinates. Any unbounded or unspecified limits will be
  /// -infinity.
  /// @throws std::logic_error if called pre-finalize.
  VectorX<double> GetVelocityLowerLimits() const {
    return internal_tree().GetVelocityLowerLimits();
  }

  /// Upper limit analog of GetVelocitysLowerLimits(), where any unbounded or
  /// unspecified limits will be +infinity.
  /// @see GetVelocityLowerLimits() for more information.
  VectorX<double> GetVelocityUpperLimits() const {
    return internal_tree().GetVelocityUpperLimits();
  }

  /// Returns a vector of size `num_velocities()` containing the lower
  /// acceleration limits for every generalized velocity coordinate. These
  /// include joint and floating base coordinates. Any unbounded or unspecified
  /// limits will be -infinity.
  /// @throws std::logic_error if called pre-finalize.
  VectorX<double> GetAccelerationLowerLimits() const {
    return internal_tree().GetAccelerationLowerLimits();
  }

  /// Upper limit analog of GetAccelerationsLowerLimits(), where any unbounded
  /// or unspecified limits will be +infinity.
  /// @see GetAccelerationLowerLimits() for more information.
  VectorX<double> GetAccelerationUpperLimits() const {
    return internal_tree().GetAccelerationUpperLimits();
  }

  /// Performs the computation of the mass matrix `M(q)` of the model using
  /// inverse dynamics, where the generalized positions q are stored in
  /// `context`. See CalcInverseDynamics().
  ///
  /// @param[in] context
  ///   The context containing the state of the model.
  /// @param[out] H
  ///   A valid (non-null) pointer to a squared matrix in `ℛⁿˣⁿ` with n the
  ///   number of generalized velocities (num_velocities()) of the model.
  ///   This method aborts if H is nullptr or if it does not have the proper
  ///   size.
  ///
  /// The algorithm used to build `M(q)` consists in computing one column of
  /// `M(q)` at a time using inverse dynamics. The result from inverse dynamics,
  /// with no applied forces, is the vector of generalized forces: <pre>
  ///   tau = M(q)v̇ + C(q, v)v
  /// </pre>
  /// where q and v are the generalized positions and velocities, respectively.
  /// When `v = 0` the Coriolis and gyroscopic forces term `C(q, v)v` is zero.
  /// Therefore the `i-th` column of `M(q)` can be obtained performing inverse
  /// dynamics with an acceleration vector `v̇ = eᵢ`, with `eᵢ` the standard
  /// (or natural) basis of `ℛⁿ` with n the number of generalized velocities.
  /// We write this as: <pre>
  ///   H.ᵢ(q) = M(q) * e_i
  /// </pre>
  /// where `H.ᵢ(q)` (notice the dot for the rows index) denotes the `i-th`
  /// column in M(q).
  ///
  /// @warning This is an O(n²) algorithm. Avoid the explicit computation of the
  /// mass matrix whenever possible.
  void CalcMassMatrixViaInverseDynamics(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> H) const {
    internal_tree().CalcMassMatrixViaInverseDynamics(context, H);
  }

  // TODO(amcastro-tri): Add state accessors for free body spatial velocities.

  /// @}

  /// This method creates an actuation matrix B mapping a vector of actuation
  /// values u into generalized forces `tau_u = B * u`, where B is a matrix of
  /// size `nv x nu` with `nu` equal to num_actuators() and `nv` equal to
  /// num_velocities().
  /// The vector u of actuation values is of size num_actuators(). For a given
  /// JointActuator, `u[JointActuator::index()]` stores the value for the
  /// external actuation corresponding to that actuator. `tau_u` on the other
  /// hand is indexed by generalized velocity indexes according to
  /// `Joint::velocity_start()`.
  /// @warning B is a permutation matrix. While making a permutation has
  /// `O(n)` complexity, making a full B matrix has `O(n²)` complexity. For most
  /// applications this cost can be neglected but it could become significant
  /// for very large systems.
  MatrixX<T> MakeActuationMatrix() const;

  /// Registers `this` plant to serve as a source for an instance of
  /// SceneGraph. This registration allows %MultibodyPlant to
  /// register geometry with `scene_graph` for visualization and/or
  /// collision queries.
  /// Successive registration calls with SceneGraph **must** be performed on
  /// the same instance to which the pointer argument `scene_graph` points
  /// to. Failure to do so will result in runtime exceptions.
  /// @param scene_graph
  ///   A valid non nullptr to the SceneGraph instance for which
  ///   `this` plant will sever as a source, see SceneGraph documentation
  ///   for further details.
  /// @returns the SourceId of `this` plant in `scene_graph`. It can also
  /// later on be retrieved with get_source_id().
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if `scene_graph` is the nullptr.
  /// @throws std::exception if called more than once.
  geometry::SourceId RegisterAsSourceForSceneGraph(
      geometry::SceneGraph<T>* scene_graph);

  /// Registers geometry in a SceneGraph with a given geometry::Shape to be
  /// used for visualization of a given `body`.
  ///
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] X_BG
  ///   The fixed pose of the geometry frame G in the body frame B.
  /// @param[in] shape
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder, etc.
  /// @param[in] name
  ///   The name for the geometry. It must satisfy the requirements defined in
  ///   drake::geometry::GeometryInstance.
  /// @param[in] properties
  ///   The illustration properties for this geometry.
  /// @param[out] scene_graph
  ///   (Deprecated) A valid non nullptr to a SceneGraph on which geometry will
  ///   get registered.
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if `scene_graph` does not correspond to the same
  /// instance with which RegisterAsSourceForSceneGraph() was called.
  /// @returns the id for the registered geometry.
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const geometry::IllustrationProperties& properties,
      geometry::SceneGraph<T>* scene_graph = nullptr);

  /// Overload for visual geometry registration; it converts the `diffuse_color`
  /// (RGBA with values in the range [0, 1]) into a
  /// geometry::ConnectDrakeVisualizer()-compatible set of
  /// geometry::IllustrationProperties.
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const Vector4<double>& diffuse_color,
      geometry::SceneGraph<T>* scene_graph = nullptr);

  /// Overload for visual geometry registration; it relies on the downstream
  /// geometry::IllustrationProperties _consumer_ to provide default parameter
  /// values (see @ref geometry_roles for details).
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      geometry::SceneGraph<T>* scene_graph = nullptr);

  /// Returns an array of GeometryId's identifying the different visual
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterVisualGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetVisualGeometriesForBody(
      const Body<T>& body) const;

  /// Returns the number of geometries registered for visualization.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_visual_geometries() const {
    return static_cast<int>(geometry_id_to_visual_index_.size());
  }

  /// Registers geometry in a SceneGraph with a given geometry::Shape to be
  /// used for the contact modeling of a given `body`.
  /// More than one geometry can be registered with a body, in which case the
  /// body's contact geometry is the union of all geometries registered to that
  /// body.
  ///
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] X_BG
  ///   The fixed pose of the geometry frame G in the body frame B.
  /// @param[in] shape
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder, etc.
  /// @param[in] coulomb_friction
  ///   Coulomb's law of friction coefficients to model friction on the
  ///   surface of `shape` for the given `body`.
  /// @param[out] scene_graph
  ///   (Deprecated) A valid, non-null pointer to a SceneGraph on which
  ///   geometry will get registered.
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if `scene_graph` does not correspond to the
  /// same instance with which RegisterAsSourceForSceneGraph() was called.
  geometry::GeometryId RegisterCollisionGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const CoulombFriction<double>& coulomb_friction,
      geometry::SceneGraph<T>* scene_graph = nullptr);

  /// Returns an array of GeometryId's identifying the different contact
  /// geometries for `body` previously registered with a SceneGraph.
  /// @note This method can be called at any time during the lifetime of `this`
  /// plant, either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  /// @see RegisterCollisionGeometry(), Finalize()
  const std::vector<geometry::GeometryId>& GetCollisionGeometriesForBody(
      const Body<T>& body) const;

  /// Returns the number of geometries registered for contact modeling.
  /// This method can be called at any time during the lifetime of `this` plant,
  /// either pre- or post-finalize, see Finalize().
  /// Post-finalize calls will always return the same value.
  int num_collision_geometries() const {
    return geometry_id_to_collision_index_.size();
  }

  /// For each of the provided `bodies`, collects up all geometries that have
  /// been registered to that body. Intended to be used in conjunction with
  /// SceneGraph::ExcludeCollisionsWithin() and
  /// SceneGraph::ExcludeCollisionsBetween() to filter collisions between the
  /// geometries registered to the bodies.
  ///
  /// For example:
  /// ```
  /// // Don't report on collisions between geometries affixed to `body1`,
  /// // `body2`, or `body3`.
  /// std::vector<const RigidBody<T>*> bodies{&body1, &body2, &body3};
  /// geometry::GeometrySet set = plant.CollectRegisteredGeometries(bodies);
  /// scene_graph.ExcludeCollisionsWithin(set);
  /// ```
  ///
  /// @note There is a *very* specific order of operations:
  ///
  /// 1. Bodies and geometries must be added to the %MultibodyPlant.
  /// 2. The %MultibodyPlant must be finalized (via Finalize()).
  /// 3. Create GeometrySet instances from bodies (via this method).
  /// 4. Invoke SceneGraph::ExcludeCollisions*() to filter collisions.
  /// 5. Allocate context.
  ///
  /// Changing the order will cause exceptions to be thrown.
  ///
  /// @throws std::exception if called pre-finalize.
  geometry::GeometrySet CollectRegisteredGeometries(
      const std::vector<const Body<T>*>& bodies) const;

  /// Returns all bodies that are transitively welded, or rigidly affixed, to
  /// `body`, per these two definitions:
  ///
  /// 1. A body is always considered welded to itself.
  /// 2. Two unique bodies are considered welded together exclusively by the
  /// presence of a weld joint, not by other constructs that prevent mobility
  /// (e.g. constraints).
  ///
  /// Meant to be used with `CollectRegisteredGeometries`.
  ///
  /// The following example demonstrates filtering collisions between all
  /// bodies rigidly affixed to a door (which could be moving) and all bodies
  /// rigidly affixed to the world:
  /// @code
  /// GeometrySet g_world = plant.CollectRegisteredGeometries(
  ///     plant.GetBodiesWeldedTo(plant.world_body()));
  /// GeometrySet g_door = plant.CollectRegisteredGeometries(
  ///     plant.GetBodiesWeldedTo(plant.GetBodyByName("door")));
  /// scene_graph.ExcludeCollisionsBetweeen(g_world, g_door);
  /// @endcode
  /// @note Usages akin to this example may introduce redundant collision
  /// filtering; this will not have a functional impact, but may have a minor
  /// performance impact.
  ///
  /// @returns all bodies rigidly fixed to `body`. This does not return the
  /// bodies in any prescribed order.
  /// @throws std::exception if called pre-finalize.
  /// @throws std::exception if `body` is not part of this plant.
  std::vector<const Body<T>*> GetBodiesWeldedTo(const Body<T>& body) const;

  /// Returns the friction coefficients provided during geometry registration
  /// for the given geometry `id`. We call these the "default" coefficients but
  /// note that we mean user-supplied per-geometry default, not something more
  /// global.
  /// @throws std::exception if `id` does not correspond to a geometry in `this`
  /// model registered for contact modeling.
  /// @see RegisterCollisionGeometry() for details on geometry registration.
  // TODO(amcastro-tri): This API might change or disappear completely as GS
  // provides support for the specification of surface properties.
  const CoulombFriction<double>& default_coulomb_friction(
      geometry::GeometryId id) const {
    DRAKE_DEMAND(is_collision_geometry(id));
    const int collision_index = geometry_id_to_collision_index_.at(id);
    return default_coulomb_friction_[collision_index];
  }

  /// @name Retrieving ports for communication with a SceneGraph.
  /// @{

  /// Returns the unique id identifying `this` plant as a source for a
  /// SceneGraph.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize(). However, a geometry::SourceId is
  /// only assigned once at the first call of any of this plant's geometry
  /// registration methods, and it does not change after that.
  /// Post-finalize calls will always return the same value.
  optional<geometry::SourceId> get_source_id() const {
    return source_id_;
  }

  /// Returns a constant reference to the input port used to perform geometric
  /// queries on a SceneGraph. See SceneGraph::get_query_output_port().
  /// Refer to section @ref geometry_registration of this class's
  /// documentation for further details on collision geometry registration and
  /// connection with a SceneGraph.
  /// @throws std::exception if this system was not registered with a
  /// SceneGraph.
  const systems::InputPort<T>& get_geometry_query_input_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// SceneGraph.
  /// @throws std::exception if this system was not registered with a
  /// SceneGraph.
  const systems::OutputPort<T>& get_geometry_poses_output_port() const;
  /// @}

  /// Returns `true` if `this` %MultibodyPlant was registered with a
  /// SceneGraph.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize().
  bool geometry_source_is_registered() const {
    if (source_id_) {
      if (!is_finalized()) {
        DRAKE_DEMAND(scene_graph_ != nullptr);
      }
      return true;
    } else {
      return false;
    }
  }

  /// Given a geometry frame identifier, returns a pointer to the body
  /// associated with that id (nullptr if there is no such body).
  const Body<T>* GetBodyFromFrameId(geometry::FrameId frame_id) const {
    const auto it = frame_id_to_body_index_.find(frame_id);
    if (it == frame_id_to_body_index_.end()) return nullptr;
    return &internal_tree().get_body(it->second);
  }

  /// If the body with `body_index` has geometry registered with it, it returns
  /// the geometry::FrameId associated with it. Otherwise, it returns nullopt.
  /// @throws std::exception if called pre-finalize.
  optional<geometry::FrameId> GetBodyFrameIdIfExists(
      BodyIndex body_index) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    const auto it = body_index_to_frame_id_.find(body_index);
    if (it == body_index_to_frame_id_.end()) {
      return {};
    }
    return it->second;
  }

  /// If the body with `body_index` has geometry registered with it, it returns
  /// the geometry::FrameId associated with it. Otherwise this method throws
  /// an exception.
  /// @throws std::exception if no geometry has been registered with the body
  /// indicated by `body_index`.
  /// @throws std::exception if called pre-finalize.
  geometry::FrameId GetBodyFrameIdOrThrow(BodyIndex body_index) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    const auto it = body_index_to_frame_id_.find(body_index);
    if (it == body_index_to_frame_id_.end()) {
      throw std::logic_error(
          "Body '" + internal_tree().get_body(body_index).name() +
          "' does not have geometry registered with it.");
    }
    return it->second;
  }

  /// @name Actuation input
  ///
  /// The input vector of actuation values can be provided either as a single
  /// input port which describes the entire plant (in the case where only a
  /// single model instance has actuated dofs), or through multiple input ports
  /// which each provide the actuation values for a specific model instance.
  /// See AddJointActuator() and num_actuators().
  /// @{

  /// Returns a constant reference to the input port for external actuation for
  /// the case where only one model instance has actuated dofs.  This input
  /// port is a vector valued port, which can be set with
  /// JointActuator::set_actuation_vector().
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize(), if the model does not
  /// contain any actuators, or if multiple model instances have actuated dofs.
  const systems::InputPort<T>& get_actuation_input_port() const;

  /// Returns a constant reference to the input port for external actuation for
  /// a specific model instance.  This input port is a vector valued port, which
  /// can be set with JointActuator::set_actuation_vector().
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize().
  /// @throws std::exception if the model instance does not exist.
  const systems::InputPort<T>& get_actuation_input_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the vector-valued input port for applied
  /// generalized forces, and the vector will be added directly into `tau`
  /// (see @ref equations_of_motion). This vector is ordered using the same
  /// convention as the plant velocities: you can set the generalized forces
  /// that will be applied to model instance i using, e.g.,
  /// `SetVelocitiesInArray(i, model_forces, &force_array)`.
  /// @throws std::exception if called before Finalize().
  const systems::InputPort<T>& get_applied_generalized_force_input_port() const;

  /// Returns a constant reference to the input port for applying spatial
  /// forces to bodies in the plant. The data type for the port is an
  /// std::vector of ExternallyAppliedSpatialForce; any number of spatial forces
  /// can be applied to any number of bodies in the plant.
  const systems::InputPort<T>& get_applied_spatial_force_input_port() const;

  /// @}
  // Closes Doxygen section "Actuation input"

  /// @name Continuous state output
  ///
  /// Output ports are provided to access the continuous state of the whole
  /// plant and for individual model instances.
  /// @{

  /// Returns a constant reference to the output port for the full continuous
  /// state `x = [q v]` of the model.
  /// @pre Finalize() was already called on `this` plant.
  const systems::OutputPort<T>& get_continuous_state_output_port() const;

  /// Returns a constant reference to the output port for the continuous
  /// state of a specific model instance.
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if called before Finalize() or if the model
  /// instance does not have any state.
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_continuous_state_output_port(
      ModelInstanceIndex model_instance) const;
  /// @}
  // Closes Doxygen section "Continuous state output"

  /// Returns a constant reference to the output port of generalized contact
  /// forces for a specific model instance. This output port is only available
  /// when modeling the plant as a discrete system with periodic updates, see
  /// is_discrete().
  ///
  /// @pre Finalize() was already called on `this` plant.
  /// @throws std::exception if `this` plant is not modeled as a discrete system
  /// with periodic updates.
  /// @throws std::exception if called before Finalize() or if the model
  /// instance does not have any generalized velocities.
  /// @throws std::exception if the model instance does not exist.
  const systems::OutputPort<T>& get_generalized_contact_forces_output_port(
      ModelInstanceIndex model_instance) const;

  /// Returns a constant reference to the port that outputs ContactResults.
  /// @throws std::exception if `this` plant is not modeled as a discrete system
  /// with periodic updates.
  /// @throws std::exception if called pre-finalize, see Finalize().
  // TODO(amcastro-tri): report contact results for plants modeled as a
  // continuous system as well.
  const systems::OutputPort<T>& get_contact_results_output_port() const;

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    return internal_tree().world_body();
  }

  /// Returns a constant reference to the *world* frame.
  const BodyFrame<T>& world_frame() const {
    return internal_tree().world_frame();
  }

  /// Returns a constant reference to the body with unique index `body_index`.
  /// @throws std::exception if `body_index` does not correspond to a body in
  /// this model.
  const Body<T>& get_body(BodyIndex body_index) const {
    return internal_tree().get_body(body_index);
  }

  /// Returns a constant reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this model.
  const Joint<T>& get_joint(JointIndex joint_index) const {
    return internal_tree().get_joint(joint_index);
  }

  /// Returns a mutable reference to the joint with unique index `joint_index`.
  /// @throws std::runtime_error when `joint_index` does not correspond to a
  /// joint in this model.
  Joint<T>& get_mutable_joint(JointIndex joint_index) {
    return this->mutable_tree().get_mutable_joint(joint_index);
  }

  /// Returns a constant reference to the joint actuator with unique index
  /// `actuator_index`.
  /// @throws std::exception if `actuator_index` does not correspond to a joint
  /// actuator in this tree.
  const JointActuator<T>& get_joint_actuator(
      JointActuatorIndex actuator_index) const {
    return internal_tree().get_joint_actuator(actuator_index);
  }

  /// Returns a constant reference to the frame with unique index `frame_index`.
  /// @throws std::exception if `frame_index` does not correspond to a frame in
  /// this plant.
  const Frame<T>& get_frame(FrameIndex frame_index) const {
    return internal_tree().get_frame(frame_index);
  }

  /// Returns the name of a `model_instance`.
  /// @throws std::logic_error when `model_instance` does not correspond to a
  /// model in this model.
  const std::string& GetModelInstanceName(
      ModelInstanceIndex model_instance) const {
    return internal_tree().GetModelInstanceName(model_instance);
  }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize().
  /// @see Finalize().
  bool is_finalized() const { return internal_tree().topology_is_valid(); }

  /// Returns `true` if @p body is anchored (i.e. the kinematic path between
  /// @p body and the world only contains weld joints.)
  /// @throws std::exception if called pre-finalize.
  bool IsAnchored(const Body<T>& body) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return internal_tree().get_topology().IsBodyAnchored(body.index());
  }

  /// This method must be called after all elements in the model (joints,
  /// bodies, force elements, constraints, etc.) are added and before any
  /// computations are performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to enable computations at a
  /// later stage.
  ///
  /// If the finalize stage is successful, the topology of this %MultibodyPlant
  /// is valid, meaning that the topology is up-to-date after this call.
  /// No more multibody elements can be added after a call to Finalize().
  ///
  /// At Finalize(), state and input/output ports for `this` plant are declared.
  /// If `this` plant registered geometry with a SceneGraph, input and
  /// output ports to enable communication with that SceneGraph are declared
  /// as well.
  ///
  /// If geometry has been registered on a SceneGraph instance, that instance
  /// must be provided to the Finalize() method so that any geometric
  /// implications of the finalization process can be appropriately handled.
  ///
  /// @see is_finalized().
  ///
  /// @throws std::logic_error if
  ///          1. the %MultibodyPlant has already been finalized or
  ///          3. a different scene_graph instance is provided than the one
  ///             for which this plant is a geometry source.
  void Finalize(geometry::SceneGraph<T>* scene_graph = nullptr);

  /// The time step (or period) used to model `this` plant as a discrete system
  /// with periodic updates. Returns 0 (zero) if the plant is modeled as a
  /// continuous system.
  /// This property of the plant is specified at construction and therefore this
  /// query can be performed either pre- or post- finalize, see Finalize().
  /// @see MultibodyPlant::MultibodyPlant(double)
  double time_step() const { return time_step_; }

  /// @anchor mbp_penalty_method
  /// @name Contact by penalty method
  ///
  /// Currently %MultibodyPlant uses a rigid contact model that is, bodies in
  /// the model are infinitely stiff or ideal rigid bodies. Therefore, the
  /// mathematical description of the rigid contact model needs to include
  /// non-penetration constraints among bodies in the formulation. There are
  /// several numerical methods to impose and solve these constraints.
  /// In a penalty method approach, we allow for a certain amount of
  /// interpenetration and we compute contact forces according to a simple law
  /// of the form: <pre>
  ///   fₙ = k(1+dẋ)x
  /// </pre>
  /// where the normal contact force `fₙ` is made a continuous function of the
  /// penetration distance x between the bodies (defined to be
  /// positive when the bodies are in contact) and the penetration distance
  /// rate ẋ (with ẋ > 0 meaning the penetration distance is increasing and
  /// therefore the interpenetration between the bodies is also increasing).
  /// k and d are the penalty method coefficients for stiffness and damping.
  /// These are ad-hoc parameters which need to be tuned as a trade-off between:
  /// - The accuracy of the numerical approximation to rigid contact, which
  ///   requires a stiffness that approaches infinity, and
  /// - the computational cost of the numerical integration, which will
  ///   require smaller time steps for stiffer systems.
  ///
  /// There is no exact procedure for choosing these coefficients, and
  /// estimating them manually can be cumbersome since in general they will
  /// depend on the scale of the problem including masses, speeds and even
  /// body sizes. However, %MultibodyPlant aids the estimation of these
  /// coefficients using a heuristic function based on a user-supplied
  /// "penetration allowance", see set_penetration_allowance(). The penetration
  /// allowance is a number in meters that specifies the order of magnitude of
  /// the average penetration between bodies in the system that the user is
  /// willing to accept as reasonable for the problem being solved. For
  /// instance, in the robotics manipulation of ordinary daily objects the user
  /// might set this number to 1 millimeter. However, the user might want to
  /// increase it for the simulation of heavy walking robots for which an
  /// allowance of 1 millimeter would result in a very stiff system.
  ///
  /// As for the damping coefficient in the simple law above, %MultibodyPlant
  /// chooses the damping coefficient d to model inelastic collisions and
  /// therefore sets it so that the penetration distance x behaves as a
  /// critically damped oscillator. That is, at the limit of ideal rigid contact
  /// (very stiff penalty coefficient k or equivalently the penetration
  /// allowance goes to zero), this method behaves as a unilateral constraint on
  /// the penetration distance, which models a perfect inelastic collision. For
  /// most applications, such as manipulation and walking, this is the desired
  /// behavior.
  ///
  /// When set_penetration_allowance() is called, %MultibodyPlant will estimate
  /// reasonable penalty method coefficients as a function of the input
  /// penetration allowance. Users will want to run their simulation a number of
  /// times and asses they are satisfied with the level of inter-penetration
  /// actually observed in the simulation; if the observed penetration is too
  /// large, the user will want to set a smaller penetration allowance. If the
  /// system is too stiff and the time integration requires very small time
  /// steps while at the same time the user can afford larger
  /// inter-penetrations, the user will want to increase the penetration
  /// allowance. Typically, the observed penetration will be
  /// proportional to the penetration allowance. Thus scaling the penetration
  /// allowance by say a factor of 0.5, would typically results in
  /// inter-penetrations being reduced by the same factor of 0.5.
  /// In summary, users should choose the largest penetration allowance that
  /// results in inter-penetration levels that are acceptable for the particular
  /// application (even when in theory this penetration should be zero for
  /// perfectly rigid bodies.)
  ///
  /// For a given penetration allowance, the contact interaction that takes two
  /// bodies with a non-zero approaching velocity to zero approaching velocity,
  /// takes place in a finite amount of time (for ideal rigid contact this time
  /// is zero.) A good estimate of this time period is given by a call to
  /// get_contact_penalty_method_time_scale(). Users might want to query this
  /// value to either set the maximum time step in error-controlled time
  /// integration or to set the time step for fixed time step integration.
  /// As a guidance, typical fixed time step integrators will become unstable
  /// for time steps larger than about a tenth of this time scale.
  /// @{

  /// Sets the penetration allowance used to estimate the coefficients in the
  /// penalty method used to impose non-penetration among bodies. Refer to the
  /// section @ref mbp_penalty_method "Contact by penalty method" for further
  /// details.
  void set_penetration_allowance(double penetration_allowance = 0.001);

  /// Returns a time-scale estimate `tc` based on the requested penetration
  /// allowance δ set with set_penetration_allowance().
  /// For the penalty method in use to enforce non-penetration, this time scale
  /// relates to the time it takes the relative normal velocity between two
  /// bodies to go to zero. This time scale `tc` is artificially introduced by
  /// the penalty method and goes to zero in the limit to ideal rigid contact.
  /// Since numerical integration methods for continuum systems must be able to
  /// resolve a system's dynamics, the time step used by an integrator must in
  /// general be much smaller than the time scale `tc`. How much smaller will
  /// depend on the details of the problem and the convergence characteristics
  /// of the integrator and should be tuned appropriately.
  /// Another factor to take into account for setting up the simulation's time
  /// step is the speed of the objects in your simulation. If `vn` represents a
  /// reference velocity scale for the normal relative velocity between bodies,
  /// the new time scale `tn = δ / vn` represents the time it would take for the
  /// distance between two bodies approaching with relative normal velocity `vn`
  /// to decrease by the penetration_allowance δ. In this case a user should
  /// choose a time step for simulation that can resolve the smallest of the two
  /// time scales `tc` and `tn`.
  double get_contact_penalty_method_time_scale() const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return penalty_method_contact_parameters_.time_scale;
  }
  /// @}

  /// @anchor mbp_stribeck_model
  /// @name Stribeck model of friction
  ///
  /// Currently %MultibodyPlant uses the Stribeck approximation to model dry
  /// friction. The Stribeck model of friction is an approximation to Coulomb's
  /// law of friction that allows using continuous time integration without the
  /// need to specify complementarity constraints. While this results in a
  /// simpler model immediately tractable with standard numerical methods for
  /// integration of ODE's, it often leads to stiff dynamics that require
  /// an explicit integrator to take very small time steps. It is therefore
  /// recommended to use error controlled integrators when using this model.
  /// See @ref tangent_force for a detailed discussion of the Stribeck model.
  /// @{

  /// Sets the stiction tolerance `v_stiction` for the Stribeck model, where
  /// `v_stiction` must be specified in m/s (meters per second.)
  /// `v_stiction` defaults to a value of 1 millimeter per second.
  /// @throws std::exception if `v_stiction` is non-positive.
  void set_stiction_tolerance(double v_stiction = 0.001) {
    stribeck_model_.set_stiction_tolerance(v_stiction);
    // We allow calling this method post-finalize. Therefore, if the plant is
    // modeled as a discrete system, we must update the solver's stiction
    // parameter. Pre-Finalize the solver is not yet created and therefore we
    // check for nullptr.
    if (is_discrete() && implicit_stribeck_solver_ != nullptr) {
      ImplicitStribeckSolverParameters solver_parameters =
          implicit_stribeck_solver_->get_solver_parameters();
      solver_parameters.stiction_tolerance =
          stribeck_model_.stiction_tolerance();
      implicit_stribeck_solver_->set_solver_parameters(solver_parameters);
    }
  }
  /// @}

  /// Evaluates all point pairs of contact for a given state of the model stored
  /// in `context`.
  /// Each entry in the returned vector corresponds to a single point pair
  /// corresponding to two interpenetrating bodies A and B. The size of the
  /// returned vector corresponds to the total number of contact penetration
  /// pairs. If no geometry was registered, the output vector is empty.
  /// @see PenetrationAsPointPair for further details on the returned data.
  /// @throws std::exception if called pre-finalize. See Finalize().
  const std::vector<geometry::PenetrationAsPointPair<T>>&
  EvalPointPairPenetrations(const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return this->get_cache_entry(cache_indexes_.point_pairs)
        .template Eval<std::vector<geometry::PenetrationAsPointPair<T>>>(
            context);
  }

  /// Sets the `state` so that generalized positions and velocities are zero.
  /// @throws std::exception if called pre-finalize. See Finalize().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    CheckValidState(state);
    internal_tree().SetDefaultState(context, state);
  }

  /// Assigns random values to all elements of the state, by drawing samples
  /// independently for each joint/floating-base (coming soon: and then
  /// solving a mathematical program to "project" these samples onto the
  /// registered system constraints).
  ///
  /// @see @ref stochastic_systems
  void SetRandomState(const systems::Context<T>& context,
                      systems::State<T>* state,
                      RandomGenerator* generator) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    CheckValidState(state);
    internal_tree().SetRandomState(context, state, generator);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // These APIs using Isometry3 will be deprecated soon with the resolution of
  // #9865. Right now we offer them for backwards compatibility.

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  void SetFreeBodyPose(systems::Context<T>* context, const Body<T>& body,
                       const Isometry3<T>& X_WB) const {
    SetFreeBodyPose(context, body, math::RigidTransform<T>(X_WB));
  }

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  void SetFreeBodyPose(const systems::Context<T>& context,
                       systems::State<T>* state, const Body<T>& body,
                       const Isometry3<T>& X_WB) const {
    SetFreeBodyPose(context, state, body, math::RigidTransform<T>(X_WB));
  }

  // Allows having a non-empty X_PF isometry and a nullopt X_BM.
  template <template <typename> class JointType, typename... Args>
  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  const JointType<T>& AddJoint(const std::string& name, const Body<T>& parent,
                               const Isometry3<double>& X_PF,
                               const Body<T>& child,
                               const optional<Isometry3<double>>& X_BM,
                               Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();

    const math::RigidTransform<double> X_PF_rt(X_PF);
    const optional<math::RigidTransform<double>> X_BM_rt =
        X_BM ? optional<math::RigidTransform<T>>(math::RigidTransform<T>(*X_BM))
             : nullopt;

    return this->mutable_tree().template AddJoint<JointType>(
        name, parent, X_PF_rt, child, X_BM_rt, std::forward<Args>(args)...);
  }

  // Allows having a nullopt X_PF and a non-empty X_BM isometry.
  template <template <typename> class JointType, typename... Args>
  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  const JointType<T>& AddJoint(const std::string& name, const Body<T>& parent,
                               const optional<Isometry3<double>>& X_PF,
                               const Body<T>& child,
                               const Isometry3<double>& X_BM,
                               Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();

    optional<math::RigidTransform<double>> X_PF_rt =
        X_PF ? optional<math::RigidTransform<T>>(math::RigidTransform<T>(*X_PF))
             : nullopt;
    const math::RigidTransform<double> X_BM_rt(X_BM);

    return this->mutable_tree().template AddJoint<JointType>(
        name, parent, X_PF_rt, child, X_BM_rt, std::forward<Args>(args)...);
  }

  DRAKE_DEPRECATED(
      "2019-07-01",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  const WeldJoint<T>& WeldFrames(
      const Frame<T>& A, const Frame<T>& B,
      const Isometry3<double>& X_AB) {
    return WeldFrames(A, B, math::RigidTransform<double>(X_AB));
  }

  DRAKE_DEPRECATED(
      "2019-07-01",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  void SetFreeBodyPoseInWorldFrame(systems::Context<T>* context,
                                   const Body<T>& body,
                                   const Isometry3<T>& X_WB) const {
    SetFreeBodyPoseInWorldFrame(context, body, math::RigidTransform<T>(X_WB));
  }

  DRAKE_DEPRECATED(
      "2019-07-01",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  void SetFreeBodyPoseInAnchoredFrame(systems::Context<T>* context,
                                      const Frame<T>& frame_F,
                                      const Body<T>& body,
                                      const Isometry3<T>& X_FB) const {
    SetFreeBodyPoseInAnchoredFrame(context, frame_F, body,
                                   math::RigidTransform<T>(X_FB));
  }

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const geometry::IllustrationProperties& properties,
      geometry::SceneGraph<T>* scene_graph = nullptr) {
    return RegisterVisualGeometry(body, math::RigidTransform<double>(X_BG),
                                  shape, name, properties, scene_graph);
  }

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const Vector4<double>& diffuse_color,
      geometry::SceneGraph<T>* scene_graph = nullptr) {
    return RegisterVisualGeometry(body, math::RigidTransform<double>(X_BG),
                                  shape, name, diffuse_color, scene_graph);
  }

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  geometry::GeometryId RegisterVisualGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      geometry::SceneGraph<T>* scene_graph = nullptr) {
    return RegisterVisualGeometry(body, math::RigidTransform<double>(X_BG),
                                  shape, name, scene_graph);
  }

  DRAKE_DEPRECATED(
      "2019-06-15",
      "This Isometry3 overload will be removed pending the resolution of "
      "#9865. Use the RigidTransform overload instead.")
  geometry::GeometryId RegisterCollisionGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      const geometry::Shape& shape, const std::string& name,
      const CoulombFriction<double>& coulomb_friction,
      geometry::SceneGraph<T>* scene_graph = nullptr) {
    return RegisterCollisionGeometry(body, math::RigidTransform<double>(X_BG),
                                     shape, name, coulomb_friction,
                                     scene_graph);
  }
#endif

  using internal::MultibodyTreeSystem<T>::is_discrete;
  using internal::MultibodyTreeSystem<T>::EvalPositionKinematics;
  using internal::MultibodyTreeSystem<T>::EvalVelocityKinematics;

 private:
  using internal::MultibodyTreeSystem<T>::internal_tree;

  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

  // Friend class to facilitate testing.
  friend class MultibodyPlantTester;

  // This struct stores in one single place all indexes related to
  // MultibodyPlant specific cache entries. These are initialized at Finalize()
  // when the plant declares its cache entries.
  struct CacheIndexes {
    systems::CacheIndex contact_jacobians;
    systems::CacheIndex contact_results;
    systems::CacheIndex implicit_stribeck_solver_results;
    systems::CacheIndex point_pairs;
  };

  // Constructor to bridge testing from MultibodyTree to MultibodyPlant.
  // WARNING: This may *not* result in a plant with a valid state. Use
  // sparingly to test forwarding methods when the overhead is high to
  // reproduce the testing (e.g. benchmarks).
  explicit MultibodyPlant(
      std::unique_ptr<internal::MultibodyTree<T>> tree_in,
      double time_step = 0);

  // Helper method for throwing an exception within public methods that should
  // not be called post-finalize. The invoking method should pass its name so
  // that the error message can include that detail.
  void ThrowIfFinalized(const char* source_method) const;

  // Helper method for throwing an exception within public methods that should
  // not be called pre-finalize. The invoking method should pass it's name so
  // that the error message can include that detail.
  void ThrowIfNotFinalized(const char* source_method) const;

  // Helper method that is used to finalize the plant's internals after
  // MultibodyTree::Finalize() was called.
  void FinalizePlantOnly();

  // MemberSceneGraph is an alias for SceneGraph<T>, except when T = Expression.
  struct SceneGraphStub;
  using MemberSceneGraph = typename std::conditional<
      std::is_same<T, symbolic::Expression>::value,
      SceneGraphStub, geometry::SceneGraph<T>>::type;

  // Returns the SceneGraph that pre-Finalize geometry operations should
  // interact with.  In most cases, that will be whatever the user has passed
  // into RegisterAsSourceForSceneGraph.  However, when T = Expression, the
  // result will be a stub type instead.  (We can get rid of the stub once
  // SceneGraph supports symbolic::Expression.)
  MemberSceneGraph& member_scene_graph();

  // Helper to check when a deprecated user-provided `scene_graph` pointer is
  // passed in via public API (aside form `RegisterAsSourceForSceneGraph`).
  // @throws std::logic_error if `scene_graph` is non-null (non-default) and
  // either no scene graph is registered or `scene_graph` is not the same as
  // the registered instance.
  void CheckUserProvidedSceneGraph(
      const geometry::SceneGraph<T>* scene_graph) const;

  // Checks that the provided State is consistent with this plant.
  void CheckValidState(const systems::State<T>*) const;

  // Helper method to apply collision filters based on body-adjacency. By
  // default, we don't consider collisions between geometries affixed to
  // bodies connected by a joint.
  void FilterAdjacentBodies();

  // For discrete models, MultibodyPlant uses a penalty method to impose joint
  // limits. In this penalty method a force law of the form:
  //   τ = -k(q - qᵤ) - cv if q > qᵤ
  //   τ = -k(q - qₗ) - cv if q < qₗ
  // is used to limit the position q to be within the lower/upper limits
  // (qₗ, qᵤ).
  // The penalty parameters k (stiffness) and c (damping) are estimated using
  // a harmonic oscillator model of the form ẍ + 2ζω₀ ẋ + ω₀² x = 0, with
  // x = (q - qᵤ) near the upper limit when q > qᵤ and x = (q - qₗ) near the
  // lower limit when q < qₗ and where ω₀² = k / m̃ is the characteristic
  // numerical stiffness frequency and m̃ is an inertia term that for prismatic
  // joints reduces to a simple function of the mass of the bodies adjancent to
  // a particular joint. For revolute joints m̃ relates to the rotational inertia
  // of the adjacent bodies to a joint. See the implementation notes for further
  // details. Both ω₀ and ζ are non-negative numbers.
  // The characteristic frequency ω₀ is entirely a function the time step of the
  // discrete model so that, from a stability analysis of the simplified
  // harmonic oscillator model, we guarantee the resulting time stepping is
  // stable. That is, the numerical stiffness of the method is such that it
  // corresponds to the largest penalty parameter (smaller violation errors)
  // that still guarantees stability.
  void SetUpJointLimitsParameters();

  // This is a *temporary* method to eliminate visual geometries from collision
  // while we wait for geometry roles to be introduced.
  // TODO(SeanCurtis-TRI): Remove this when geometry roles are introduced.
  void ExcludeCollisionsWithVisualGeometry();

  // No inputs implies no feedthrough; this makes it explicit.
  // TODO(amcastro-tri): add input ports for actuators.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  // Helper method to declare state, cache entries, and ports after Finalize().
  void DeclareStateCacheAndPorts();

  // Declare the system-level cache entries specific to MultibodyPlant.
  void DeclareCacheEntries();

  // Helper method to assemble actuation input vector from the appropriate
  // ports.
  VectorX<T> AssembleActuationInput(
      const systems::Context<T>& context) const;

  // Implements the system dynamics according to this class's documentation.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // If the plant is modeled as a discrete system with periodic updates (see
  // is_discrete()), this method computes the periodic updates of the state
  // using a semi-explicit Euler strategy, that is:
  //   vⁿ⁺¹ = vⁿ + dt v̇ⁿ
  //   qⁿ⁺¹ = qⁿ + dt N(qⁿ) vⁿ⁺¹
  // This semi-explicit update inherits some of the nice properties of the
  // semi-implicit Euler scheme (which uses v̇ⁿ⁺¹ for the v updated instead) when
  // there are no velocity-dependent forces (including Coriolis and gyroscopic
  // terms). The semi-implicit Euler scheme is a symplectic integrator, which
  // for a Hamiltonian system has the nice property of nearly conserving energy
  // (in many cases we can write a "modified energy functional" which can be
  // shown to be exactly conserved and to be within O(dt) of the real energy of
  // the mechanical system.)
  // TODO(amcastro-tri): Update this docs when contact is added.
  void DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<T>& context0,
      const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>& events,
      drake::systems::DiscreteValues<T>* updates) const override;

  // Helper method used within DoCalcDiscreteVariableUpdates() to update
  // generalized velocities from previous step value v0 to next step value v.
  // This helper uses num_substeps within a time interval of duration dt
  // to perform the update using a step size dt_substep = dt/num_substeps.
  // During the time span dt the problem data M, Jn, Jt and minus_tau, are
  // approximated to be constant, a first order approximation.
  ImplicitStribeckSolverResult SolveUsingSubStepping(
      int num_substeps,
      const MatrixX<T>& M0, const MatrixX<T>& Jn, const MatrixX<T>& Jt,
      const VectorX<T>& minus_tau,
      const VectorX<T>& stiffness, const VectorX<T>& damping,
      const VectorX<T>& mu,
      const VectorX<T>& v0, const VectorX<T>& phi0) const;

  // This method uses the time stepping method described in
  // ImplicitStribeckSolver to advance the model's state stored in
  // `context0` taking a time step of size time_step().
  // Contact forces and velocities are computed and stored in `results`. See
  // ImplicitStribeckSolverResults for further details on the returned data.
  void CalcImplicitStribeckResults(
      const drake::systems::Context<T>& context0,
      internal::ImplicitStribeckSolverResults<T>* results) const;

  // Eval version of the method CalcImplicitStribeckResults().
  const internal::ImplicitStribeckSolverResults<T>& EvalImplicitStribeckResults(
      const systems::Context<T>& context) const {
    return this
        ->get_cache_entry(cache_indexes_.implicit_stribeck_solver_results)
        .template Eval<internal::ImplicitStribeckSolverResults<T>>(context);
  }

  // Helper method to fill in the ContactResults given the current context.
  // If cached contact solver results are not up-to-date with `context`,
  // they'll be  recomputed, see EvalImplicitStribeckResults(). The solver
  // results are then used to compute contact results into `contacts`.
  void CalcContactResults(const systems::Context<T>& context,
                          ContactResults<T>* contacts) const;

  // Eval version of the method CalcContactResults().
  const ContactResults<T>& EvalContactResults(
      const systems::Context<T>& context) const {
    return this->get_cache_entry(cache_indexes_.contact_results)
        .template Eval<ContactResults<T>>(context);
  }

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  void AddAppliedExternalSpatialForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Helper method to register geometry for a given body, either visual or
  // collision. The registration includes:
  // 1. Register a frame for this body if not already done so. The body gets
  //    associated with a FrameId.
  // 2. Register geometry for the corresponding FrameId. This associates a
  //    GeometryId with the body FrameId.
  // This assumes:
  // 1. Finalize() was not called on `this` plant.
  // 2. RegisterAsSourceForSceneGraph() was called on `this` plant.
  // 3. `scene_graph` points to the same SceneGraph instance previously
  //    passed to RegisterAsSourceForSceneGraph().
  geometry::GeometryId RegisterGeometry(
      const Body<T>& body, const math::RigidTransform<double>& X_BG,
      const geometry::Shape& shape,
      const std::string& name);

  bool body_has_registered_frame(const Body<T>& body) const {
    return body_index_to_frame_id_.find(body.index()) !=
        body_index_to_frame_id_.end();
  }

  // Helper to retrieve a constant reference to the state vector from context.
  const systems::BasicVector<T>& GetStateVector(
      const systems::Context<T>& context) const;

  // Calc method for the continuous state vector output port.
  void CopyContinuousStateOut(
      const systems::Context<T>& context, systems::BasicVector<T>* state) const;

  // Calc method for the per-model-instance continuous state vector output
  // port.
  void CopyContinuousStateOut(
      ModelInstanceIndex model_instance,
      const systems::Context<T>& context, systems::BasicVector<T>* state) const;

  // Calc method to output per model instance vector of generalized contact
  // forces.
  void CopyGeneralizedContactForcesOut(
      ModelInstanceIndex model_instance, const systems::Context<T>& context,
      systems::BasicVector<T>* tau_vector) const;

  // Helper method to declare output ports used by this plant to communicate
  // with a SceneGraph.
  void DeclareSceneGraphPorts();

  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  void CopyContactResultsOutput(
      const systems::Context<T>& context,
      ContactResults<T>* contact_results) const;

  // Helper to evaluate if a GeometryId corresponds to a collision model.
  bool is_collision_geometry(geometry::GeometryId id) const {
    return geometry_id_to_collision_index_.count(id) > 0;
  }

  // Helper method to compute penetration point pairs for a given `context`.
  // Having this as a separate method allows us to control specializations for
  // different scalar types.
  std::vector<geometry::PenetrationAsPointPair<T>>
  CalcPointPairPenetrations(const systems::Context<T>& context) const;

  // This helper method combines the friction properties for each pair of
  // contact points in `point_pairs` according to
  // CalcContactFrictionFromSurfaceProperties().
  // The i-th entry in the returned std::vector corresponds to the combined
  // friction properties for the i-th point pair in `point_pairs`.
  std::vector<CoulombFriction<double>> CalcCombinedFrictionCoefficients(
      const std::vector<geometry::PenetrationAsPointPair<T>>&
      point_pairs) const;

  // (Advanced) Helper method to compute contact forces in the normal direction
  // using a penalty method.
  void CalcAndAddContactForcesByPenaltyMethod(
      const systems::Context<T>& context,
      const internal::PositionKinematicsCache<T>& pc,
      const internal::VelocityKinematicsCache<T>& vc,
      const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs,
      std::vector<SpatialForce<T>>* F_BBo_W_array) const;

  // Helper method to add the contribution of external actuation forces to the
  // set of multibody `forces`. External actuation is applied through the
  // plant's input ports.
  void AddJointActuationForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Helper method to apply penalty forces that enforce joint limits.
  // At each joint with joint limits this penalty method applies a force law of
  // the form:
  //   τ = min(-k(q - qᵤ) - cv, 0) if q > qᵤ
  //   τ = max(-k(q - qₗ) - cv, 0) if q < qₗ
  // is used to limit the position q to be within the lower/upper limits
  // (qₗ, qᵤ).
  // The penalty parameters k (stiffness) and c (damping) are estimated using
  // a harmonic oscillator model within SetUpJointLimitsParameters().
  void AddJointLimitsPenaltyForces(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const;

  // Given a set of point pairs in `point_pairs_set`, this method computes the
  // normal velocities Jacobian Jn(q) and the tangential velocities Jacobian
  // Jt(q).
  //
  // The normal velocities Jacobian Jn(q) is defined such that:
  //   vn = Jn(q) v
  // where the i-th component of vn corresponds to the "separation velocity"
  // for the i-th point pair in the set. The i-th separation velocity is defined
  // positive for when the depth in the i-th point pair (
  // PenetrationAsPointPair::depth) is decreasing. Since for contact problems
  // the (positive) depth in PenetrationAsPointPair is defined so that it
  // corresponds to interpenetrating body geometries, a positive separation
  // velocity corresponds to bodies moving apart.
  //
  // The tangential velocities Jacobian Jt(q) is defined such that:
  //   vt = Jt(q) v
  // where v ∈ ℝⁿᵛ is the vector of generalized velocities, Jt(q) is a matrix
  // of size 2⋅nc×nv and vt is a vector of size 2⋅nc.
  // This method defines a contact frame C with orientation R_WC in the world
  // frame W such that Cz_W = nhat_BA_W, the normal direction in the point
  // pair (PenetrationAsPointPair::nhat_BA_W).
  // With this definition, the first two columns of R_WC correspond to
  // orthogonal versors Cx = that1 and Cy = that2 which span the tangent plane
  // to nhat_BA_W.
  // vt is defined such that its i-th and (i+1)-th entries correspond to
  // relative velocity of the i-th point pair in these two orthogonal
  // directions. That is:
  //   vt(2 * i)     = vx_AB_C = Cx ⋅ v_AB
  //   vt(2 * i + 1) = vy_AB_C = Cy ⋅ v_AB
  //
  // If the optional output argument R_WC_set is provided with a valid non
  // nullptr vector, on output the i-th entry of R_WC_set will contain the
  // orientation R_WC of the i-th point pair in the set.
  void CalcNormalAndTangentContactJacobians(
      const systems::Context<T>& context,
      const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs_set,
      MatrixX<T>* Jn, MatrixX<T>* Jt,
      std::vector<math::RotationMatrix<T>>* R_WC_set = nullptr) const;

  // Evaluates the contact Jacobians for the given state of the plant stored in
  // `context`.
  // This method first evaluates the point pair penetrations in the system for
  // the given `context`, see EvalPointPairPenetrations(). For each penetration
  // pair involving bodies A and B, a contact frame C is defined by the
  // rotation matrix `R_WC = [Cx_W, Cy_W, Cz_W]` where `Cz_W = nhat_BA_W`
  // equals the normal vector pointing from body B into body A, expressed in
  // the world frame W. See PenetrationAsPointPair for further details on the
  // definition of each contact pair. Versors `Cx_W` and `Cy_W` constitute a
  // basis of the plane normal to `Cz_W` and are arbitrarily chosen. The
  // contact frame basis can be accessed in the results, see
  // ContactJacobians::R_WC_list. Further, for each contact pair evaluated,
  // this method computes the Jacobians `Jn` and `Jt`. With the vector of
  // generalized velocities v of size `nv` and `nc` the number of contact
  // pairs;
  //   - `Jn` is a matrix of size `nc x nv` such that `vn = Jn⋅v` is the
  //     separation speed for each contact point, defined to be positive when
  //     bodies are moving away at the contact.
  //   - `Jt` is a matrix of size `2⋅nc x nv` such that `vt = Jt⋅v`
  //     concatenates the tangential components of the relative velocity vector
  //     `v_AcBc` in the frame C of contact. That is, for the k-th contact
  //     pair, `vt.segment<2>(2 * ik)` stores the components of `v_AcBc` in the
  //     `Cx` and `Cy` directions.
  //
  // If no geometry was registered or if `nc = 0`, ContactJacobians holds empty
  // results.
  //
  // See ContactJacobians for specifics on the returned data storage.
  // This method throws std::exception if called pre-finalize. See Finalize().
  const internal::ContactJacobians<T>& EvalContactJacobians(
      const systems::Context<T>& context) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return this->get_cache_entry(cache_indexes_.contact_jacobians)
        .template Eval<internal::ContactJacobians<T>>(context);
  }

  // The gravity field force element.
  optional<const UniformGravityFieldElement<T>*> gravity_field_;

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  optional<geometry::SourceId> source_id_{nullopt};

  // Frame Id's for each body in the model:
  // Not all bodies need to be in this map.

  // Map provided at construction that tells how bodies (referenced by name),
  // map to frame ids.
  std::unordered_map<std::string, geometry::FrameId> body_name_to_frame_id_;

  // This struct contains the parameters to compute forces to enforce
  // no-interpenetration between bodies by a penalty method.
  struct ContactByPenaltyMethodParameters {
    // Penalty method coefficients used to compute contact forces.
    // TODO(amcastro-tri): consider having these per body. That would allow us
    // for instance to calibrate the stiffness at the fingers (stiffness related
    // to the weight of the objects being manipulated) of a walking robot (
    // stiffness related to the weight of the entire robot) with the same
    // penetration allowance.
    double stiffness{0};
    double damping{0};
    // An estimated time scale in which objects come to a relative stop during
    // contact.
    double time_scale{-1.0};
    // Acceleration of gravity in the model. Used to estimate penalty method
    // constants from a static equilibrium analysis.
    optional<double> gravity;
  };
  ContactByPenaltyMethodParameters penalty_method_contact_parameters_;

  // Stribeck model of friction.
  class StribeckModel {
   public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(StribeckModel)

    /// Creates an uninitialized Stribeck model with an invalid value (negative)
    /// of the stiction tolerance.
    StribeckModel() = default;

    /// Computes the friction coefficient based on the tangential *speed*
    /// `speed_BcAc` of the contact point `Ac` on A relative to the
    /// contact point `Bc` on B. That is, `speed_BcAc = ‖vt_BcAc‖`, where
    /// `vt_BcAc` is the tangential component of the velocity `v_BcAc` of
    /// contact point `Ac` relative to point `Bc`.
    ///
    /// See contact_model_doxygen.h @section tangent_force for details.
    T ComputeFrictionCoefficient(
        const T& speed_BcAc,
        const CoulombFriction<double>& friction) const;

    /// Evaluates an S-shaped quintic curve, f(x), mapping the domain [0, 1] to
    /// the range [0, 1] where f(0) = f''(0) = f''(1) = f'(0) = f'(1) = 0 and
    /// f(1) = 1.
    static T step5(const T& x);

    /// Sets the stiction tolerance `v_stiction` for the Stribeck model, where
    /// `v_stiction` must be specified in m/s (meters per second.)
    /// @throws std::exception if `v_stiction` is non-positive.
    void set_stiction_tolerance(double v_stiction) {
      DRAKE_THROW_UNLESS(v_stiction > 0);
      v_stiction_tolerance_ = v_stiction;
      inv_v_stiction_tolerance_ = 1.0 / v_stiction;
    }

    /// Returns the value of the stiction tolerance for `this` %MultibodyPlant.
    /// It returns a negative value when the stiction tolerance has not been set
    /// previously with set_stiction_tolerance().
    double stiction_tolerance() const { return v_stiction_tolerance_; }

   private:
    // Stiction velocity tolerance for the Stribeck model.
    // A negative value indicates it was not properly initialized.
    double v_stiction_tolerance_{-1};
    // Note: this is the *inverse* of the v_stiction_tolerance_ parameter to
    // optimize for the division.
    // A negative value indicates it was not properly initialized.
    double inv_v_stiction_tolerance_{-1};
  };
  StribeckModel stribeck_model_;

  // This structure aids in the bookkeeping of parameters associated with joint
  // limits and the penalty method parameters used to enforce them.
  struct JointLimitsParameters {
    // list of joints that have limits. These are all single-dof joints.
    std::vector<JointIndex> joints_with_limits;
    // Position lower/upper bounds for each joint in joints_with_limits. The
    // Units depend on the particular joint type. For instance, radians for
    // RevoluteJoint or meters for PrismaticJoint.
    std::vector<double> lower_limit;
    std::vector<double> upper_limit;
    // Penalty parameters. These are defined in accordance to the penalty force
    // internally implemented by MultibodyPlant in
    // AddJointLimitsPenaltyForces().
    std::vector<double> stiffness;
    std::vector<double> damping;
  } joint_limits_parameters_;

  // Iteration order on this map DOES matter, and therefore we use an std::map.
  std::map<BodyIndex, geometry::FrameId> body_index_to_frame_id_;

  // Data to get back from a SceneGraph-reported frame id to its associated
  // body.
  std::unordered_map<geometry::FrameId, BodyIndex> frame_id_to_body_index_;

  // Map from GeometryId to BodyIndex. During contact queries, it allows to find
  // out to which body a given geometry corresponds to.
  std::unordered_map<geometry::GeometryId, BodyIndex>
      geometry_id_to_body_index_;

  // Maps a GeometryId with a visual index. This allows, for instance, to find
  // out visual properties for a given geometry.
  // TODO(amcastro-tri): verify insertions were correct once visual_index gets
  // used with the landing of visual properties in SceneGraph.
  std::unordered_map<geometry::GeometryId, int> geometry_id_to_visual_index_;

  // Per-body arrays of visual geometries indexed by BodyIndex.
  // That is, visual_geometries_[body_index] corresponds to the array of visual
  // geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> visual_geometries_;

  // Per-body arrays of collision geometries indexed by BodyIndex.
  // That is, collision_geometries_[body_index] corresponds to the array of
  // collision geometries for body with index body_index.
  std::vector<std::vector<geometry::GeometryId>> collision_geometries_;

  // Maps a GeometryId with a collision index. This allows, for instance, to
  // find out collision properties (such as friction coefficient) for a given
  // geometry.
  std::unordered_map<geometry::GeometryId, int> geometry_id_to_collision_index_;

  // Friction coefficients ordered by collision index.
  // See geometry_id_to_collision_index_.
  std::vector<CoulombFriction<double>> default_coulomb_friction_;

  // Port handles for geometry:
  systems::InputPortIndex geometry_query_port_;
  systems::OutputPortIndex geometry_pose_port_;

  // For geometry registration with a GS, we save a pointer to the GS instance
  // on which this plants calls RegisterAsSourceForSceneGraph(). This will be
  // set to `nullptr` after finalization, to mirror constraints presented by
  // scalar conversion (where we cannot easily obtain a reference to the
  // scalar-converted scene graph).
  geometry::SceneGraph<T>* scene_graph_{nullptr};

  // Input/Output port indexes:
  // A vector containing actuation ports for each model instance indexed by
  // ModelInstanceIndex.
  std::vector<systems::InputPortIndex> instance_actuation_ports_;

  // If only one model instance has actuated dofs, remember it here.  If
  // multiple instances have actuated dofs, this index will not be valid.
  ModelInstanceIndex actuated_instance_;

  // A port for externally applied generalized forces.
  systems::InputPortIndex applied_generalized_force_input_port_;

  // Port for externally applied spatial forces.
  systems::InputPortIndex applied_spatial_force_input_port_;

  systems::OutputPortIndex continuous_state_output_port_;
  // A vector containing state output ports for each model instance indexed by
  // ModelInstanceIndex. An invalid value indicates that the model instance has
  // no state.
  std::vector<systems::OutputPortIndex> instance_continuous_state_output_ports_;

  // Index for the output port of ContactResults.
  systems::OutputPortIndex contact_results_port_;

  // A vector containing the index for the generalized contact forces port for
  // each model instance. This vector is indexed by ModelInstanceIndex. An
  // invalid value indicates that the model instance has no generalized
  // velocities and thus no generalized forces.
  std::vector<systems::OutputPortIndex>
      instance_generalized_contact_forces_output_ports_;

  // If the plant is modeled as a discrete system with periodic updates,
  // time_step_ corresponds to the period of those updates. Otherwise, if the
  // plant is modeled as a continuous system, it is exactly zero.
  double time_step_{0};

  // The solver used when the plant is modeled as a discrete system.
  std::unique_ptr<ImplicitStribeckSolver<T>> implicit_stribeck_solver_;

  // All MultibodyPlant cache indexes are stored in cache_indexes_.
  CacheIndexes cache_indexes_;
};

/// @cond
// Undef macros defined at the top of the file. From the GSG:
// "Exporting macros from headers (i.e. defining them in a header without
// #undefing them before the end of the header) is extremely strongly
// discouraged."
// This will require us to re-define them in the .cc file.
#undef DRAKE_MBP_THROW_IF_FINALIZED
#undef DRAKE_MBP_THROW_IF_NOT_FINALIZED
/// @endcond

// Forward declare to permit exclusive friendship for construction.
template <typename T>
struct AddMultibodyPlantSceneGraphResult;

/// Adds a MultibodyPlant and a SceneGraph instance to a diagram builder,
/// connecting the geometry ports.
/// @param[in,out] builder
///   Builder to add to.
/// @param[in] plant (optional)
///   Constructed plant (e.g. for using a discrete plant). By default, a
///   continuous plant is used.
/// @param[in] scene_graph (optional)
///   Constructed scene graph. If none is provided, one will be created and
///   used.
/// @return Pair of the registered plant and scene graph.
/// @pre `builder` must be non-null.
///
/// Recommended usages:
///
/// Assign to a MultibodyPlant reference (ignoring the SceneGraph):
/// @code
///   MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder);
///   plant.DoFoo(...);
/// @endcode
/// This flavor is the simplest, when the SceneGraph is not explicitly needed.
/// (It can always be retrieved later via GetSubsystemByName("scene_graph").)
///
/// Assign to auto, and use the named public fields:
/// @code
///   auto items = AddMultibodyPlantSceneGraph(&builder);
///   items.plant.DoFoo(...);
///   items.scene_graph.DoBar(...);
/// @endcode
/// or
/// @code
///   auto items = AddMultibodyPlantSceneGraph(&builder);
///   MultibodyPlant<double>& plant = items.plant;
///   SceneGraph<double>& scene_graph = items.scene_graph;
///   ...
///   plant.DoFoo(...);
///   scene_graph.DoBar(...);
/// @endcode
/// This is the easiest way to use both the plant and scene_graph.
///
/// Assign to already-declared pointer variables:
/// @code
///   MultibodyPlant<double>* plant{};
///   SceneGraph<double>* scene_graph{};
///   std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder);
///   plant->DoFoo(...);
///   scene_graph->DoBar(...);
/// @endcode
/// This flavor is most useful when the pointers are class member fields
/// (and so perhaps cannot be references).
template <typename T>
AddMultibodyPlantSceneGraphResult<T>
AddMultibodyPlantSceneGraph(
    systems::DiagramBuilder<T>* builder,
    std::unique_ptr<MultibodyPlant<T>> plant = nullptr,
    std::unique_ptr<geometry::SceneGraph<T>> scene_graph = nullptr);

/// Temporary result from `AddMultibodyPlantSceneGraph`. This cannot be
/// constructed outside of this method.
/// @warning Do NOT use this as a function argument or member variable. The
/// lifetime of this object should be as short as possible.
template <typename T>
struct AddMultibodyPlantSceneGraphResult final {
  MultibodyPlant<T>& plant;
  geometry::SceneGraph<T>& scene_graph;

  /// For assignment to a plant reference (ignoring the scene graph).
  operator MultibodyPlant<T>&() { return plant; }

  /// For assignment to a std::tie of pointers.
  operator std::tuple<MultibodyPlant<T>*&, geometry::SceneGraph<T>*&>() {
    return std::tie(plant_ptr, scene_graph_ptr);
  }

#ifndef DRAKE_DOXYGEN_CXX
  // Only the move constructor is enabled; copy/assign/move-assign are deleted.
  AddMultibodyPlantSceneGraphResult(
      AddMultibodyPlantSceneGraphResult&&) = default;
  AddMultibodyPlantSceneGraphResult(
      const AddMultibodyPlantSceneGraphResult&) = delete;
  void operator=(const AddMultibodyPlantSceneGraphResult&) = delete;
  void operator=(AddMultibodyPlantSceneGraphResult&&) = delete;
#endif

 private:
  // Deter external usage by hiding construction.
  friend AddMultibodyPlantSceneGraphResult AddMultibodyPlantSceneGraph<T>(
    systems::DiagramBuilder<T>*, std::unique_ptr<MultibodyPlant<T>>,
    std::unique_ptr<geometry::SceneGraph<T>>);

  AddMultibodyPlantSceneGraphResult(
      MultibodyPlant<T>* plant_in, geometry::SceneGraph<T>* scene_graph_in)
      : plant(*plant_in), scene_graph(*scene_graph_in),
        plant_ptr(plant_in), scene_graph_ptr(scene_graph_in) {}

  // Pointers to enable implicit casts for `std::tie()` assignments using
  // `T*&`.
  MultibodyPlant<T>* plant_ptr{};
  geometry::SceneGraph<T>* scene_graph_ptr{};
};

#ifndef DRAKE_DOXYGEN_CXX
// Forward-declare specializations, prior to DRAKE_DECLARE... below.
// See the .cc file for an explanation why we specialize these methods.
template <>
typename MultibodyPlant<symbolic::Expression>::SceneGraphStub&
MultibodyPlant<symbolic::Expression>::member_scene_graph();
template <>
std::vector<geometry::PenetrationAsPointPair<double>>
MultibodyPlant<double>::CalcPointPairPenetrations(
    const systems::Context<double>&) const;
template <>
std::vector<geometry::PenetrationAsPointPair<AutoDiffXd>>
MultibodyPlant<AutoDiffXd>::CalcPointPairPenetrations(
    const systems::Context<AutoDiffXd>&) const;
#endif

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::MultibodyPlant)
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct drake::multibody::AddMultibodyPlantSceneGraphResult)
