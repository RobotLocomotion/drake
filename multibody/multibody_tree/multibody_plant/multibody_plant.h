#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_set.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/multibody_tree/implicit_stribeck/implicit_stribeck_solver.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

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
/// collection of interconnected bodies.
/// %MultibodyPlant provides a user-facing API to:
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
/// @section adding_elements Adding modeling elements
///
/// @cond
/// TODO(amcastro-tri): Update this section to add force elements and
/// constraints.
/// @endcond
///
/// Clients of a %MultibodyPlant can add multibody elements with the following
/// methods:
/// - Bodies: AddRigidBody().
/// - Joints: AddJoint().
///
/// All modeling elements **must** be added pre-finalize.
///
/// @section geometry_registration Registering geometry with a SceneGraph
///
/// %MultibodyPlant users can register geometry with a SceneGraph for
/// essentially two purposes; a) visualization and, b) contact modeling.
// TODO(SeanCurtis-TRI): update this comment as the number of SceneGraph
// roles changes.
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
/// 1. Call to RegisterAsSourceForSceneGraph().
/// 2. Calls to RegisterCollisionGeometry(), as many as needed.
/// 3. Call to Finalize(), user is done specifying the model.
/// 4. Connect SceneGraph::get_query_output_port() to
///    get_geometry_query_input_port().
/// Refer to the documentation provided in each of the methods above for further
/// details.
///
/// @section Finalize() stage
///
/// Once the user is done adding modeling elements and registering geometry, a
/// call to Finalize() must be performed. This call will:
/// - Build the underlying MultibodyTree topology, see MultibodyTree::Finalize()
///   for details,
/// - declare the plant's state,
/// - declare the plant's input and output ports,
/// - declare input and output ports for communication with a SceneGraph.
/// @cond
/// TODO(amcastro-tri): Consider making the actual geometry registration with GS
/// AFTER Finalize() so that we can tell if there are any bodies welded to the
/// world to which we could just assign anchored geometry instead of dynamic
/// geometry. This is an optimization and the API, and pre/post-finalize
/// conditions should not change.
/// @endcond
///
/// <h3> References </h3>
/// - [Featherstone 2008] Featherstone, R., 2008.
///     Rigid body dynamics algorithms. Springer.
/// - [Jain 2010] Jain, A., 2010.
///     Robot and multibody dynamics: analysis and algorithms.
///     Springer Science & Business Media.
/// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
///     Minimal formulation of joint motion for biomechanisms.
///     Nonlinear dynamics, 62(1), pp.291-303.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template<typename T>
class MultibodyPlant : public systems::LeafSystem<T> {
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
  template<typename U>
  MultibodyPlant(const MultibodyPlant<U>& other) :
      systems::LeafSystem<T>(systems::SystemTypeTag<
          drake::multibody::multibody_plant::MultibodyPlant>()) {
    DRAKE_THROW_UNLESS(other.is_finalized());
    model_ = other.model_->template CloneToScalar<T>();
    time_step_ = other.time_step_;
    // Copy of all members related with geometry registration.
    source_id_ = other.source_id_;
    body_index_to_frame_id_ = other.body_index_to_frame_id_;
    geometry_id_to_body_index_ = other.geometry_id_to_body_index_;
    geometry_id_to_visual_index_ = other.geometry_id_to_visual_index_;
    geometry_id_to_collision_index_ = other.geometry_id_to_collision_index_;
    visual_geometries_ = other.visual_geometries_;
    collision_geometries_ = other.collision_geometries_;
    // MultibodyTree::CloneToScalar() already called MultibodyTree::Finalize()
    // on the new MultibodyTree on U. Therefore we only Finalize the plant's
    // internals (and not the MultibodyTree).
    FinalizePlantOnly();
  }


  /// Returns the number of bodies in the model, including the "world" body,
  /// which is always part of the model.
  /// @see AddRigidBody().
  int num_bodies() const {
    return model_->num_bodies();
  }

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const {
    return model_->num_joints();
  }

  /// Returns the number of joint actuators in the model.
  /// @see AddJointActuator().
  int num_actuators() const {
    return model_->num_actuators();
  }

  /// Returns the size of the generalized position vector `q` for `this` model.
  int num_positions() const { return model_->num_positions(); }

  /// Returns the size of the generalized velocity vector `v` for `this` model.
  int num_velocities() const { return model_->num_velocities(); }

  /// Returns the size of the multibody system state vector `x = [q; v]` for
  /// `this` model. This will equal the number of generalized positions
  /// (see num_positions()) plus the number of generalized velocities
  /// (see num_velocities()).
  /// Notice however that the state of a %MultibodyPlant, stored in its Context,
  /// can actually contain other variables such as integrated power and discrete
  /// states.
  int num_multibody_states() const { return model_->num_states(); }

  /// Returns the total number of actuated degrees of freedom.
  /// That is, the vector of actuation values u has this size.
  /// See AddJointActuator().
  int num_actuated_dofs() const { return model_->num_actuated_dofs(); }

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

  /// Creates a rigid body model with the provided name and spatial inertia.
  /// This method returns a constant reference to the body just added, which
  /// will remain valid for the lifetime of `this` %MultibodyPlant.
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
  ///   A string that uniquely identifies the new body to be added to `this`
  ///   model. A std::runtime_error is thrown if a body named `name` already is
  ///   part of the model. See HasBodyNamed(), Body::name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this` model,
  ///   computed about the body frame origin `Bo` and expressed in the body
  ///   frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_B) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    const RigidBody<T>& body = model_->AddRigidBody(name, M_BBo_B);
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
  ///   identity pose, provide `Isometry3<double>::Identity()` as your input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame M with pose `X_BM` equal to the
  ///   identity pose, provide `Isometry3<double>::Identity()` as your input.
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
  /// @throws if `this` model already contains a joint with the given `name`.
  /// See HasJointNamed(), Joint::name().
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    return model_->template AddJoint<JointType>(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
  }

  /// Adds a new force element model of type `ForceElementType` to `this` model.
  /// The arguments to this method `args` are forwarded to `ForceElementType`'s
  /// constructor.
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
    return model_->template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
  }

  // SFINAE overload for ForceElementType = UniformGravityFieldElement.
  // This allow us to keep track of the gravity field parameters.
  template<template<typename Scalar> class ForceElementType, typename... Args>
  typename std::enable_if<std::is_same<
      ForceElementType<T>,
      UniformGravityFieldElement<T>>::value, const ForceElementType<T>&>::type
  AddForceElement(Args&&... args) {
    DRAKE_MBP_THROW_IF_FINALIZED();
    DRAKE_DEMAND(!gravity_field_.has_value());
    // We save the force element so that we can grant users access to it for
    // gravity field specific queries.
    gravity_field_ =
        &model_->template AddForceElement<UniformGravityFieldElement>(
            std::forward<Args>(args)...);
    return *gravity_field_.value();
  }

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
  /// @throws if `joint.num_dofs() > 1` since for now we only support actuators
  /// for single dof joints.
  const JointActuator<T>& AddJointActuator(
      const std::string& name, const Joint<T>& joint) {
    DRAKE_THROW_UNLESS(joint.num_dofs() == 1);
    return model_->AddJointActuator(name, joint);
  }
  /// @}

  /// @name Querying for multibody elements by name
  /// These methods allow a user to query whether a given multibody element is
  /// part of this plant's model. These queries can be performed at any time
  /// during the lifetime of a %MultibodyPlant model, i.e. there is no
  /// restriction on whether they must be called before or after Finalize().
  /// That is, these queries can be performed while new multibody elements are
  /// being added to the model.
  /// @{

  /// @returns `true` if a body named `name` was added to the model.
  /// @see AddRigidBody().
  bool HasBodyNamed(const std::string& name) const {
    return model_->HasBodyNamed(name);
  }

  /// @returns `true` if a joint named `name` was added to the model.
  /// @see AddJoint().
  bool HasJointNamed(const std::string& name) const {
    return model_->HasJointNamed(name);
  }

  /// @returns `true` if an actuator named `name` was added to the model.
  /// @see AddJointActuator().
  bool HasJointActuatorNamed(const std::string& name) const {
    return model_->HasJointActuatorNamed(name);
  }
  /// @}

  /// @name Retrieving multibody elements by name
  /// These methods allow a user to retrieve a reference to a multibody element
  /// by its name. An exception is thrown if there is no element with the
  /// requested name.
  /// These queries can be performed at any time during the lifetime of a
  /// %MultibodyPlant model, i.e. there is no restriction on whether they must
  /// be called before or after Finalize(). This implies that these queries can
  /// be performed while new multibody elements are being added to the model.
  /// @{

  /// Returns a constant reference to the rigid body that is uniquely identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no body with the requested name.
  /// @see HasBodyNamed() to query if there exists a body in `this` model with a
  /// given specified name.
  const Body<T>& GetBodyByName(const std::string& name) const {
    return model_->GetBodyByName(name);
  }

  /// Returns a constant reference to the joint that is uniquely identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no joint with the requested name.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  const Joint<T>& GetJointByName(const std::string& name) const {
    return model_->GetJointByName(name);
  }

  /// A templated version of GetJointByName() to return a constant reference of
  /// the specified type `JointType` in place of the base Joint class. See
  /// GetJointByName() for details.
  /// @tparam JointType The specific type of the Joint to be retrieved. It must
  /// be a subclass of Joint.
  /// @throws std::logic_error if the named joint is not of type `JointType` or
  /// if there is no Joint with that name.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  template <template<typename> class JointType>
  const JointType<T>& GetJointByName(const std::string& name) const {
    return model_->template GetJointByName<JointType>(name);
  }

  /// Returns a constant reference to the actuator that is uniquely identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no actuator with the requested name.
  /// @see HasJointActuatorNamed() to query if there exists an actuator in
  /// `this` model with a given specified name.
  const JointActuator<T>& GetJointActuatorByName(
      const std::string& name) const {
    return model_->GetJointActuatorByName(name);
  }
  /// @}

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
  /// @throws if called post-finalize.
  /// @throws if `scene_graph` is the nullptr.
  /// @throws if called more than once.
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
  /// @param[out] scene_graph
  ///   A valid non nullptr to a SceneGraph on which geometry will get
  ///   registered.
  /// @throws if `scene_graph` is the nullptr.
  /// @throws if called post-finalize.
  /// @throws if `scene_graph` does not correspond to the same instance with
  /// which RegisterAsSourceForSceneGraph() was called.
  // TODO(amcastro-tri): When GS supports it, provide argument to specify
  // visual properties.
  void RegisterVisualGeometry(const Body<T>& body,
                              const Isometry3<double>& X_BG,
                              const geometry::Shape& shape,
                              geometry::SceneGraph<T>* scene_graph);

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
  ///   A valid, non-null pointer to a SceneGraph on which geometry will get
  ///   registered.
  /// @throws std::exception if `scene_graph` is the nullptr.
  /// @throws std::exception if called post-finalize.
  /// @throws std::exception if `scene_graph` does not correspond to the
  /// same instance with which RegisterAsSourceForSceneGraph() was called.
  geometry::GeometryId RegisterCollisionGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      const geometry::Shape& shape,
      const CoulombFriction<double>& coulomb_friction,
      geometry::SceneGraph<T>* scene_graph);

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
  /// Note: There is a *very* specific order of operations.
  ///   1. Bodies and geometries must be added to the %MultibodyPlant.
  ///   2. The %MultibodyPlant must be finalized (via Finalize()).
  ///   3. Create GeometrySet instances from bodies (via this method).
  ///   4. Invoke SceneGraph::ExcludeCollisions*() to filter collisions.
  ///   5. Allocate context.
  /// Changing the order will cause exceptions to be thrown.
  ///
  /// @throws std::exception if called pre-finalize.
  geometry::GeometrySet CollectRegisteredGeometries(
      const std::vector<const RigidBody<T>*>& bodies) const;

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
  /// @throws std::exception if called pre-finalize. See Finalize().
  const systems::InputPortDescriptor<T>& get_geometry_query_input_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// SceneGraph.
  /// @throws std::exception if this system was not registered with a
  /// SceneGraph.
  /// @throws std::exception if called pre-finalize. See Finalize().
  const systems::OutputPort<T>& get_geometry_poses_output_port() const;
  /// @}

  /// Returns `true` if `this` %MultibodyPlant was registered with a
  /// SceneGraph.
  /// This method can be called at any time during the lifetime of `this` plant
  /// to query if `this` plant has been registered with a SceneGraph, either
  /// pre- or post-finalize, see Finalize().
  bool geometry_source_is_registered() const {
    return !!source_id_;
  }

  /// If the body with `body_index` has geometry registered with it, it returns
  /// the geometry::FrameId associated with it. Otherwise, it returns nullopt.
  /// @throws if called pre-finalize.
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
  /// @throws if no geometry has been registered with the body indicated by
  /// `body_index`.
  /// @throws if called pre-finalize.
  geometry::FrameId GetBodyFrameIdOrThrow(BodyIndex body_index) const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    const auto it = body_index_to_frame_id_.find(body_index);
    if (it == body_index_to_frame_id_.end()) {
      throw std::logic_error(
          "Body '" + model().get_body(body_index).name() +
          "' does not have geometry registered with it.");
    }
    return it->second;
  }

  /// Returns a constant reference to the input port for external actuation.
  /// This input port is a vector valued port, which can be set with
  /// JointActuator::set_actuation_vector().
  /// @pre Finalize() was already called on `this` plant.
  /// @throws if called before Finalize() or if the model does not contain any
  /// actuators. See AddJointActuator() and num_actuators().
  const systems::InputPortDescriptor<T>& get_actuation_input_port() const;

  /// Returns a constant reference to the output port for the full continuous
  /// state of the model.
  /// @throws std::exception if called pre-finalize.
  const systems::OutputPort<T>& get_continuous_state_output_port() const;

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& world_body() const {
    return model_->world_body();
  }

  /// Returns a constant reference to the underlying MultibodyTree model for
  /// `this` plant.
  /// @throws if called pre-finalize. See Finalize().
  const MultibodyTree<T>& model() const {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    return *model_;
  }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize().
  /// @see Finalize().
  bool is_finalized() const { return model_->topology_is_valid(); }

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
  ///          1. the %MultibodyPlant has already been finalized,
  ///          2. `scene_graph` isn't provided when required, or
  ///          3. a different scene_graph instance is provided than the one
  ///             for which this plant is a geometry source.
  void Finalize(geometry::SceneGraph<T>* scene_graph = nullptr);

  /// Returns `true` if this plant is modeled as a discrete system.
  /// This property of the plant is specified at construction and therefore this
  /// query can be performed either pre- or post- finalize, see Finalize().
  bool is_discrete() const { return time_step_ > 0.0; }

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
  }
  /// @}

  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero.
  /// @throws if called pre-finalize. See Finalize().
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_MBP_THROW_IF_NOT_FINALIZED();
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

  // Friend class to facilitate testing.
  friend class MultibodyPlantTester;

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

  // Helper method to apply collision filters based on body-adjacency. By
  // default, we don't consider collisions between geometries affixed to
  // bodies connected by a joint.
  void FilterAdjacentBodies(geometry::SceneGraph<T>* scene_graph);

  // This is a *temporary* method to eliminate visual geometries from collision
  // while we wait for geometry roles to be introduced.
  // TODO(SeanCurtis-TRI): Remove this when geometry roles are introduced.
  void ExcludeCollisionsWithVisualGeometry(
      geometry::SceneGraph<T>* scene_graph);

  // No inputs implies no feedthrough; this makes it explicit.
  // TODO(amcastro-tri): add input ports for actuators.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  // Helper method to declare state and ports after Finalize().
  void DeclareStateAndPorts();

  // This override gives System::AllocateContext() the chance to create a more
  // specialized context type, in this case, a MultibodyTreeContext.
  std::unique_ptr<systems::LeafContext<T>> DoMakeLeafContext() const override;

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

  void DoMapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      systems::VectorBase<T>* generalized_velocity) const override;

  void DoMapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& generalized_velocity,
      systems::VectorBase<T>* qdot) const override;

  // Helper method to declare cache entries to be allocated in the context.
  void DeclareCacheEntries();

  // Helper method to Eval() position kinematics cached in the context.
  const PositionKinematicsCache<T>& EvalPositionKinematics(
      const systems::Context<T>& context) const;

  // Helper method to Eval() velocity kinematics cached in the context.
  const VelocityKinematicsCache<T>& EvalVelocityKinematics(
      const systems::Context<T>& context) const;

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
  // 4. The body is *not* the world body.
  geometry::GeometryId RegisterGeometry(const Body<T>& body,
                                        const Isometry3<double>& X_BG,
                                        const geometry::Shape& shape,
                                        geometry::SceneGraph<T>* scene_graph);

  // Helper method to register anchored geometry to the world, either visual or
  // collision. This associates a GeometryId with the world body.
  // This assumes:
  // 1. Finalize() was not called on `this` plant.
  // 2. RegisterAsSourceForSceneGraph() was called on `this` plant.
  // 3. `scene_graph` points to the same SceneGraph instance previously
  //    passed to RegisterAsSourceForSceneGraph().
  geometry::GeometryId RegisterAnchoredGeometry(
      const Isometry3<double>& X_WG, const geometry::Shape& shape,
      geometry::SceneGraph<T>* scene_graph);

  bool body_has_registered_frame(const Body<T>& body) const {
    return body_index_to_frame_id_.find(body.index()) !=
        body_index_to_frame_id_.end();
  }

  // Calc method for the continuous state vector output port.
  void CopyContinuousStateOut(
      const systems::Context<T>& context, systems::BasicVector<T>* state) const;

  // Helper method to declare output ports used by this plant to communicate
  // with a SceneGraph.
  void DeclareSceneGraphPorts();

  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  // Helper to evaluate if a GeometryId corresponds to a collision model.
  bool is_collision_geometry(geometry::GeometryId id) const {
    return geometry_id_to_collision_index_.count(id) > 0;
  }

  std::vector<geometry::PenetrationAsPointPair<T>>
  CalcPointPairPenetrations(const systems::Context<T>& context) const;

  std::vector<CoulombFriction<T>> CalcCombinedFrictionCoefficients(
      const std::vector<geometry::PenetrationAsPointPair<T>>&
      point_pairs) const;

  // Helper method to compute contact forces in the normal direction using a
  // penalty method.
  void CalcAndAddContactForcesByPenaltyMethod(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const VelocityKinematicsCache<T>& vc,
      std::vector<SpatialForce<T>>* F_BBo_W_array, bool include_friction,
      const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs,
      VectorX<T>* fn) const;

  // Given a set of point pairs in `point_pairs_set`, this method computes the
  // Jacobian N(q) such that:
  //   vn = N(q) v
  // where the i-th component of vn corresponds to the "separation velocity"
  // for the i-th point pair in the set. The i-th separation velocity is defined
  // positive for when the depth in the i-th point pair (
  // PenetrationAsPointPair::depth) is decreasing. Since for contact problems
  // the (positive) depth in PenetrationAsPointPair is defined so that it
  // corresponds to interpenetrating body geometries, a positive separation
  // velocity corresponds to bodies moving apart.
  MatrixX<T> CalcNormalSeparationVelocitiesJacobian(
      const systems::Context<T>& context,
      const std::vector<geometry::PenetrationAsPointPair<T>>&
      point_pairs_set) const;

  // Given a set of nc point pairs in `point_pairs_set`, this method computes
  // the tangential velocities Jacobian D(q) such that:
  //   vt = D(q) v
  // where v ∈ ℝⁿᵛ is the vector of generalized velocities, D(q) is a matrix of
  // size 2⋅nc×nv and vt is a vector of size 2⋅nc.
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
  MatrixX<T> CalcTangentVelocitiesJacobian(
      const systems::Context<T>& context,
      const std::vector<geometry::PenetrationAsPointPair<T>>& point_pairs_set,
      std::vector<Matrix3<T>>* R_WC_set = nullptr) const;

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

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
        const CoulombFriction<T>& friction) const;

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

    /// Returns the value of the stiction tolerance for `this` model.
    /// It returns a negative value when the stiction tolerance has not been set
    /// previously with set_stiction_tolerance().
    double stiction_tolerance() const { return v_stiction_tolerance_; }

    // Dimensionless modified Stribeck function defined as:
    // ms(x) = ⌈ mu * x * (2.0 - x),  x  < 1
    //         ⌊ mu                ,  x >= 1
    // where x corresponds to the dimensionless tangential speed
    // x = v / v_stribeck.
    static T ModifiedStribeck(const T& x, const T& mu);

    // Derivative of the dimensionless modified Stribeck function:
    // ms(x) = ⌈ mu * (2 * (1 - x)),  x  < 1
    //         ⌊ 0                 ,  x >= 1
    // where x corresponds to the dimensionless tangential speed
    // x = v / v_stribeck.
    static T ModifiedStribeckPrime(const T& speed_BcAc, const T& mu);

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

  // Iteration order on this map DOES matter, and therefore we use an std::map.
  std::map<BodyIndex, geometry::FrameId> body_index_to_frame_id_;

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
  int geometry_query_port_{-1};
  int geometry_pose_port_{-1};

  // For geometry registration with a GS, we save a pointer to the GS instance
  // on which this plants calls RegisterAsSourceForSceneGraph(). This is
  // ONLY (and it MUST ONLY be used) used to verify that successive registration
  // calls are performed on the same instance of GS.
  const geometry::SceneGraph<T>* scene_graph_{nullptr};

  // Input/Output port indexes:
  int actuation_port_{-1};
  int continuous_state_output_port_{-1};

  // If the plant is modeled as a discrete system with periodic updates,
  // time_step_ corresponds to the period of those updates. Otherwise, if the
  // plant is modeled as a continuous system, it is exactly zero.
  double time_step_{0};

  // The default solver when the plant is modeled as a discrete system.
  std::unique_ptr<implicit_stribeck::ImplicitStribeckSolver<T>> implicit_stribeck_solver_;

  // Temporary solution for fake cache entries to help stabilize the API.
  // TODO(amcastro-tri): Remove these when caching lands.
  std::unique_ptr<PositionKinematicsCache<T>> pc_;
  std::unique_ptr<VelocityKinematicsCache<T>> vc_;
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

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::multibody::multibody_plant::MultibodyPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
