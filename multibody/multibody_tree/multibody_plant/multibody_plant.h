#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/// %MultibodyPlant is a Drake system framework representation (see
/// systems::System) for the model of a physical system consisting of a
/// collection of interconnected bodies.
/// %MultibodyPlant provides a user-facing API to:
/// - add bodies, joints, force elements, and constraints,
/// - register geometries to a provided GeometrySystem instance,
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
/// @section mbp_geometry_registration Registering with a GeometrySystem
///
/// %MultibodyPlant users can register geometry with a GeometrySystem for
/// essentially two purposes; a) visualization and, b) contact modeling.
/// To add geometry for visualization %MultibodyPlant provides:
/// - RegisterVisualGeometry() for dynamically moving geometry.
/// - RegisterVisualAnchoredGeometry() for geometry that does not move but that
///   is fixed to the world.
///
/// The process of geometry registration includes:
/// 1. If not done yet, the plant is registered as a source for the given
///    GeometrySystem. This assigns a SourceId to the plant.
/// 2. For the body on which geometry is being registered, a new frame is
///    registered with GeometrySystem is not done already. Therefore, each body
///    gets associated with a FrameId.
/// 3. Finally, geometry is registered for the corresponding FrameId. This
///    associates a GeometryId with the body FrameId.
/// This entire process is controlled by the %MultibodyPlant's registration
/// methods.
///
/// @section Finalize() stage
///
/// Once the user is done adding modeling elements and registering geometry, a
/// a call to Finalize() must be performed. This call will:
/// - Build the underlying MultibodyTree topology, see MultibodyTree::Finalize()
///   for details,
/// - declare the plant's state,
/// - declare the plant's input and output ports,
/// - declare input and output ports for communication with a GeometrySystem.
///
/// @cond
/// TODO(amcastro-tri): Add next section in future PR's as funcionality lands.
/// @section computational_queries Performing computational queries
/// Once a %MultibodyPlant model of a multibody system is created, a number of
/// computational queries can be performed on a given Context:
/// - CalcMassMatrix(): Computes the mass matrix of the system in `O(n²)`.
/// - CalcInverseDynamics(): `O(n)` Newton-Euler recursive algorithm.
/// - CalcForwardDynamics(): `O(n)` Articulated Body Inertia algorithm.
/// - CalcPointsGeometricJacobianExpressedInWorld(): Jacobian matrix linearly
///   relating a set of points' translational velocities to the system's
///   generalized velocities.
/// - Others...
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
class MultibodyPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)

  /// Default constructor creates a plant with a single "world" body.
  /// Therefore, right after creation, num_bodies() returns one.
  MultibodyPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template<typename U>
  explicit MultibodyPlant(const MultibodyPlant<U>& other);

  /// Returns the number of bodies in the model, including the "world" body,
  /// which is always part of the model.
  /// @see AddRigidBody().
  int num_bodies() const {
    return model_->get_num_bodies();
  }

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const {
    return model_->get_num_joints();
  }

  /// Returns the size of the generalized position vector `q` for `this` model.
  int num_positions() const { return model_->get_num_positions(); }

  /// Returns the size of the generalized velocity vector `v` for `this` model.
  int num_velocities() const { return model_->get_num_velocities(); }

  /// Returns the size of the multibody system state vector `x = [q; v]` for
  /// `this` model. This will equal the number of generalized positions
  /// (see num_positions()) plus the number of generalized velocities
  /// (see num_velocities()).
  /// Notice however that the state of a %MultibodyPlant, stored in its Context,
  /// can actually contain other variables such as integrated power and discrete
  /// states.
  int num_multibody_states() const { return model_->get_num_states(); }

  /// @name Adding new multibody elements
  /// %MultibodyPlant users will add modeling elements like bodies,
  /// joints, force elements, constraints, etc, using one of these methods.
  /// Once a user is done adding __all__ modeling elements, the Finalize()
  /// method **must** be called before invoking any %MultibodyPlant service to
  /// perform computations.
  /// An attempt to call any of these methods **after** a call to Finalize() on
  /// the plant, will result on a std::runtime_error being thrown.
  /// See Finalize() for details.
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
  ///   part of the model. See HasBodyNamed(), Body::get_name().
  /// @param[in] M_BBo_B
  ///   The SpatialInertia of the new rigid body to be added to `this` model,
  ///   computed about the body frame origin `Bo` and expressed in the body
  ///   frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_B) {
    return model_->AddRigidBody(name, M_BBo_B);
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
  ///   part of the model. See HasJointNamed(), Joint::get_name().
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
  /// See HasJointNamed(), Joint::get_name().
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
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
  /// @returns A constant reference to the new ForceElement just added, of type
  ///   `ForceElementType<T>` specialized on the scalar type T of `this`
  ///   %MultibodyPlant. It will remain valid for the lifetime of `this`
  ///   %MultibodyPlant.
  /// @see The ForceElement class's documentation for further details on how a
  /// force element is defined.
  template<template<typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args) {
    return model_->template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
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
  /// @}

  /// @name Retrieving multibody elements by name
  /// These methods allow a user to retrieve a reference to a multibody element
  /// by its name. A std::logic_error is thrown if there is no element with the
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
  /// @}

  /// @name Registering geometry for visualization
  /// These methods allow to register geometry with a GeometrySystem in order to
  /// visualize a plant's model. At the time of the first registration, `this`
  /// `MultibodyPlant` is registered with the provided GeometrySystem and gets
  /// assigned a SourceId that can be retrieved with get_source_id().
  /// At Finalize(), two output ports are declared to enable communication with
  /// GeometrySystem:
  ///   1. A port with the GeometryId for each geometry that was registered. It
  ///      can be retrieved with get_geometry_ids_output_port().
  ///   2. A port with the pose for each of the geometries that was registered.
  ///      It can be retrieved with get_geometry_poses_output_port().
  // TODO(amcastro-tri): When GS supports it, provide argument to specify
  // visual properties.
  /// @{

  /// This method allows to register (dynamic, i.e. moving) geometry with a
  /// given geometry::Shape to be used for visualization of a given `body`.
  /// @param[in] body
  ///   The body for which geometry is being registered.
  /// @param[in] X_BG
  ///   The fixed pose of the geometry frame G in the body frame B.
  /// @param[in] shape
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder.
  /// @param[out] geometry_system
  ///   A valid non nullptr to a GeometrySystem on which geometry will get
  ///   registered.
  /// @throws if `geometry_system` is the nullptr.
  void RegisterVisualGeometry(
      const Body<T>& body,
      const Isometry3<double>& X_BG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  /// This method allows to register anchored geometry to the world with a given
  /// geometry::Shape. This geometry does not move and has a fixed pose `X_WG`
  /// in the world frame W.
  /// @param[in] X_WG
  ///   The fixed pose of the geometry frame G in the world frame W.
  /// @param[in] shape
  ///   The geometry::Shape used for visualization. E.g.: geometry::Sphere,
  ///   geometry::Cylinder.
  /// @param[out] geometry_system
  ///   A valid non nullptr to a GeometrySystem on which geometry will get
  ///   registered.
  /// @throws if `geometry_system` is the nullptr.
  void RegisterAnchoredVisualGeometry(
      const Isometry3<double>& X_WG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  /// Returns the number of geometries registered for visualization.
  int get_num_visual_geometries() const {
    return static_cast<int>(geometry_id_to_visual_index_.size());
  }
  /// @}

  /// @name Retrieving ports for communication with a GeometrySystem.
  /// @{

  /// Returns the unique id identifying `this` plant as a source for a
  /// GeometrySystem.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  optional<geometry::SourceId> get_source_id() const {
    return source_id_;
  }

  /// Returns the output port of frame id's used to communicate poses to a
  /// GeometrySystem.
  /// @throws if this system was not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_ids_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// GeometrySystem.
  /// @throws if this system was not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_poses_output_port() const;
  /// @}

  /// @returns `true` if `this` %MultibodyPlant was registered with a
  /// GeometrySystem.
  bool geometry_source_is_registered() const {
    return !!source_id_;
  }

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& get_world_body() const {
    return model_->get_world_body();
  }

  const MultibodyTree<T>& model() const { return *model_; }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize().
  /// @see Finalize().
  bool is_finalized() const { return model_->topology_is_valid(); }

  /// This method must be called after all elements in the model (joints, bodies,
  /// force elements, constraints, etc.) are added and before any computations
  /// are performed.
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
  /// If `this` plant registered geometry with a GeometrySystem, input and
  /// output ports to enable communication with that GeometrySystem are declared
  /// as well.
  ///
  /// @see is_finalized().
  ///
  /// @throws std::logic_error if the %MultibodyPlant has already been
  /// finalized.
  void Finalize();

  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

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
  // 1. If not done yet, register this plant as a source for the given
  //    GeometrySystem. This assigns a SourceId to this plant.
  // 2. Register a frame for this body if not already done so. The body gets
  //    associated with a FrameId.
  // 3. Register geometry for the corresponding FrameId. This associates a
  //    GeometryId with the body FrameId.
  geometry::GeometryId RegisterGeometry(
      const Body<T>& body,
      const Isometry3<double>& X_BG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  // Helper method to register anchored geometry to the world, either visual or
  // collision. The anchored geometry registration includes:
  // 1. If not done yet, register this plant as a source for the given
  //    GeometrySystem. This assigns a SourceId to this plant.
  // 2. Register geometry for the world body. This associates a GeometryId with
  //    the world body.
  geometry::GeometryId RegisterAnchoredGeometry(
      const Isometry3<double>& X_WG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  bool body_has_registered_frame(const Body<T>& body) const {
    return body_index_to_frame_id_.find(body.get_index()) !=
        body_index_to_frame_id_.end();
  }

  // Helper method to declare output ports used by this plant to communicate
  // with a GeometrySystem.
  void DeclareGeometrySystemPorts();

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(
      const systems::Context<T>& context) const;

  // Calculate the id output.
  void CalcFrameIdOutput(
      const systems::Context<T>& context,
      geometry::FrameIdVector* id_set) const;

  // Allocate the frame pose set output port value.
  geometry::FramePoseVector<T> AllocateFramePoseOutput(
      const systems::Context<T>& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  optional<geometry::SourceId> source_id_{nullopt};

  // Frame Id's for each body in the model:
  // Not all bodies need to be in this map.
  std::unordered_map<BodyIndex, geometry::FrameId> body_index_to_frame_id_;

  std::unordered_map<geometry::GeometryId, BodyIndex> geometry_id_to_body_index_;

  // Map provided at construction that tells how bodies (referenced by name),
  // map to frame ids.
  std::unordered_map<std::string, geometry::FrameId> body_name_to_frame_id_;

  std::unordered_map<geometry::GeometryId, int> geometry_id_to_visual_index_;

  std::unordered_map<geometry::GeometryId, int> geometry_id_to_collision_index_;

  // Port handles for geometry:
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};

  // Temporary solution for fake cache entries to help statbilize the API.
  // TODO(amcastro-tri): Remove these when caching lands.
  std::unique_ptr<PositionKinematicsCache<T>> pc_;
  std::unique_ptr<VelocityKinematicsCache<T>> vc_;
};

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
