#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
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
/// @cond
/// TODO(amcastro-tri): In subsequent PR add doc on how to register geometry.
/// @endcond
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
    DRAKE_THROW_UNLESS(!HasBodyNamed(name));
    const RigidBody<T>& body =
        model_->template AddBody<RigidBody>(name, M_BBo_B);
    body_name_to_index_[name] = body.get_index();
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
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");
    DRAKE_THROW_UNLESS(!HasJointNamed(name));
    const JointType<T>& joint = model_->template AddJoint<JointType>(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
    joint_name_to_index_[name] = joint.get_index();
    return joint;
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
  bool HasBodyNamed(const std::string& name) {
    return body_name_to_index_.find(name) != body_name_to_index_.end();
  }

  /// @returns `true` if a joint named `name` was added to the model.
  /// @see AddJoint().
  bool HasJointNamed(const std::string& name) {
    return joint_name_to_index_.find(name) != joint_name_to_index_.end();
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
    auto it = body_name_to_index_.find(name);
    if (it == body_name_to_index_.end()) {
      throw std::logic_error("There is no body named '" + name +
          "' in the model.");
    }
    return model_->get_body(it->second);
  }

  /// Returns a constant reference to the joint that is uniquely identified
  /// by the string `name` in `this` model.
  /// @throws std::logic_error if there is no joint with the requested name.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  const Joint<T>& GetJointByName(const std::string& name) const {
    auto it = joint_name_to_index_.find(name);
    if (it == joint_name_to_index_.end()) {
      throw std::logic_error("There is no joint named '" + name +
          "' in the model.");
    }
    return model_->get_joint(it->second);
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
    static_assert(std::is_base_of<Joint<T>, JointType<T>>::value,
                  "JointType<T> must be a sub-class of Joint<T>.");
    const JointType<T>* joint =
        dynamic_cast<const JointType<T>*>(&GetJointByName(name));
    if (joint == nullptr) {
      throw std::logic_error("Joint '" + name + "' is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "' but of type '" +
          NiceTypeName::Get(GetJointByName(name)) + "'.");
    }
    return *joint;
  }
  /// @}

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& get_world_body() const {
    return model_->get_world_body();
  }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize().
  /// @see Finalize().
  bool is_finalized() const { return model_->topology_is_valid(); }

  /// This method must be called after all elements in the tree (joints, bodies,
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
  /// @throws std::logic_error if the %MultibodyPlant has already been
  /// finalized.
  void Finalize();

  /// @name Kinematic computations
  /// @{

  /// Computes the world pose `X_WB(q)` of each body B in the model as a
  /// function of the generalized positions q stored in `context`.
  /// @param[in] context
  ///   The context containing the state of the model. It stores the generalized
  ///   positions q of the model.
  /// @param[out] X_WB
  ///   On output this vector will contain the pose of each body in the model
  ///   ordered by BodyIndex. The index of a body in the model can be obtained
  ///   with Body::get_index(). This method throws an exception if `X_WB` is
  ///   `nullptr`. Vector `X_WB` is resized when needed to have size
  ///   num_bodies().
  void CalcAllBodyPosesInWorld(
      const systems::Context<T>& context,
      std::vector<Isometry3<T>>* X_WB) const;

  /// Computes the spatial velocity `V_WB(q, v)` of each body B in the model,
  /// measured and expressed in the world frame W. The body spatial velocities
  /// are a function of the generalized positions q and generalized velocities
  /// v, both stored in `context`.
  /// @param[in] context
  ///   The context containing the state of the model. It stores the generalized
  ///   positions q and velocities v of the model.
  /// @param[out] V_WB
  ///   On output this vector will contain the spatial velocity of each body in
  ///   the model ordered by BodyIndex. The index of a body in the model can be
  ///   obtained with Body::get_index(). This method throws an exception if
  ///   `V_WB` is `nullptr`. Vector `V_WB` is resized when needed to have size
  ///   num_bodies().
  void CalcAllBodySpatialVelocitiesInWorld(
      const systems::Context<T>& context,
      std::vector<SpatialVelocity<T>>* V_WB) const;

  /// Given the positions `p_BQi` for a set of points `Qi` measured and
  /// expressed in a frame B, this method computes the positions `p_AQi(q)` of
  /// each point `Qi` in the set as measured and expressed in another frame A,
  /// as a function of the generalized positions q of the model.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the generalized
  ///   positions q of the model.
  /// @param[in] from_frame_B
  ///   The frame B in which the positions `p_BQi` of a set of points `Qi` are
  ///   given.
  /// @param[in] p_BQi
  ///   The input positions of each point `Qi` in frame B. `p_BQi ∈ ℝ³ˣⁿᵖ` with
  ///   `np` the number of points in the set. Each column of `p_BQi` corresponds
  ///   to a vector in ℝ³ holding the position of one of the points in the set
  ///   as measured and expressed in frame B.
  /// @param[in] to_frame_A
  ///   The frame A in which it is desired to compute the positions `p_AQi` of
  ///   each point `Qi` in the set.
  /// @param[out] p_AQi
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame A. The output `p_AQi` **must** have the same size as
  ///   the input `p_BQi` or otherwise this method throws an exception.
  ///   That is `p_AQi` **must** be in `ℝ³ˣⁿᵖ`.
  ///
  /// @throws if either `p_BQi` or `p_AQi` does not have three rows (since these
  /// are meant to be row vectors of 3D vectors).
  /// @throws if `p_BQi` and `p_AQi` differ in the number of columns (the number
  /// of points in the input set).
  void CalcPointsPositions(
      const systems::Context<T>& context,
      const Frame<T>& from_frame_B,
      const Eigen::Ref<const MatrixX<T>>& p_BQi,
      const Frame<T>& to_frame_A,
      EigenPtr<MatrixX<T>> p_AQi) const;
  /// @}

  /// @name Methods to compute multibody Jacobians
  /// @{

  /// Given a set of points `Qi` with fixed position vectors `p_BQi` in a frame
  /// B, (that is, their time derivative `ᴮd/dt(p_BQi)` in frame B is zero),
  /// this method computes the geometric Jacobian `Jg_WQi` defined by:
  /// <pre>
  ///   v_WQi(q, v) = Jg_WQi(q)⋅v
  /// </pre>
  /// where `p_WQi` is the position vector in the world frame for each point
  /// `Qi` in the input set, `v_WQi(q, v)` is the translational velocity of
  /// point `Qi` in the world frame W and q and v are the vectors of generalized
  /// position and velocity, respectively. Since the spatial velocity of each
  /// point `Qi` is linear in the generalized velocities, the geometric
  /// Jacobian `Jg_WQi` is a function of the generalized coordinates q only.
  ///
  /// @param[in] context
  ///   The context containing the state of the model. It stores the
  ///   generalized positions q.
  /// @param[in] frame_B
  ///   The positions `p_BQi` of each point in the input set are measured and
  ///   expressed in this frame B and are constant (fixed) in this frame.
  /// @param[in] p_BQi_set
  ///   A matrix with the fixed position of a set of points `Qi` measured and
  ///   expressed in `frame_B`.
  ///   Each column of this matrix contains the position vector `p_BQi` for a
  ///   point `Qi` measured and expressed in frame B. Therefore this input
  ///   matrix lives in ℝ³ˣⁿᵖ with `np` the number of points in the set.
  /// @param[out] p_WQi_set
  ///   The output positions of each point `Qi` now computed as measured and
  ///   expressed in frame W. These positions are computed in the process of
  ///   computing the geometric Jacobian `J_WQi` and therefore external storage
  ///   must be provided.
  ///   The output `p_WQi_set` **must** have the same size
  ///   as the input set `p_BQi_set` or otherwise this method throws a
  ///   std::runtime_error exception. That is `p_WQi_set` **must** be in
  ///   `ℝ³ˣⁿᵖ`.
  /// @param[out] Jg_WQi
  ///   The geometric Jacobian `Jg_WQi(q)`, function of the generalized
  ///   positions q only. This Jacobian relates the translational velocity
  ///   `v_WQi` of each point `Qi` in the input set by: <pre>
  ///     `v_WQi(q, v) = Jg_WQi(q)⋅v`
  ///   </pre>
  ///   so that `v_WQi` is a column vector of size `3⋅np` concatenating the
  ///   velocity of all points `Qi` in the same order they were given in the
  ///   input set. Therefore `Jg_WQi` is a matrix of size `3⋅np x nv`, with `nv`
  ///   the number of generalized velocities. Only if needed, the Jacobian
  ///   matrix `J_WQi` will be resized to `3⋅np x nv`. An exception is thrown
  ///   if `Jg_WQi` is not a valid pointer.
  ///
  /// @throws an exception if the output `p_WQi_set` is nullptr or does not have
  /// the same size as the input array `p_BQi_set`.
  /// @throws an exception if `Jg_WQi` is nullptr or if it does not have the
  /// appropriate size, see documentation for `Jg_WQi` for details.
  void CalcPointsGeometricJacobianExpressedInWorld(
      const systems::Context<T>& context,
      const Frame<T>& frame_B, const Eigen::Ref<const MatrixX<T>>& p_BQi_set,
      EigenPtr<MatrixX<T>> p_WQi_set, EigenPtr<MatrixX<T>> Jg_WQi) const;
  /// @}
  // End of multibody Jacobian methods section.


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
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  // Implements the system dynamics according to this class's documentation.
  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

  // Map used to find body indexes by their unique body name.
  std::unordered_map<std::string, BodyIndex> body_name_to_index_;

  // Map used to find joint indexes by their joint name.
  std::unordered_map<std::string, JointIndex> joint_name_to_index_;
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
