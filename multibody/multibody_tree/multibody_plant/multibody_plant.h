#pragma once

#include <memory>
#include <string>
#include <unordered_map>

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

/// %MultibodyPlant offers a Drake system framework representation (see
/// systems::System) for a physical system consisting of a collection of
/// interconnected bodies.
/// %MultibodyPlant provides the user facing API's for adding bodies, joints,
/// force elements and constraints. It also provides the interface to register
/// geometry with a given GeometrySystem.
/// As a Drake System, it also provides the interfaces create and
/// manipulate its Context as well as to perform Context dependent computational
/// queries.
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
/// The state of a %MultibodyPlant `x = [q; v]` is given by its generalized
/// positions vector `q ∈ ℝⁿᵖ` and by its generalized velocities vector
/// `v ∈ ℝⁿᵛ` where `np` is the number of generalized positions (see
/// num_positions()) and `nv` is the number of generalized velocities (see
/// num_velocities()). As a Drake System, %MultibodyPlant implements the
/// governing equations for the multibody dynamical system in the form
/// `ẋ = f(t, x, u)` with t being the time and u the input vector of actuation
/// forces. For mechanical systems as modeled by %MultibodyPlant these equations
/// are given by [Featherstone 2008, Jain 2010]: <pre>
///   q̇ = N(q)v
///   M(q)v̇ + C(q, v)v = tau                                                (1)
/// </pre>
/// where `M(q)` is the mass matrix of the multibody system, `C(q, v)v`
/// corresponds to the bias term containing Coriolis and gyroscopic effects and
/// `N(q)` is the kinematic coupling matrix describing the relationship between
/// the rate of change of the generalized coordinates and the generalized
/// velocities, [Seth 2010]. N(q) is an `np x nv` matrix.
/// The vector `tau ∈ ℝⁿᵛ` on the right hand side of Eq. (1) corresponds to
/// generalized forces applied on the system. These can include externally
/// applied body forces, constraint forces and contact forces.
///
/// @section adding_elements Adding modeling elements
///
/// @cond
/// TODO(amcastro-tri): Update this section to add force elements and
/// constraints.
/// @endcond
///
/// Clients of a %MultibodyPlant can add multibody modeling elements with the
/// provided user facing API's:
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
///   relating a set of points translational velocity to the system's
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

  /// Returns the number of bodies in the model, including the "world" body.
  /// @see AddRigidBody().
  int num_bodies() const {
    return model_->get_num_bodies();
  }

  /// Returns the number of joints in the model.
  /// @see AddJoint().
  int num_joints() const {
    return model_->get_num_joints();
  }

  /// Returns the size of the generalized positions vector `q` for this plant's
  /// model.
  int num_positions() const { return model_->get_num_positions(); }

  /// Returns the size of the generalized velocities vector `v` for this plant's
  /// model.
  int num_velocities() const { return model_->get_num_velocities(); }

  /// Returns the size of the state vector `x` for this plant's model.
  int num_states() const { return model_->get_num_states(); }

  /// @name Methods to add new multibody elements.
  /// %MultibodyPlant users will add modeling elements like bodies,
  /// joints, force elements, constraints, etc, using one of these methods.
  /// Once a user is done adding modeling elements, the Finalize() method
  /// **must** be called before invoking any %MultibodyPlant service.
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
  /// @throws std::logic_error if Finalize() was already called on `this` plant.
  ///
  /// @param[in] name
  ///   A string that uniquely identifies the new body to be added to `this`
  ///   model. A std::runtime_error is thrown if a body named `name` already is
  ///   part of the model. See HasBodyNamed(), Body::get_name().
  /// @param[in] M_BBo_E
  ///   The SpatialInertia of the new rigid body to be added to `this` model,
  ///   computed about the body frame origin `Bo` and expressed in the body
  ///   frame B.
  /// @returns A constant reference to the new RigidBody just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_E) {
    // An exception is thrown if a body named `name` was already exists.
    DRAKE_THROW_UNLESS(!HasBodyNamed(name));
    const RigidBody<T>& body =
        model_->template AddBody<RigidBody>(name, M_BBo_E);
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
  /// The arguments to this method `args` are forwarded to `JointType`'s
  /// constructor. The newly created `JointType` object will be specialized on
  /// the scalar type T of this %MultibodyPlant.
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
  ///   your intention is to make a frame F with pose `X_PF`, provide
  ///   `Isometry3<double>::Identity()` as your input.
  /// @param[in] child
  ///   The child body connected by the new joint.
  /// @param[in] X_BM
  ///   The fixed pose of frame M attached to the child body, measured in
  ///   the frame B of that body. `X_BM` is an optional parameter; empty curly
  ///   braces `{}` imply that frame M **is** the same body frame B. If instead
  ///   your intention is to make a frame F with pose `X_PF`, provide
  ///   `Isometry3<double>::Identity()` as your input.
  /// @returns A constant reference to the new Joint just added, which will
  ///          remain valid for the lifetime of `this` %MultibodyPlant.
  ///
  /// Example of usage:
  /// @code
  ///   MultibodyPlant<T> plant;
  ///   // ... Code to define a parent body P and a child body B.
  ///   const RigidBody<double>& parent_body =
  ///     plant.AddRigidBody("ParentBodyName", SpatialInertia<double>(...));
  ///   const RigidBody<double>& child_body =
  ///     plant.AddRigidBody("ChildBodyName", SpatialInertia<double>(...));
  ///   // Define the pose X_BM of a frame M rigidly atached to child body B.
  ///   const RevoluteJoint<double>& elbow =
  ///     plant.AddJoint<RevoluteJoint>(
  ///       "Elbow",                /* joint name */
  ///       model.get_world_body(), /* parent body */
  ///       {},                     /* frame F IS the parent body frame P */
  ///       pendulum,               /* child body, the pendulum */
  ///       X_BM,                   /* pose of frame M in the body frame B */
  ///       Vector3d::UnitZ());     /* revolute axis in this case */
  /// @endcode
  ///
  /// @see The Joint class's documentation for further details on how a Joint
  /// is defined.
  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
    // Make sure there is no other joint with the same name.
    DRAKE_THROW_UNLESS(!HasJointNamed(name));
    const JointType<T>& joint = model_->template AddJoint<JointType>(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
    joint_name_to_index_[name] = joint.get_index();
    return joint;
  };

  /// Adds a new force element model of type `ForceElementType` to `this`
  /// %MultibodyPlant.
  /// The arguments to this method `args` are forwarded to `ForceElementType`'s
  /// constructor.
  /// @returns A constant reference to the new ForceElement just added, which
  ///          will remain valid for the lifetime of `this` %MultibodyPlant.
  ///
  /// The newly created `ForceElementType` object will be specialized on the
  /// scalar type T of this %MultibodyTree.
  template<template<typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args) {
    return model_->template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
  }
  /// @}

  /// @name Methods to retrieve multibody elements by name.
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

  /// Given the string `name` that uniquely identifies a single body in `this`
  /// model, this method returns a constant reference to that rigid body.
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

  /// Given the string `name` that uniquely identifies a single joint in `this`
  /// model, this method returns a constant reference to that joint.
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

  /// Templatized version to retrieve a joint of a specific `JointType` by its
  /// name, which is a string that uniquely identifies a single joint in `this`
  /// model.
  /// @throws std::logic_error if there is no joint with the requested name.
  /// @see HasJointNamed() to query if there exists a joint in `this` model with
  /// a given specified name.
  template <template<typename> class JointType>
  const JointType<T>& GetJointByName(const std::string& name) const {
    const JointType<T>* joint =
        dynamic_cast<const JointType<T>*>(&GetJointByName(name));
    if (joint == nullptr) {
      throw std::logic_error("Joint '" + name + "' is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "'.");
    }
    return *joint;
  }
  /// @}

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& get_world_body() const {
    return model_->get_world_body();
  }

  /// Returns `true` if this %MultibodyPlant was finalized with a call to
  /// Finalize() after all multibody elements were added, and `false` otherwise.
  /// @see Finalize().
  bool is_finalized() const { return model_->topology_is_valid(); }

  /// This method must be called after all elements in the tree (joints, bodies,
  /// force elements, constraints) were added and before any computations are
  /// performed.
  /// It essentially compiles all the necessary "topological information", i.e.
  /// how bodies, joints and, any other elements connect with each other, and
  /// performs all the required pre-processing to perform computations at a
  /// later stage.
  ///
  /// If the finalize stage is successful, the topology of this %MultibodyTree
  /// is validated, meaning that the topology is up-to-date after this call.
  /// No more multibody tree elements can be added after a call to Finalize().
  ///
  /// @throws std::logic_error If users attempt to call this method on an
  ///         already finalized %MultibodyTree.
  void Finalize();

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
