#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;
template<typename T> class BodyNode;

/// %Mobilizer is a fundamental object within Drake's multibody engine used to
/// specify the allowed motions between two Frame objects within a
/// MultibodyTree. Specifying the allowed motions between two Frame objects
/// effectively also specifies a kinematic relationship between the two bodies
/// associated with those two frames. Consider the following example to build a
/// simple pendulum system:
///
/// @code
/// MultibodyTree<double> model;
/// // ... Code here to setup quantities below as mass, com, X_BP, etc. ...
/// const Body<double>& pendulum =
///   model.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world frame using a
/// // RevoluteMobilizer. To do so we define a pin frame P rigidly attached to
/// // the pendulum body.
/// FixedOffsetFrame<double>& pin_frame =
///   model.AddFrame<FixedOffsetFrame>(
///     pendulum.get_body_frame(),
///     X_BP /* pose of pin frame P in body frame B */);
/// // The mobilizer connects the world frame and the pin frame effectively
/// // adding the single degree of freedom describing this system. In this
/// // regard, the the role of a mobilizer is equivalent but conceptually
/// // different than a set of constraints that effectively remove all degrees
/// // of freedom but the one permitting rotation about the z-axis.
/// const RevoluteMobilizer<double>& revolute_mobilizer =
///   model.AddMobilizer<RevoluteMobilizer>(
///     model.get_world_frame(), /* inboard frame */
///     pin_frame, /* outboard frame */
///     Vector3d::UnitZ() /* revolute axis in this case */));
/// @endcode
///
/// A %Mobilizer induces a tree structure within a MultibodyTree
/// model, connecting an inboard (topologically closer to the world) frame to an
/// outboard (topologically further from the world) frame. Every time a
/// %Mobilizer is added to a MultibodyTree (using the
/// MultibodyTree::AddMobilizer() method), a number of degrees of
/// freedom associated with the particular type of %Mobilizer are added to the
/// multibody system. In the example above for the single pendulum, adding a
/// RevoluteMobilizer has two purposes:
/// - It defines the tree structure of the model. World is the inboard body
///   while "pendulum" is the outboard body in the MultibodyTree.
/// - It informs the MultibodyTree of the degrees of freedom granted by the
///   revolute mobilizer between the two frames it connects.
/// - It defines a permissible motion space spanned by the generalized
///   coordinates introduced by the mobilizer.
///
/// A %Mobilizer describes the kinematics relationship between an inboard frame
/// F and an outboard frame M, introducing an nq-dimensional vector of
/// generalized coordinates q and an nv-dimensional vector of generalized
/// velocities v. Notice that in general `nq != nv`, though `nq == nv` is a very
/// common case. The kinematic relationships introduced by a %Mobilizer are
/// fully specified by, [Seth 2010]:
/// - X_FM(q): The pose of the outboard frame M as measured and expressed in the
///            inboard frame F, as a function of the mobilizer's generalized
///            positions. This pose is computed by
///            CalcAcrossMobilizerTransform().
/// - H_FM(q): the Jacobian matrix describing the relationship between
///            generalized velocities v and the spatial velocity `V_FM` by
///            `V_FM(q, v) = H_FM(q) * v`. `H_FM` is a `6 x nv` matrix.
///            See [Jain 2010] for details.
/// - Hdot_FM(q): The time derivative of the Jacobian matrix allowing to compute
///               the spatial acceleration between the F and M frames:
///               `A_FM(q, v) = Vdot_FM(q, v) = H_FM(q) * vdot + Hdot_FM(q) * v`
/// - N(q): The kinematic coupling matrix describing the relationship between
///         the rate of change of generalized coordinates and the generalized
///         velocities by `qdot = N(q) * v`, [Seth 2010]. N(q) is an `nq x nv`
///         matrix.
///
/// In general, `nv != nq`. As an example, consider a quaternion mobilizer that
/// would allow frame M to move freely with respect to frame F. For such a
/// mobilizer the generalized positions vector might contain a quaternion to
/// describe rotations plus a position vector to describe translations. However,
/// we might choose the angular velocity `w_FM` and the linear velocity `v_FM`
/// as the generalized velocities (or more generally, the spatial velocity
/// `V_FM`.) In such a case `nq = 7` (4 dofs for a quaternion plus 3 dofs for a
/// position vector) and `nv = 6` (3 dofs for an angular velocity and 3 dofs for
/// a linear velocity).
///
/// For a detailed discussion on the concept of a mobilizer please refer to
/// [Seth 2010]. The Jacobian or "Hinge" matrix `H_FM(q)` is introduced in
/// [Jain 2010], though be aware that what [Jain 2010] calls the hinge matrix is
/// the transpose of the Jacobian H_FM matrix here in Drake.
/// For details in the monogram notation used above please refer to
/// @ref multibody_spatial_algebra.
///
/// %Mobilizer is an abstract base class defining the minimum functionality that
/// derived %Mobilizer objects must implement in order to fully define the
/// kinematic relationship between the two frames they connect.
///
/// - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
///               algorithms. Springer Science & Business Media.
/// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
///               Minimal formulation of joint motion for biomechanisms.
///               Nonlinear dynamics, 62(1), pp.291-303.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Mobilizer : public MultibodyTreeElement<Mobilizer<T>, MobilizerIndex> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Mobilizer)

  /// The minimum amount of information that we need to define a %Mobilizer is
  /// the knowledge of the inboard and outboard frames it connects.
  /// Subclasses of %Mobilizer are therefore required to provide this
  /// information in their respective constructors.
  /// @throws std::runtime_error if `inboard_frame` and `outboard_frame`
  /// reference the same frame object.
  Mobilizer(const Frame<T>& inboard_frame,
            const Frame<T>& outboard_frame) :
      inboard_frame_(inboard_frame), outboard_frame_(outboard_frame) {
    // Verify they are not the same frame.
    if (&inboard_frame == &outboard_frame) {
      throw std::runtime_error(
          "The provided inboard and outboard frames reference the same object");
    }
  }

  /// Returns the number of generalized coordinates granted by this mobilizer.
  /// As an example, consider RevoluteMobilizer, for which
  /// `get_num_positions() == 1` since RevoluteMobilizer adds a single
  /// generalized coordinate representing the rotational degree of freedom about
  /// a given axis between the inboard and outboard frames. Another example
  /// would be a 6 DOF "free" mobilizer internally using a quaternion
  /// representation to parametrize free rotations and a position vector to
  /// parametrize free translations; this method would return 7 (a quaternion
  /// plus a position vector).
  /// @see get_num_velocities()
  virtual int get_num_positions() const = 0;

  /// Returns the number of generalized velocities granted by this mobilizer.
  /// Given that all physics occurs in the generalized velocities space, the
  /// number of generalized velocities exactly matches the number of degrees of
  /// freedom granted by the mobilizer.
  /// As an example, consider RevoluteMobilizer, for which
  /// `get_num_velocities() == 1` since for RevoluteMobilizer its one and only
  /// generalized velocity describes the magnitude of the angular velocity about
  /// a given axis between the inboard and outboard frames.
  /// @see get_num_positions()
  virtual int get_num_velocities() const = 0;

  /// Returns a constant reference to the inboard frame.
  const Frame<T>& get_inboard_frame() const {
    return inboard_frame_;
  }

  /// Returns a constant reference to the outboard frame.
  const Frame<T>& get_outboard_frame() const {
    return outboard_frame_;
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's inboard frame.
  const Body<T>& get_inboard_body() const {
    return get_inboard_frame().get_body();
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's outboard frame.
  const Body<T>& get_outboard_body() const {
    return get_outboard_frame().get_body();
  }

  /// Returns the topology information for this mobilizer. Users should not
  /// need to call this method since MobilizerTopology is an internal
  /// bookkeeping detail.
  const MobilizerTopology& get_topology() const { return topology_; }

  /// @name Methods that Define a %Mobilizer
  /// @{

  /// Sets what will be considered to be the _zero_ configuration for `this`
  /// mobilizer. For most mobilizers the _zero_ configuration corresponds to the
  /// value of genelized positions at which the inboard frame F and the outboard
  /// frame coincide or, in other words, when `X_FM = Id` is the identity pose.
  /// This however, does not necessarily have to be the case for all mobilizers.
  /// Most often the _zero_ configuration will correspond to setting
  /// the vector of generalized positions related to this mobilizer to zero.
  /// However, in the general case, setting all generalized coordinates to zero
  /// does not correspond to the _zero_ configuration and it might even not
  /// represent a mathematicaly valid one. Consider for instance a quaternion
  /// mobilizer, for which its _zero_ configuration corresponds to the
  /// quaternion [1, 0, 0, 0].
  virtual void set_zero_configuration(systems::Context<T>* context) const = 0;

  /// Computes the across-mobilizer transform `X_FM(q)` between the inboard
  /// frame F and the outboard frame M as a function of the vector of
  /// generalized postions `q`.
  /// %Mobilizer subclasses implementing this method can retrieve the fixed-size
  /// vector of generalized positions for `this` mobilizer from `context` with:
  ///
  /// @code
  /// auto q = this->get_positions(context);
  /// @endcode
  ///
  /// Additionally, `context` can provide any other parameters the mobilizer
  /// could depend on.
  virtual Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const = 0;
  /// @}

  /// Computes position dependent kinematics associated with `this` mobilizer
  /// which includes:
  /// - X_FM(q): The pose of the outboard frame M as measured and expressed in
  ///            the inboard frame F.
  /// - H_FM(q): the Jacobian matrix describing the relationship between
  ///            generalized velocities v and the spatial velocity `V_FM` by
  ///            `V_FM(q, v) = H_FM(q) * v`.
  /// - Hdot_FM(q): The time derivative of the Jacobian matrix which allows
  ///               computing the spatial acceleration between the F and M
  ///               frames as:
  ///               `A_FM(q, v, vdot) = H_FM(q) * vdot + Hdot_FM(q) * v`
  /// - N(q): The kinematic coupling matrix describing the relationship between
  ///         the rate of change of generalized coordinates and the generalized
  ///         velocities by `q̇ = N(q) * v`.
  ///
  /// This method is used by MultibodyTree to update the position kinematics
  /// quantities associated with `this` mobilizer. MultibodyTree will always
  /// provide a valid PositionKinematicsCache pointer, otherwise this method
  /// aborts in Debug builds.
  void CalcPositionKinematicsCache(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    DRAKE_ASSERT(pc != nullptr);
    Isometry3<T>& X_FM = pc->get_mutable_X_FM(topology_.body_node);
    X_FM = this->CalcAcrossMobilizerTransform(context);
  }

  /// For MultibodyTree internal use only.
  virtual std::unique_ptr<BodyNode<T>> CreateBodyNode(
      const Body<T>& body, const Mobilizer<T>* mobilizer) const = 0;

 private:
  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each mobilizer retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_mobilizer(this->get_index());
  }

  const Frame<T>& inboard_frame_;
  const Frame<T>& outboard_frame_;
  MobilizerTopology topology_;
};

}  // namespace multibody
}  // namespace drake
