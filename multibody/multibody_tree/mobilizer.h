#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/math/spatial_acceleration.h"
#include "drake/multibody/multibody_tree/math/spatial_force.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;
namespace internal {
template<typename T> class BodyNode;
}

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
/// <h3>Tree Structure</h3>
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
/// <h3>Mathematical Description of a %Mobilizer</h3>
///
/// A %Mobilizer describes the kinematics relationship between an inboard frame
/// F and an outboard frame M, introducing an nq-dimensional vector of
/// generalized coordinates q and an nv-dimensional vector of generalized
/// velocities v. Notice that in general `nq != nv`, though `nq == nv` is a very
/// common case. The kinematic relationships introduced by a %Mobilizer are
/// fully specified by, [Seth 2010]:
/// - X_FM(q):
///     The pose of the outboard frame M as measured and expressed in the
///     inboard frame F, as a function of the mobilizer's generalized positions.
///     This pose is computed by CalcAcrossMobilizerTransform().
/// - H_FM(q):
///     the geometric Jacobian matrix describing the relationship between
///     generalized velocities `v  ∈ ℝⁿᵛ` and the spatial velocity `V_FM  ∈ M⁶`.
///     This Jacobian can be thought of as the application:
///     `v ∈ ℝⁿᵛ → M⁶: V_FM(q, v) = H_FM(q) * v`, where M⁶ is the vector space
///     of "motion vectors" (be aware that while M⁶ is introduced in
///     [Featherstone 2008, Ch. 2] spatial velocities in Drake are not Plücker
///     vectors as in Featherstone's book). A %Mobilizer implements this
///     operator in the method CalcAcrossMobilizerSpatialVelocity().
/// - H_FMᵀ(q):
///     The transpose of the geometric Jacobian `H_FM(q)` describing the
///     relationship between the spatial force `F_Mo_F ∈ F⁶` and the generalized
///     forces `tau ∈ ℝⁿᵛ`, where F⁶ is the vector space of "force vectors"
///     (be aware that while F⁶ is introduced in [Featherstone 2008, Ch. 2]
///     spatial forces in Drake are not Plücker vectors as in Featherstone's
///     book.) This mathematical object can be thought of as the application:
///     `F_Mo_F ∈ F⁶ → ℝⁿᵛ: tau = H_FMᵀ(q) * F_Mo_F`, where `Mo` is M's origin
///     (see @ref multibody_frames_and_bodies for the monogram notation in use.)
///     A %Mobilizer implements this operator in the method
///     ProjectSpatialForce().
/// - Hdot_FM(q, v):
///     The time derivative of the Jacobian matrix involved in the computation
///     of the spatial acceleration `A_FM(q, v, v̇)` between the F and M frames
///     as the application:
///     `v̇ ∈ ℝⁿᵛ → M⁶: A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q, v) * v`.
///     A %Mobilizer implements this application in
///     CalcAcrossMobilizerSpatialAcceleration().
/// - N(q):
///     The kinematic coupling matrix describing the relationship between the
///     rate of change of generalized coordinates and the generalized velocities
///     by `q̇ = N(q)⋅v`, [Seth 2010]. N(q) is an `nq x nv` matrix. A
///     %Mobilizer implements this application in MapVelocityToQDot().
/// - N⁺(q):
///     The left pseudo-inverse of `N(q)`. `N⁺(q)` can be used to invert the
///     relationship `q̇ = N(q)⋅v` without residual error, provided that `q̇` is
///     in the range space of `N(q)` (that is, if it *could* have been produced
///     as `q̇ = N(q)⋅v` for some `v`). The application `v = N⁺(q)⋅q̇` is
///     implemented in MapQDotToVelocity().
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
/// [Seth 2010]. The Jacobian or "hinge" matrix `H_FM(q)` is introduced in
/// [Jain 2010], though be aware that what [Jain 2010] calls the hinge matrix is
/// the transpose of the Jacobian H_FM matrix here in Drake.
/// For details in the monogram notation used above please refer to
/// @ref multibody_spatial_algebra.
///
/// %Mobilizer is an abstract base class defining the minimum functionality that
/// derived %Mobilizer objects must implement in order to fully define the
/// kinematic relationship between the two frames they connect. Geometric and
/// analytical Jacobian matrices in the context of differential kinematics are
/// described in [Sciavicco 2000].
///
/// <h4>Relation between the analytical and geometric Jacobians</h4>
///
/// The time derivative of the across-mobilizer transform `X_FM` is intimately
/// related to the across-mobilizer spatial velocity `V_FM`. This relationship
/// immediately implies a relationship between the analytical Jacobian
/// `dX_FM/dq` and the geometric Jacobian matrix `H_FM`.
/// The linear component of the spatial velocity `V_FM` relates to the time
/// derivative of `X_FM` by: <pre>
///   v_FM = V_FM.translational() = dp_FM/dt = Xdot_FM.translational()
/// </pre>
/// where `p_FM = X_FM.translational()` and `Xdot_FM = dX_FM/dt`. The time
/// derivative of `p_FM` can be rewritten as: <pre>
///   dp_FM/dt = dp_FM/dq * N(q) * v = Hv_FM * v
/// </pre>
/// where `Hv_FM` denotes the last three rows in `H_FM` related with the
/// translational component of the Jacobian matrix
/// Therefore: <pre>
///   Hv_FM = dp_FM/dq(q) * N(q)
/// </pre>
///
/// Similarly, for the rotational component: <pre>
///  dR_FM/dt = Xdot_FM.linear() = [w_FM] * R_FM = [Hw_FM * v] * R_FM
/// </pre>
/// where `[w_FM]` is the cross product matrix of the across-mobilizer angular
/// velocity `w_FM`, `R_FM` is the orientation of M in F, and `Hw_FM`
/// corresponds to the first three rows in `H_FM` related to the angular
/// component of the geometric Jacobian matrix.
/// The time derivative of the orientation `R_FM` can be expressed in terms of
/// the analytic Jacobian of `R_FM` as: <pre>
///   dR_FM/dt = dR_FM/dq * N(q) * v
/// </pre>
/// These last two equations show that the angular components of the Jacobian
/// matrix `Hw_FM` are directly related to the gradients of the rotation
/// matrix `R_FM`. This relationhip is: <pre>
///   [Hwi_FM(q)] * R_FM(q) = dR_FM/dqi(q) * N(q)
/// </pre>
/// corresponding to the i-th generalized position `qi` where `Hwi_FM(q)` is the
/// i-th column of `Hw_FM(q)` and `dR_FM/dqi(q)` is the partial derivative of
/// `R_FM` with respect to the i-th generalized coordinate for this mobilizer.
///
/// <h4>Active forces and power</h4>
///
/// The power generated by a mobilizer can be computed in two equivalent ways.
/// That is, the power can be computed in terms of the spatial force `F_Mo` and
/// the spatial velocity `V_FM` as: <pre>
///   P = F_Moᵀ * V_FM
/// </pre>
/// or in terms of the generalized forces `tau = H_FMᵀ(q) * F_Mo` and the
/// generalized velocities v as: <pre>
///   P = tauᵀ * v
/// </pre>
/// Notice that spatial forces in the null space of `H_FM(q)` do not perform any
/// work.
/// Since the result from the previous two expressions must be equal, the
/// Jacobian operator `H_FM(q)` and the transpose operator `H_FMᵀ(q)`
/// are constrained by: <pre>
///   (H_FMᵀ(q) * F) * v = Fᵀ * (H_FM(q) * v), ∀ v ∈ ℝⁿᵛ ∧ `F ∈ F⁶`
/// </pre>
/// Therefore, this enforces a relationship to the operations implemented by
/// CalcAcrossMobilizerSpatialVelocity() and ProjectSpatialForce() for any
/// %Mobilizer object.
///
/// - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
///               algorithms. Springer Science & Business Media.
/// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
///               Minimal formulation of joint motion for biomechanisms.
///               Nonlinear dynamics, 62(1), pp.291-303.
/// - [Sciavicco 2000] Sciavicco, L. and Siciliano, B., 2000. Modelling and
///               control of robot manipulators, 2nd Edn. Springer.
/// - [Featherstone 2008] Featherstone, R., 2008. Rigid body dynamics
///                       algorithms. Springer.
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

  /// @name Methods that define a %Mobilizer
  /// @{

  /// Sets the `state` to what will be considered to be the _zero_ configuration
  /// for `this` mobilizer. For most mobilizers the _zero_ configuration
  /// corresponds to the value of generalized positions at which the inboard
  /// frame F and the outboard frame coincide or, in other words, when
  /// `X_FM = Id` is the identity pose. In the general case however, the zero
  /// configuration will correspond to a value of the generalized positions for
  /// which `X_FM = X_FM_ref` where `X_FM_ref` may generally be different from
  /// the identity transformation.
  /// In other words, `X_FM_ref = CalcAcrossMobilizerTransform(ref_context)`
  /// where `ref_context` is a Context storing a State set to the zero
  /// configuration with set_zero_state().
  /// In addition, all generalized velocities are set to zero in the _zero_
  /// configuration.
  ///
  /// Most often the _zero_ configuration will correspond to setting
  /// the vector of generalized positions related to this mobilizer to zero.
  /// However, in the general case, setting all generalized coordinates to zero
  /// does not correspond to the _zero_ configuration and it might even not
  /// represent a mathematicaly valid one. Consider for instance a quaternion
  /// mobilizer, for which its _zero_ configuration corresponds to the
  /// quaternion [1, 0, 0, 0].
  virtual void set_zero_state(const systems::Context<T>& context,
                              systems::State<T>* state) const = 0;

  /// Sets the state stored in `context` to a _zero configuration_ as defined by
  /// set_zero_state().
  /// See set_zero_state() for details.
  void set_zero_configuration(systems::Context<T>* context) const {
    set_zero_state(*context, &context->get_mutable_state());
  }

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

  /// Computes the across-mobilizer spatial velocity `V_FM(q, v)` of the
  /// outboard frame M in the inboard frame F.
  /// This method can be thought of as the application of the operator `H_FM(q)`
  /// to the input vector of generalized velocities `v`, i.e. the output of this
  /// method is the application `v ∈ ℝⁿᵛ → M⁶: V_FM(q, v) = H_FM(q) * v`, where
  /// `nv` is the number of generalized velocities of this mobilizer (see
  /// get_num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  /// aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// velocities in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore we say this method is the _operator form_ of the Jacobian
  /// matrix `H_FM(q)`.
  /// This method aborts in Debug builds if the dimension of the input vector of
  /// generalized velocities has a size different from get_num_velocities().
  ///
  /// @param[in] context The context of the parent tree that owns this
  /// mobilizer. This mobilizer's generalized positions q are inferred from this
  /// context.
  /// @param[in] v A vector of generalized velocities. It must live in ℝⁿᵛ.
  /// @retval V_FM The across-mobilizer spatial velocity of the outboard frame
  /// M measured and expressed in the inboard frame F.
  virtual SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const = 0;

  /// Computes the across-mobilizer spatial accelerations `A_FM(q, v, v̇)` of the
  /// outboard frame M in the inboard frame F.
  /// This method can be thought of as the application of the operation
  /// `v̇ ∈ ℝⁿᵛ → M⁶: A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q) * v`, where
  /// `nv` is the number of generalized velocities of this mobilizer (see
  /// get_num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  /// aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// vectors in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore, we say this method is in its _operator form_; the Jacobian
  /// matrix `H_FM(q)` is not explicitly formed.
  /// This method aborts in Debug builds if the dimension of the input vector of
  /// generalized accelerations has a size different from get_num_velocities().
  ///
  /// @param[in] context
  ///   The context of the parent tree that owns this mobilizer. This
  ///   mobilizer's generalized positions q and generalized velocities v are
  ///   taken from this context.
  /// @param[in] vdot
  ///   The vector of generalized velocities' time derivatives v̇. It must live
  ///   in ℝⁿᵛ.
  /// @retval A_FM
  ///   The across-mobilizer spatial acceleration of the outboard frame M
  ///   measured and expressed in the inboard frame F.
  virtual SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const = 0;

  /// Projects the spatial force `F_Mo` on `this` mobilizer's outboard frame
  /// M onto the sub-space of motions spanned by the geometric Jacobian
  /// `H_FM(q)` to obtain the generalized forces `tau` (i.e. the active
  /// components of `F_Mo`).
  /// @see CalcAcrossMobilizerSpatialVelocity() and this class' documentation
  /// for the definition of the geometric Jacobian `H_FM(q)`.
  ///
  /// This method can be thought of as the application of the transpose operator
  /// `H_FMᵀ(q)` to the input spatial force `F_Mo_F`, i.e. the output of this
  /// method is the application `F_Mo_F ∈ F⁶ → ℝⁿᵛ: tau = H_FMᵀ(q) * F_Mo_F`,
  /// where `nv` is the number of generalized velocities of this mobilizer (see
  /// get_num_velocities()) and F⁶ is the vector space of "force vectors" (be
  /// aware that while F⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// forces in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore we say this method is the _operator form_ of the Jacobian
  /// matrix transpose `H_FMᵀ(q)`.
  /// This method aborts in Debug builds if the dimension of the output vector
  /// of generalized forces has a size different from get_num_velocities().
  ///
  /// @param[in] context
  ///   The context of the parent tree that owns this mobilizer. This
  ///   mobilizer's generalized positions q are stored in this context.
  /// @param[in] F_Mo_F
  ///   A SpatialForce applied at `this` mobilizer's outboard frame origin `Mo`,
  ///   expressed in the inboard frame F.
  /// @retval tau
  ///   The vector of generalized forces. It must live in ℝⁿᵛ.
  virtual void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const = 0;

  /// Computes the kinematic mapping `q̇ = N(q)⋅v` between generalized
  /// velocities v and time derivatives of the generalized positions `qdot`.
  /// The generalized positions vector is stored in `context`.
  virtual void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const = 0;

  /// Computes the mapping `v = N⁺(q)⋅q̇` from time derivatives of the
  /// generalized positions `qdot` to generalized velocities v, where `N⁺(q)` is
  /// the left pseudo-inverse of `N(q)` defined by MapVelocityToQDot().
  /// The generalized positions vector is stored in `context`.
  virtual void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const = 0;
  /// @}

  /// Returns a const Eigen expression of the vector of generalized positions
  /// for `this` mobilizer from a vector `q_array` of generalized positions for
  /// the entire MultibodyTree model.
  /// This method aborts if `q_array` is not of size
  /// MultibodyTree::get_num_positions().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_positions_from_array(const Eigen::Ref<const VectorX<T>>& q_array) const {
    DRAKE_DEMAND(
        q_array.size() == this->get_parent_tree().get_num_positions());
    return q_array.segment(topology_.positions_start,
                           topology_.num_positions);
  }

  /// Mutable version of get_positions_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> get_mutable_positions_from_array(
      EigenPtr<VectorX<T>> q_array) const {
    DRAKE_DEMAND(q_array != nullptr);
    DRAKE_DEMAND(
        q_array->size() == this->get_parent_tree().get_num_positions());
    return q_array->segment(topology_.positions_start,
                            topology_.num_positions);
  }

  /// Returns a const Eigen expression of the vector of generalized velocities
  /// for `this` mobilizer from a vector `v_array` of generalized velocities for
  /// the entire MultibodyTree model.
  /// This method aborts if the input array is not of size
  /// MultibodyTree::get_num_velocities().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_velocities_from_array(const Eigen::Ref<const VectorX<T>>& v_array) const {
    DRAKE_DEMAND(
        v_array.size() == this->get_parent_tree().get_num_velocities());
    return v_array.segment(topology_.velocities_start_in_v,
                           topology_.num_velocities);
  }

  /// Mutable version of get_velocities_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> get_mutable_velocities_from_array(
      EigenPtr<VectorX<T>> v_array) const {
    DRAKE_DEMAND(v_array != nullptr);
    DRAKE_DEMAND(
        v_array->size() == this->get_parent_tree().get_num_velocities());
    return v_array->segment(topology_.velocities_start_in_v,
                            topology_.num_velocities);
  }

  /// Returns a const Eigen expression of the vector of generalized
  /// accelerations for `this` mobilizer from a vector `vdot_array` of
  /// generalized accelerations for the entire MultibodyTree model.
  /// This method aborts if the input array is not of size
  /// MultibodyTree::get_num_velocities().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_accelerations_from_array(
      const Eigen::Ref<const VectorX<T>>& vdot_array) const {
    return get_velocities_from_array(vdot_array);
  }

  /// Mutable version of get_accelerations_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>>
  get_mutable_accelerations_from_array(
      EigenPtr<VectorX<T>> vdot_array) const {
    return get_mutable_velocities_from_array(vdot_array);
  }

  /// Returns a const Eigen expression of the vector of generalized forces
  /// for `this` mobilizer from a vector of generalized forces for the
  /// entire MultibodyTree model.
  /// This method aborts if the input array is not of size
  /// MultibodyTree::get_num_velocities().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_generalized_forces_from_array(
      const Eigen::Ref<const VectorX<T>>& tau_array) const {
    return get_velocities_from_array(tau_array);
  }

  /// Mutable version of get_generalized_forces_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>>
  get_mutable_generalized_forces_from_array(
      EigenPtr<VectorX<T>> tau_array) const {
    return get_mutable_velocities_from_array(tau_array);
  }

  /// NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  /// be created. This method is mostly intended to be called by
  /// MultibodyTree::CloneToScalar(). Most users should not call this clone
  /// method directly but rather clone the entire parent MultibodyTree if
  /// needed.
  /// @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }

  /// For MultibodyTree internal use only.
  virtual std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node,
      const Body<T>* body, const Mobilizer<T>* mobilizer) const = 0;

 protected:
  /// @name Methods to make a clone templated on different scalar types.
  ///
  /// The only const argument to these methods is the new MultibodyTree clone
  /// under construction, which is required to already own the clones of the
  /// inboard and outboard frames of the mobilizer being cloned.
  /// @{

  /// Clones this %Mobilizer (templated on T) to a mobilizer templated on
  /// `double`.
  /// @pre Inboard and outboard frames for this mobilizer already have a clone
  /// in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Mobilizer (templated on T) to a mobilizer templated on
  /// AutoDiffXd.
  /// @pre Inboard and outboard frames for this mobilizer already have a clone
  /// in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;
  /// @}

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
