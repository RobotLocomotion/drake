#pragma once

#include <memory>
#include <string>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/multibody/math/spatial_algebra.h"
#include "drake/multibody/topology/forest.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <typename T>
class RigidBody;

namespace internal {

template <typename T>
class BodyNode;

// %Mobilizer is a fundamental object within Drake's multibody engine used to
// specify the allowed motions between two Frame objects within a
// MultibodyTree. Specifying the allowed motions between two Frame objects
// effectively also specifies a kinematic relationship between the two bodies
// associated with those two frames. Consider the following example to build a
// simple pendulum system:
//
// @code
// MultibodyTree<double> model;
// // ... Code here to setup quantities below as mass, com, X_BP, etc. ...
// const RigidBody<double>& pendulum =
//   model.AddRigidBody(SpatialInertia<double>(mass, com, unit_inertia));
// // We will connect the pendulum body to the world frame using a
// // RevoluteMobilizer. To do so we define a pin frame P rigidly attached to
// // the pendulum body.
// FixedOffsetFrame<double>& pin_frame =
//   model.AddFrame<FixedOffsetFrame>(
//     pendulum.body_frame(),
//     X_BP /* pose of pin frame P in body frame B */);
// // The mobilizer connects the world frame and the pin frame effectively
// // adding the single degree of freedom describing this system. In this
// // regard, the role of a mobilizer is equivalent but conceptually
// // different than a set of constraints that effectively remove all degrees
// // of freedom but the one permitting rotation about the z-axis.
// const RevoluteMobilizer<double>& revolute_mobilizer =
//   model.AddMobilizer<RevoluteMobilizer>(
//     model.world_frame(), /* inboard frame */
//     pin_frame, /* outboard frame */
//     Vector3d::UnitZ() /* revolute axis in this case */));
// @endcode
//
// <h3>Tree Structure</h3>
//
// A %Mobilizer induces a tree structure within a MultibodyTree
// model, connecting an inboard (topologically closer to the world) frame to an
// outboard (topologically further from the world) frame. Every time a
// %Mobilizer is added to a MultibodyTree (using the
// MultibodyTree::AddMobilizer() method), a number of degrees of
// freedom associated with the particular type of %Mobilizer are added to the
// multibody system. In the example above for the single pendulum, adding a
// RevoluteMobilizer has two purposes:
//
// - It defines the tree structure of the model. World is the inboard body
//   while "pendulum" is the outboard body in the MultibodyTree.
// - It informs the MultibodyTree of the degrees of freedom granted by the
//   revolute mobilizer between the two frames it connects.
// - It defines a permissible motion space spanned by the generalized
//   coordinates introduced by the mobilizer.
//
// <h3>Mathematical Description of a %Mobilizer</h3>
//
// A %Mobilizer describes the kinematics relationship between an inboard frame
// F and an outboard frame M, introducing an nq-dimensional vector of
// generalized coordinates q and an nv-dimensional vector of generalized
// velocities v. Notice that in general `nq != nv`, though `nq == nv` is a very
// common case. The kinematic relationships introduced by a %Mobilizer are
// fully specified by, [Seth 2010]. The monogram notation used below for X_FM,
// V_FM, F_Mo_F, etc., are described in @ref multibody_frames_and_bodies.
//
// - X_FM(q):
//     The outboard frame M's pose as measured and expressed in the inboard
//     frame F, as a function of the mobilizer's generalized positions `q`.
//     This pose is computed by CalcAcrossMobilizerTransform().
// - H_FM(q):
//     The `6 x nv` mobilizer hinge matrix `H_FM` relates `V_FM` (outboard
//     frame M's spatial velocity in its inboard frame F, expressed in F) to
//     the mobilizer's `nv` generalized velocities (or mobilities) `v` as
//     `V_FM = H_FM * v`.  The method CalcAcrossMobilizerSpatialVelocity()
//     calculates `V_FM`.  Be aware that Drake's spatial velocities are not the
//     Plücker vectors defined in [Featherstone 2008, Ch. 2].
//     Note: `H_FM` is only a function of the `nq` generalized positions `q`.
// - H_FMᵀ(q):
//     H_FMᵀ is the `nv x 6` matrix transpose of `H_FM`.  It relates the `nv`
//     generalized forces `tau` to `F_Mo_F` (the spatial force on frame M at
//     point Mo, expressed in F) as `tau = H_FMᵀ ⋅ F_Mo_F`
//     The %Mobilizer method ProjectSpatialForce() calculates `tau`.
//     Be aware that Drake's spatial forces are not the Plücker vectors defined
//     in [Featherstone 2008, Ch. 2].
// - Hdot_FM(q, v):
//     The time derivative of the mobilizer hinge matrix `H_FM` is used in the
//     calculation of `A_FM(q, v, v̇)` (outboard frame M's spatial acceleration
//     in its inboard frame F, expressed in F) as
//     `A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q, v) * v`.  The %Mobilizer method
//     CalcAcrossMobilizerSpatialAcceleration() calculates `A_FM`.
// - N(q):
//     This `nq x nv` kinematic coupling matrix relates q̇ (the time-derivative
//     of the nq mobilizer's generalized positions) to `v` (the mobilizer's
//     generalized velocities) as `q̇ = N(q) * v`, [Seth 2010].
//     The %Mobilizer method MapVelocityToQDot() calculates `N(q)`.
// - N⁺(q):
//     The left pseudo-inverse of `N(q)`. `N⁺(q)` can be used to invert the
//     relationship `q̇ = N(q) * v` without residual error, provided that `q̇` is
//     in the range space of `N(q)` (that is, if it *could* have been produced
//     as `q̇ = N(q) * v` for some `v`). The application `v = N⁺(q) * q̇` is
//     implemented in MapQDotToVelocity().
//
// In general, `nv != nq`. As an example, consider a quaternion mobilizer that
// would allow frame M to move freely with respect to frame F. For such a
// mobilizer the generalized positions vector might contain a quaternion to
// describe rotations plus a position vector to describe translations. However,
// we might choose the angular velocity `w_FM` and the linear velocity `v_FM`
// as the generalized velocities (or more generally, the spatial velocity
// `V_FM`.) In such a case `nq = 7` (4 generalized coordinates for a quaternion
// plus 3 generalized coordinates for a position vector) and `nv = 6` (3
// generalized speeds for an angular velocity and 3 generalized speeds for a
// linear velocity).
//
// For a detailed discussion on the concept of a mobilizer please refer to
// [Seth 2010]. The mobilizer "hinge" matrix `H_FM(q)` is introduced in
// [Jain 2010], though be aware that what [Jain 2010] calls the hinge matrix is
// the transpose of the mobilizer hinge matrix H_FM matrix here in Drake.
// For details in the monogram notation used above please refer to
// @ref multibody_spatial_algebra.
//
// %Mobilizer is an abstract base class defining the minimum functionality that
// derived %Mobilizer objects must implement in order to fully define the
// kinematic relationship between the two frames they connect.
//
// <h4>Relation between hinge matrix and Jacobians</h4>
//
// The relationship between the across-mobilizer spatial velocity `V_FM` and
// the time derivative of the across-mobilizer transform `X_FM` is similar to
// the relationship between the rigid transform Jacobian Jq_X_VM (partial
// derivatives of rigid transform X_FM with respect to generalized positions q)
// and the Drake mobilizer hinge matrix `H_FM` (partial derivatives of
// across-mobilizer q̇ with respect to generalized velocities v).
//
// The translational velocity v_FM component of the spatial velocity `V_FM` is
// defined as the time derivative of the position vector p_FM in `X_FM`. <pre>
//   v_FM = dp_FM/dt = ∂p_FM/∂q * q̇ = ∂p_FM/∂q * N(q) * v = Hv_FM * v
// </pre>
// where `Hv_FM = ∂p_FM/∂q * N(q)` is the last three rows in `H_FM`.
//
// The angular velocity w_FM component of the spatial velocity `V_FM` can be
// related to the time derivative of the rotation matrix R_FM in `X_FM`. This
// complicated relationship can be written in terms of the skew symmetric
// angular velocity matrix [w_FM] as <pre>
//  [w_FM] = d(R_FM)/dt * (R_FM)ᵀ
// </pre>
// The ordinary time-derivative of the rotation matrix R_FM is <pre>
//   d(R_FM)/dt = ∂R/∂q * q̇ = ∂R/∂q * N(q) * v
// </pre>
// Combining the previous two equations leads to <pre>
//  [w_FM] = ∂R/∂q * N(q) * v * (R_FM)ᵀ
// </pre>
// Post-multiplying both sides of the previous equation by R_FM gives <pre>
//  [w_FM] * R_FM = ∂R/∂q * N(q) * v
// </pre>
// `Hw_FM` is the first three rows in `H_FM`, defined by context as <pre>
//  Hw_FM * R_FM = ∂R/∂q * N(q)
// </pre>
//
// <h4>Active forces and power</h4>
//
// The power generated by a mobilizer can be computed in two equivalent ways.
// That is, the power can be computed in terms of the spatial force `F_Mo` and
// the spatial velocity `V_FM` as: <pre>
//   P = F_Moᵀ * V_FM
// </pre>
// or in terms of the generalized forces `tau = H_FMᵀ(q) ⋅ F_Mo` and the
// generalized velocities v as: <pre>
//   P = tauᵀ * v
// </pre>
// Notice that spatial forces in the null space of `H_FM(q)` do not perform any
// work.  Since the result from the previous two expressions must be equal, the
// mobilizer hinge matrix `H_FM(q)` and its transpose `H_FMᵀ(q)` are
// constrained by: <pre>
//   (H_FMᵀ(q) * F) * v = Fᵀ * (H_FM(q) * v), ∀ v ∈ ℝⁿᵛ ∧ `F ∈ F⁶`
// </pre>
// Therefore, this enforces a relationship to the operations implemented by
// CalcAcrossMobilizerSpatialVelocity() and ProjectSpatialForce() for any
// %Mobilizer object.
//
// - [Jain 2010] Jain, A., 2010. Robot and multibody dynamics: analysis and
//               algorithms. Springer Science & Business Media.
// - [Seth 2010] Seth, A., Sherman, M., Eastman, P. and Delp, S., 2010.
//               Minimal formulation of joint motion for biomechanisms.
//               Nonlinear dynamics, 62(1), pp.291-303.
// - [Sciavicco 2000] Sciavicco, L. and Siciliano, B., 2000. Modelling and
//               control of robot manipulators, 2nd Edn. Springer.
// - [Featherstone 2008] Featherstone, R., 2008. Rigid body dynamics
//                       algorithms. Springer.
//
// <h3>Interaction with the Context</h3>
//
// Some member functions of a Mobilizer take a `const Context& context` as an
// input argument. To ensure correctness of the MultibodyTreeSystem's cache
// entry dependencies, it is essential that such functions only access an
// appropriate subset the Context.
//
// A mobilizer's generalized positions q and generalized velocities v exist as
// State in the context. The mobilizer is FORBIDDEN from using any State from
// the context other than its own q and v data. Some functions are documented
// to be only a function of q (not v); those functions must not access v.
//
// A mobilizer is allowed to access any Parameters in the context that it has
// declared, from any method that takes a Context, without any further comment.
// We always conservatively assume that all Mobilizer methods depend on all of a
// Mobilizer's Parameters.
//
// A mobilizer is FORBIDDEN from being time- or input-dependent. (The context
// provides access to the current simulation time and input port values, but
// the mobilizer must not use that information.)
//
// @tparam_default_scalar
template <typename T>
class Mobilizer : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Mobilizer);

  // The minimum amount of information that we need to define a %Mobilizer is
  //   - the mobilized body (Mobod) that defines its topology within the
  //     SpanningForest, and
  //   - the specific Frames of the inboard and outboard Links that form
  //     the inboard F frame and outboard M frame for this Mobilizer.
  //
  // Subclasses of %Mobilizer are therefore required to provide this
  // information in their respective constructors.
  // @throws std::exception if `inboard_frame` and `outboard_frame`
  // reference the same frame object.
  // TODO(sherm1) Since the Mobod and Frames are required for all Mobilizers,
  //  the concrete classes shouldn't have to deal with them. Make the base
  //  class take care of those for construction and scalar conversion.
  Mobilizer(const SpanningForest::Mobod& mobod, const Frame<T>& inboard_frame,
            const Frame<T>& outboard_frame)
      : mobod_(mobod),
        inboard_frame_(inboard_frame),
        outboard_frame_(outboard_frame) {
    // Verify they are not the same frame unless this is the dummy World
    // Mobilizer. Don't reference the Mobod if the frames are OK to permit
    // some low-level mobilizer tests to use a dummy Mobod.
    const bool frames_are_different = &inboard_frame != &outboard_frame;
    if (!frames_are_different && !mobod.is_world()) {
      throw std::runtime_error(
          "The provided inboard and outboard frames reference the same object");
    }
  }

  ~Mobilizer() override;

  // Returns this element's unique index.
  MobodIndex index() const { return this->template index_impl<MobodIndex>(); }

  // Returns the number of generalized coordinates granted by this mobilizer.
  // As an example, consider RevoluteMobilizer, for which
  // `num_positions() == 1` since RevoluteMobilizer adds a single
  // generalized coordinate representing the rotational degree of freedom about
  // a given axis between the inboard and outboard frames. Another example
  // would be a 6 DOF "free" mobilizer internally using a quaternion
  // representation to parameterize free rotations and a position vector to
  // parameterize free translations; this method would return 7 (a quaternion
  // plus a position vector).
  // @see num_velocities()
  int num_positions() const { return mobod_.nq(); }

  // Returns the number of generalized velocities granted by this mobilizer.
  // Given that all physics occurs in the generalized velocities space, the
  // number of generalized velocities exactly matches the number of degrees of
  // freedom granted by the mobilizer.
  // As an example, consider RevoluteMobilizer, for which
  // `num_velocities() == 1` since for RevoluteMobilizer its one and only
  // generalized velocity describes the magnitude of the angular velocity about
  // a given axis between the inboard and outboard frames.
  // @see num_positions()
  int num_velocities() const { return mobod_.nv(); }

  // Returns the index to the first generalized position for this mobilizer
  // within the vector q of generalized positions for the full multibody
  // system.
  int position_start_in_q() const { return mobod_.q_start(); }

  // Returns the index to the first generalized velocity for this mobilizer
  // within the vector v of generalized velocities for the full multibody
  // system.
  int velocity_start_in_v() const { return mobod_.v_start(); }

  // Returns a string suffix (e.g. to be appended to the name()) to identify
  // the `k`th position in this mobilizer. Mobilizers with more than one
  // position or that don't want to use the default `q`, must override this
  // method.
  virtual std::string position_suffix(int position_index_in_mobilizer) const {
    // Mobilizers with num_positions > 1 must overload this method.
    // The method should not be called for mobilizers with num_positions < 1.
    DRAKE_DEMAND(num_positions() == 1);
    DRAKE_DEMAND(position_index_in_mobilizer == 0);
    return "q";
  }

  // Returns a string suffix (e.g. to be appended to the name()) to identify
  // the `k`th velocity in this mobilizer. Mobilizers with more than one
  // velocity or that don't want to use the default 'v' must override this
  // method.
  virtual std::string velocity_suffix(int velocity_index_in_mobilizer) const {
    // Mobilizers with num_velocities > 1 must overload this method.
    // The method should not be called for mobilizers with num_velocities < 1.
    DRAKE_DEMAND(num_velocities() == 1);
    DRAKE_DEMAND(velocity_index_in_mobilizer == 0);
    return "v";
  }

  // Returns true if this mobilizer can rotate.
  virtual bool can_rotate() const = 0;

  // Returns true if this mobilizer can translate.
  virtual bool can_translate() const = 0;

  // Returns a reference to the mobilized body (Mobod) implemented by this
  // Mobilizer.
  const SpanningForest::Mobod& mobod() const { return mobod_; }

  // Returns a constant reference to the inboard frame.
  const Frame<T>& inboard_frame() const { return inboard_frame_; }

  // Returns a constant reference to the outboard frame.
  const Frame<T>& outboard_frame() const { return outboard_frame_; }

  // Returns a constant reference to the body associated with `this`
  // mobilizer's inboard frame.
  const RigidBody<T>& inboard_body() const { return inboard_frame().body(); }

  // Returns a constant reference to the body associated with `this`
  // mobilizer's outboard frame.
  const RigidBody<T>& outboard_body() const { return outboard_frame().body(); }

  // Returns `true` if `this` mobilizer grants 6-dofs to the outboard frame.
  // Ignoring joint limits, this means this mobilizer can represent any pose
  // and any spatial velocity to machine precision.
  bool has_six_dofs() const { return num_velocities() == 6; }

  bool is_floating_base_mobilizer() const {
    return is_floating_base_mobilizer_;
  }

  // Returns `true` if `this` uses a quaternion parameterization of rotations.
  virtual bool has_quaternion_dofs() const { return false; }

  // @name         Methods that define the Mobilizer abstraction
  // For inner-loop computations, don't use this API. Use the templatized
  // APIs of the concrete mobilizers.
  // @{

  // Sets the `state` to what will be considered to be the _zero_ state
  // (position and velocity) for `this` mobilizer. For most mobilizers the
  // _zero_ position corresponds to the value of generalized positions at
  // which the inboard frame F and the outboard frame coincide or, in other
  // words, when `X_FM = Id` is the identity pose. In the general case
  // however, the zero position will correspond to a value of the
  // generalized positions for which `X_FM = X_FM_ref` where `X_FM_ref` may
  // generally be different from the identity transformation.
  // In other words, `X_FM_ref = CalcAcrossMobilizerTransform(ref_context)`
  // where `ref_context` is a Context storing a State set to the zero
  // configuration with SetZeroState().
  // In addition, all generalized velocities are set to zero in the _zero_
  // state.
  //
  // Most often the _zero_ position will correspond to setting
  // the vector of generalized positions related to this mobilizer to zero.
  // However, in the general case, setting all generalized coordinates to zero
  // does not correspond to the _zero_ position and it might even not
  // represent a mathematically valid one. Consider for instance a quaternion
  // mobilizer, for which its _zero_ position corresponds to the quaternion
  // [1, 0, 0, 0].
  //
  // Note that the zero state may fall outside of the limits for any joints
  // associated with this type of mobilizer.
  // @see set_default_state().
  virtual void SetZeroState(const systems::Context<T>& context,
                            systems::State<T>* state) const = 0;

  // Sets the state for this mobilizer in the given State to some
  // approximation of this pose. It's up to the concrete mobilizer to figure out
  // what to do. (Only QuaternionFloatingMobilizer can represent this pose
  // bit-exactly.)
  // Returns false if this mobilizer doesn't implement this feature.
  // TODO(sherm1) Currently this is only implemented for 6dof mobilizers.
  //  It's still useful for other joints; broaden support.
  virtual bool SetPosePair(const systems::Context<T>& context,
                           const Eigen::Quaternion<T> q_FM,
                           const Vector3<T>& p_FM,
                           systems::State<T>* state) const = 0;

  // Given the generalized positions q for this Mobilizer in the given
  // `context`, returns the cross-mobilizer pose X_FM as a (quaternion, vec3)
  // pair. Most mobilizers can use the default implementation here, but
  // quaternion floating mobilizer is required to return bit-identical data
  // so must override.
  // @returns (q_FM, p_FM)
  virtual std::pair<Eigen::Quaternion<T>, Vector3<T>> GetPosePair(
      const systems::Context<T>& context) const;

  // Sets the velocity state v for this mobilizer in the given State to some
  // approximation of the given spatial velocity. It's up to the concrete
  // mobilizer to figure out what to do. 6-dof mobilizers can represent this
  // velocity exactly; others must project.
  // Note: there is no SetDefaultSpatialVelocity() because that is always zero.
  // Returns false if this mobilizer doesn't implement this feature.
  // TODO(sherm1) Currently this is only implemented for 6dof mobilizers.
  //  It's still useful for other joints; broaden support.
  virtual bool SetSpatialVelocity(const systems::Context<T>& context,
                                  const SpatialVelocity<T>& V_FM,
                                  systems::State<T>* state) const = 0;

  // Given the generalized positions q and generalized velocities v for this
  // Mobilizer in the given `context`, returns the cross-mobilizer spatial
  // velocity V_FM. (Not virtual)
  SpatialVelocity<T> GetSpatialVelocity(
      const systems::Context<T>& context) const {
    DRAKE_ASSERT(this->has_parent_tree());
    const Eigen::VectorBlock<const VectorX<T>> all_v =
        this->get_parent_tree().get_velocities(context);
    const Eigen::Ref<const VectorX<T>> my_v = get_velocities_from_array(all_v);
    return CalcAcrossMobilizerSpatialVelocity(context, my_v);
  }

  // Sets the `state` to the _default_ state (position and velocity) for
  // `this` mobilizer.  For example, the zero state for our standard IIWA
  // model has the arm pointing directly up; this is the correct definition of
  // the zero state (it is where our joint angles measure zero).  But we also
  // support a default state (perhaps a more comfortable initial configuration
  // of the IIWA), which need not be the zero state, that describes a state of
  // the Mobilizer to be used in e.g. MultibodyPlant::SetDefaultContext().
  virtual void set_default_state(const systems::Context<T>& context,
                                 systems::State<T>* state) const = 0;

  // Sets the `state` to a (potentially) random position and velocity, by
  // evaluating any random distributions that were declared (via e.g.
  // MobilizerImpl::set_random_position_distribution() and/or
  // MobilizerImpl::set_random_velocity_distribution(), or calling
  // set_default_state() if none have been declared. Note that the intended
  // caller of this method is `MultibodyTree::SetRandomState()` which treats
  // the independent samples returned from this sample as an initial guess,
  // but may change the value in order to "project" it onto a constraint
  // manifold.
  virtual void set_random_state(const systems::Context<T>& context,
                                systems::State<T>* state,
                                RandomGenerator* generator) const = 0;

  // Computes the across-mobilizer transform `X_FM(q)` between the inboard
  // frame F and the outboard frame M as a function of the vector of
  // generalized positions `q`.
  // %Mobilizer subclasses implementing this method can retrieve the fixed-size
  // vector of generalized positions for `this` mobilizer from `context` with:
  //
  // @code
  // auto q = this->get_positions(context);
  // @endcode
  //
  // Additionally, `context` can provide any other parameters the mobilizer
  // could depend on.

  // TODO(sherm1) Consider getting rid of this function altogether and
  //  making use only of the concrete mobilizer's calc_X_FM() method.
  virtual math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const = 0;

  // Computes the across-mobilizer spatial velocity `V_FM(q, v)` of the
  // outboard frame M in the inboard frame F.
  // This method can be thought of as the application of the operator `H_FM(q)`
  // to the input vector of generalized velocities `v`, i.e. the output of this
  // method is the application `v ∈ ℝⁿᵛ → M⁶: V_FM(q, v) = H_FM(q) * v`, where
  // `nv` is the number of generalized velocities of this mobilizer (see
  // num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  // aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  // velocities in Drake are not Plücker vectors as in Featherstone's book).
  // Therefore we say this method is the _operator form_ of the mobilizer hinge
  // matrix `H_FM(q)`.
  // This method aborts in Debug builds if the dimension of the input vector of
  // generalized velocities has a size different from num_velocities().
  //
  // @param[in] context The context of the parent tree that owns this
  // mobilizer. This mobilizer's generalized positions q are inferred from this
  // context.
  // @param[in] v A vector of generalized velocities. It must live in ℝⁿᵛ.
  // @retval V_FM The across-mobilizer spatial velocity of the outboard frame
  // M measured and expressed in the inboard frame F.
  virtual SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const = 0;

  // Computes the across-mobilizer spatial accelerations `A_FM(q, v, v̇)` of the
  // outboard frame M in the inboard frame F.
  // This method can be thought of as the application of the operation
  // `v̇ ∈ ℝⁿᵛ → M⁶: A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q) * v`, where
  // `nv` is the number of generalized velocities of this mobilizer (see
  // num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  // aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  // vectors in Drake are not Plücker vectors as in Featherstone's book).
  // Therefore, we say this method is in its _operator form_; the mobilizer
  // hinge matrix `H_FM(q)` is not explicitly formed.
  // This method aborts in Debug builds if the dimension of the input vector of
  // generalized accelerations has a size different from num_velocities().
  //
  // @param[in] context
  //   The context of the parent tree that owns this mobilizer. This
  //   mobilizer's generalized positions q and generalized velocities v are
  //   taken from this context.
  // @param[in] vdot
  //   The vector of generalized velocities' time derivatives v̇. It must live
  //   in ℝⁿᵛ.
  // @retval A_FM
  //   The across-mobilizer spatial acceleration of the outboard frame M
  //   measured and expressed in the inboard frame F.
  virtual SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const = 0;

  // Calculates a mobilizer's generalized forces `tau = H_FMᵀ(q) ⋅ F_Mo_F`,
  // where `H_FMᵀ` is the transpose of frame M's mobilizer hinge matrix and
  // `F_Mo_F` is the spatial force on frame M at Mo, expressed in F.
  // @see CalcAcrossMobilizerSpatialVelocity() and this class' documentation
  // for the definition of the mobilizer hinge matrix `H_FM`.
  //
  // This method can be thought of as the application of the transpose operator
  // `H_FMᵀ(q)` to the input spatial force `F_Mo_F`, i.e. the output of this
  // method is the application `F_Mo_F ∈ F⁶ → ℝⁿᵛ: tau = H_FMᵀ(q) * F_Mo_F`,
  // where `nv` is the number of generalized velocities of this mobilizer (see
  // num_velocities()) and F⁶ is the vector space of "force vectors" (be
  // aware that while F⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  // forces in Drake are not Plücker vectors as in Featherstone's book).
  // Therefore we say this method is the _operator form_ of the mobilizer
  // hinge matrix transpose `H_FMᵀ(q)`.
  // This method aborts in Debug builds if the dimension of the output vector
  // of generalized forces has a size different from num_velocities().
  //
  // @param[in] context
  //   The context of the parent tree that owns this mobilizer. This
  //   mobilizer's generalized positions q are stored in this context.
  // @param[in] F_Mo_F
  //   A SpatialForce applied at `this` mobilizer's outboard frame origin `Mo`,
  //   expressed in the inboard frame F.
  // @retval tau
  //   The vector of generalized forces. It must live in ℝⁿᵛ.
  virtual void ProjectSpatialForce(const systems::Context<T>& context,
                                   const SpatialForce<T>& F_Mo_F,
                                   Eigen::Ref<VectorX<T>> tau) const = 0;

  // Computes the kinematic mapping matrix `N(q)` that maps generalized
  // velocities for this mobilizer to time derivatives of the generalized
  // positions for this mobilizer according to `q̇ = N(q)⋅v`.
  // @param[in] context
  //   The context for the parent tree that owns this mobilizer storing the
  //   generalized positions q.
  // @param[out] N
  //   The kinematic mapping matrix `N(q)`. On input it must have size
  //   `nq x nv` with nq and nv the number of generalized positions and the
  //   number of generalized velocities for this mobilizer, respectively.
  // @see MapVelocityToQDot().
  void CalcNMatrix(const systems::Context<T>& context,
                   EigenPtr<MatrixX<T>> N) const {
    DRAKE_DEMAND(N != nullptr);
    DRAKE_DEMAND(N->rows() == num_positions());
    DRAKE_DEMAND(N->cols() == num_velocities());
    DoCalcNMatrix(context, N);
  }

  // Computes the kinematic mapping matrix `N⁺(q)` that maps time
  // derivatives of the generalized positions to generalized velocities
  // according to `v = N⁺(q)⋅q̇`. `N⁺(q)` is the left pseudoinverse of the
  // kinematic mapping `N(q)`, see CalcNMatrix().
  // @param[in] context
  //   The context for the parent tree that owns this mobilizer storing the
  //   generalized positions q.
  // @param[out] Nplus
  //   The kinematic mapping matrix `N⁺(q)`. On input it must have size
  //   `nv x nq` with nq the number of generalized positions and nv the
  //   number of generalized velocities.
  // @see MapVelocityToQDot().
  void CalcNplusMatrix(const systems::Context<T>& context,
                       EigenPtr<MatrixX<T>> Nplus) const {
    DRAKE_DEMAND(Nplus != nullptr);
    DRAKE_DEMAND(Nplus->rows() == num_velocities());
    DRAKE_DEMAND(Nplus->cols() == num_positions());
    DoCalcNplusMatrix(context, Nplus);
  }

  // Computes the matrix Ṅ(q,q̇) that helps relate q̈ (2ⁿᵈ time derivative of the
  // generalized positions) to v̇ (1ˢᵗ time derivative of generalized velocities)
  // via q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇, where N(q) is formed by CalcNMatrix().
  // @param[in] context stores generalized positions q and velocities v.
  // @param[out] Ndot The matrix Ṅ(q,q̇). On input Ndot must have size
  //   nq x nv where nq is the number of generalized positions and
  //   nv is the number of generalized velocities for this mobilizer.
  void CalcNDotMatrix(const systems::Context<T>& context,
                      EigenPtr<MatrixX<T>> Ndot) const {
    DRAKE_DEMAND(Ndot != nullptr);
    DRAKE_DEMAND(Ndot->rows() == num_positions());
    DRAKE_DEMAND(Ndot->cols() == num_velocities());
    DoCalcNDotMatrix(context, Ndot);
  }

  // Computes the matrix Ṅ⁺(q,q̇) that helps relate v̇ (1ˢᵗ time derivative of
  // generalized velocities) to q̈ (2ⁿᵈ time derivative of generalized positions)
  // via v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈, where N⁺(q) is formed by CalcNPlusMatrix().
  // @param[in] context stores generalized positions q and velocities v.
  // @param[out] NplusDot The matrix Ṅ(q,q̇). On input NplusDot must have size
  //   nv x nq where nv is the number of generalized velocities and
  //   nq is the number of generalized positions for this mobilizer.
  void CalcNplusDotMatrix(const systems::Context<T>& context,
                          EigenPtr<MatrixX<T>> NplusDot) const {
    DRAKE_DEMAND(NplusDot != nullptr);
    DRAKE_DEMAND(NplusDot->rows() == num_velocities());
    DRAKE_DEMAND(NplusDot->cols() == num_positions());
    DoCalcNplusDotMatrix(context, NplusDot);
  }

  virtual bool is_velocity_equal_to_qdot() const = 0;

  // Computes the kinematic mapping `q̇ = N(q)⋅v` between generalized
  // velocities v and time derivatives of the generalized positions `qdot`.
  // The generalized positions vector is stored in `context`.
  void MapVelocityToQDot(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& v,
                         EigenPtr<VectorX<T>> qdot) const {
    DRAKE_ASSERT(v.size() == num_velocities());
    DRAKE_ASSERT(qdot != nullptr);
    DRAKE_ASSERT(qdot->size() == num_positions());
    DoMapVelocityToQDot(context, v, qdot);
  }

  // Computes the mapping `v = N⁺(q)⋅q̇` from time derivatives of the
  // generalized positions `qdot` to generalized velocities v, where `N⁺(q)` is
  // the left pseudo-inverse of `N(q)` defined by MapVelocityToQDot().
  // The generalized positions vector is stored in `context`.
  void MapQDotToVelocity(const systems::Context<T>& context,
                         const Eigen::Ref<const VectorX<T>>& qdot,
                         EigenPtr<VectorX<T>> v) const {
    DRAKE_ASSERT(qdot.size() == num_positions());
    DRAKE_ASSERT(v != nullptr);
    DRAKE_ASSERT(v->size() == num_velocities());
    DoMapQDotToVelocity(context, qdot, v);
  }

  // Calculates q̈ from v̇, v, q using q̈ = N(q)⋅v̇ + Ṅ(q,v)⋅v.
  // @param[in] context stores generalized positions q and velocities v.
  // @param[in] vdot (v̇) 1ˢᵗ time derivatives of generalized velocities.
  // @param[out] qddot (q̈) 2ⁿᵈ time derivatives of the generalized positions.
  void MapAccelerationToQDDot(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& vdot,
                              EigenPtr<VectorX<T>> qddot) const {
    DRAKE_DEMAND(vdot.size() == num_velocities());
    DRAKE_DEMAND(qddot != nullptr);
    DRAKE_DEMAND(qddot->size() == num_positions());
    DoMapAccelerationToQDDot(context, vdot, qddot);
  }

  // Calculates v̇ from q̈, v, q using v̇ = N⁺(q)⋅q̈ + Ṅ⁺(q,v)⋅q̇ where q̇ = N(q)⋅v.
  // @param[in] context stores generalized positions q and velocities v.
  // @param[in] qddot (q̈) 2ⁿᵈ time derivatives of the generalized positions.
  // @param[out] vdot (v̇) 1ˢᵗ time derivatives of generalized velocities.
  void MapQDDotToAcceleration(const systems::Context<T>& context,
                              const Eigen::Ref<const VectorX<T>>& qddot,
                              EigenPtr<VectorX<T>> vdot) const {
    DRAKE_DEMAND(qddot.size() == num_positions());
    DRAKE_DEMAND(vdot != nullptr);
    DRAKE_DEMAND(vdot->size() == num_velocities());
    DoMapQDDotToAcceleration(context, qddot, vdot);
  }
  // @}

  // Returns a const Eigen expression of the vector of generalized positions
  // for `this` mobilizer from a vector `q_array` of generalized positions for
  // the entire MultibodyTree model.
  // @pre @p q_array is of size MultibodyTree::num_positions().
  Eigen::Ref<const VectorX<T>> get_positions_from_array(
      const Eigen::Ref<const VectorX<T>>& q_array) const {
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_DEMAND(q_array.size() == this->get_parent_tree().num_positions());
    return q_array.segment(position_start_in_q(), num_positions());
  }

  // Mutable version of get_positions_from_array().
  Eigen::Ref<VectorX<T>> get_mutable_positions_from_array(
      EigenPtr<VectorX<T>> q_array) const {
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_DEMAND(q_array != nullptr);
    DRAKE_DEMAND(q_array->size() == this->get_parent_tree().num_positions());
    return q_array->segment(position_start_in_q(), num_positions());
  }

  // Returns a const Eigen expression of the vector of generalized velocities
  // for `this` mobilizer from a vector `v_array` of generalized velocities for
  // the entire MultibodyTree model.
  // @pre @p v_array is of size MultibodyTree::num_velocities().
  Eigen::Ref<const VectorX<T>> get_velocities_from_array(
      const Eigen::Ref<const VectorX<T>>& v_array) const {
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_DEMAND(v_array.size() == this->get_parent_tree().num_velocities());
    return v_array.segment(velocity_start_in_v(), num_velocities());
  }

  // Mutable version of get_velocities_from_array().
  Eigen::Ref<VectorX<T>> get_mutable_velocities_from_array(
      EigenPtr<VectorX<T>> v_array) const {
    DRAKE_ASSERT(this->has_parent_tree());
    DRAKE_DEMAND(v_array != nullptr);
    DRAKE_DEMAND(v_array->size() == this->get_parent_tree().num_velocities());
    return v_array->segment(velocity_start_in_v(), num_velocities());
  }

  // Returns a const Eigen expression of the vector of generalized
  // accelerations for `this` mobilizer from a vector `vdot_array` of
  // generalized accelerations for the entire MultibodyTree model.
  // This method aborts if the input array is not of size
  // MultibodyTree::num_velocities().
  Eigen::Ref<const VectorX<T>> get_accelerations_from_array(
      const Eigen::Ref<const VectorX<T>>& vdot_array) const {
    return get_velocities_from_array(vdot_array);
  }

  // Mutable version of get_accelerations_from_array().
  Eigen::Ref<VectorX<T>> get_mutable_accelerations_from_array(
      EigenPtr<VectorX<T>> vdot_array) const {
    return get_mutable_velocities_from_array(vdot_array);
  }

  // Returns a const Eigen expression of the vector of generalized forces
  // for `this` mobilizer from a vector of generalized forces for the
  // entire MultibodyTree model.
  // This method aborts if the input array is not of size
  // MultibodyTree::num_velocities().
  Eigen::Ref<const VectorX<T>> get_generalized_forces_from_array(
      const Eigen::Ref<const VectorX<T>>& tau_array) const {
    return get_velocities_from_array(tau_array);
  }

  // Mutable version of get_generalized_forces_from_array().
  Eigen::Ref<VectorX<T>> get_mutable_generalized_forces_from_array(
      EigenPtr<VectorX<T>> tau_array) const {
    return get_mutable_velocities_from_array(tau_array);
  }

  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is mostly intended to be called by
  // MultibodyTree::CloneToScalar(). Most users should not call this clone
  // method directly but rather clone the entire parent MultibodyTree if
  // needed.
  // @sa MultibodyTree::CloneToScalar()
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> CloneToScalar(
      const MultibodyTree<ToScalar>& cloned_tree) const {
    return DoCloneToScalar(cloned_tree);
  }

  // For MultibodyTree internal use only.
  virtual std::unique_ptr<internal::BodyNode<T>> CreateBodyNode(
      const internal::BodyNode<T>* parent_node, const RigidBody<T>* body,
      const Mobilizer<T>* mobilizer) const = 0;

  // Lock the mobilizer. Its generalized velocities will be 0 until it is
  // unlocked.
  void Lock(systems::Context<T>* context) const {
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    // Joint locking is only supported for discrete mode.
    context->get_mutable_abstract_parameter(is_locked_parameter_index_)
        .set_value(true);
    this->get_parent_tree()
        .GetMutableVelocities(context)
        .segment(this->velocity_start_in_v(), this->num_velocities())
        .setZero();
  }

  // Unlock the mobilizer. Unlocking is not yet supported for continuous-mode
  // systems.
  void Unlock(systems::Context<T>* context) const {
    context->get_mutable_abstract_parameter(is_locked_parameter_index_)
        .set_value(false);
  }

  // @return true if the mobilizer is locked, false otherwise.
  bool is_locked(const systems::Context<T>& context) const {
    return context.get_parameters().template get_abstract_parameter<bool>(
        is_locked_parameter_index_);
  }

  void set_is_floating_base_mobilizer(bool is_floating) {
    is_floating_base_mobilizer_ = is_floating;
  }

 protected:
  // NVI to CalcNMatrix(). Implementations can safely assume that N is not the
  // nullptr and that N has the proper size.
  virtual void DoCalcNMatrix(const systems::Context<T>& context,
                             EigenPtr<MatrixX<T>> N) const = 0;

  // NVI to CalcNplusMatrix(). Implementations can safely assume that Nplus is
  // not the nullptr and that Nplus has the proper size.
  virtual void DoCalcNplusMatrix(const systems::Context<T>& context,
                                 EigenPtr<MatrixX<T>> Nplus) const = 0;

  // NVI to CalcNDotMatrix(). Implementations can safely assume that Ndot is
  // not the nullptr and that Ndot has the proper size.
  // TODO(Mitiguy) change this function to a pure virtual function when it has
  //  been overridden in all subclasses.
  virtual void DoCalcNDotMatrix(const systems::Context<T>& context,
                                EigenPtr<MatrixX<T>> Ndot) const;

  // NVI to CalcNplusDotMatrix(). Implementations can safely assume that
  // NplusDot is not the nullptr and that NplusDot has the proper size.
  // TODO(Mitiguy) change this function to a pure virtual function when it has
  //  been overridden in all subclasses.
  virtual void DoCalcNplusDotMatrix(const systems::Context<T>& context,
                                    EigenPtr<MatrixX<T>> NplusDot) const;

  // NVI to MapVelocityToQDot(). Implementations can safely assume that
  // v has size kNv, qdot is not the nullptr, and qdot has size kNq.
  virtual void DoMapVelocityToQDot(const systems::Context<T>& context,
                                   const Eigen::Ref<const VectorX<T>>& v,
                                   EigenPtr<VectorX<T>> qdot) const = 0;

  // NVI to MapQDotToVelocity(). Implementations can safely assume that
  // qdot has size kNq, v is not the nullptr, and v has size kNv
  virtual void DoMapQDotToVelocity(const systems::Context<T>& context,
                                   const Eigen::Ref<const VectorX<T>>& qdot,
                                   EigenPtr<VectorX<T>> v) const = 0;

  // NVI to MapAccelerationToQDDot(). Implementations can safely assume that
  // vdot has size kNv, qddot is not the nullptr, and qddot has size kNq.
  // TODO(Mitiguy) change this function to a pure virtual function when it has
  //  been overridden in all subclasses.
  virtual void DoMapAccelerationToQDDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot,
      EigenPtr<VectorX<T>> qddot) const;

  // NVI to MapQDDotToAcceleration(). Implementations can safely assume that
  // qddot has size kNq, vdot is not the nullptr, and vdot has size kNv.
  // TODO(Mitiguy) change this function to a pure virtual function when it has
  //  been overridden in all subclasses.
  virtual void DoMapQDDotToAcceleration(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qddot,
      EigenPtr<VectorX<T>> vdot) const;

  // @name Methods to make a clone templated on different scalar types.
  //
  // The only const argument to these methods is the new MultibodyTree clone
  // under construction, which is required to already own the clones of the
  // inboard and outboard frames of the mobilizer being cloned.
  // @{

  // Clones this %Mobilizer (templated on T) to a mobilizer templated on
  // `double`.
  // @pre Inboard and outboard frames for this mobilizer already have a clone
  // in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const = 0;

  // Clones this %Mobilizer (templated on T) to a mobilizer templated on
  // AutoDiffXd.
  // @pre Inboard and outboard frames for this mobilizer already have a clone
  // in `tree_clone`.
  virtual std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const = 0;

  virtual std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const = 0;
  // @}

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    is_locked_parameter_index_ =
        this->DeclareAbstractParameter(tree_system, Value<bool>(false));
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    // TODO(joemasterjohn): Consider exposing a default locked model value.
    parameters->template get_mutable_abstract_parameter<bool>(
        is_locked_parameter_index_) = false;
  }

 private:
  // Implementation for MultibodyElement::DoSetTopology().
  void DoSetTopology() final {
    // Mobod provides topology info at construction.
  }

  const SpanningForest::Mobod& mobod_;  // Topology information.
  const Frame<T>& inboard_frame_;
  const Frame<T>& outboard_frame_;

  // System parameter index for `this` mobilizer's lock state stored in a
  // context.
  systems::AbstractParameterIndex is_locked_parameter_index_;

  // Set according to the policy that defines a "floating base body". See
  // MultibodyTree::CreateJointImplementations() which enforces that policy.
  // (We define a floating base body as one for which we automatically added
  // a 6-dof joint to connect it directly to World; a floating base mobilizer
  // is the mobilizer implementing that joint.)
  bool is_floating_base_mobilizer_{false};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
