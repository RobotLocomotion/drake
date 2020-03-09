#pragma once

#include <memory>
#include <vector>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/random.h"
#include "drake/multibody/math/spatial_acceleration.h"
#include "drake/multibody/math/spatial_force.h"
#include "drake/multibody/math/spatial_velocity.h"
#include "drake/multibody/tree/frame.h"
#include "drake/multibody/tree/multibody_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

namespace internal {

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
///     pendulum.body_frame(),
///     X_BP /* pose of pin frame P in body frame B */);
/// // The mobilizer connects the world frame and the pin frame effectively
/// // adding the single degree of freedom describing this system. In this
/// // regard, the the role of a mobilizer is equivalent but conceptually
/// // different than a set of constraints that effectively remove all degrees
/// // of freedom but the one permitting rotation about the z-axis.
/// const RevoluteMobilizer<double>& revolute_mobilizer =
///   model.AddMobilizer<RevoluteMobilizer>(
///     model.world_frame(), /* inboard frame */
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
///
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
/// fully specified by, [Seth 2010]. The monogram notation used below for X_FM,
/// V_FM, F_Mo_F, etc., are described in @ref multibody_frames_and_bodies.
///
/// - X_FM(q):
///     The outboard frame M's pose as measured and expressed in the inboard
///     frame F, as a function of the mobilizer's generalized positions `q`.
///     This pose is computed by CalcAcrossMobilizerTransform().
/// - H_FM(q):
///     The `6 x nv` mobilizer hinge matrix `H_FM` relates `V_FM` (outboard
///     frame M's spatial velocity in its inboard frame F, expressed in F) to
///     the mobilizer's `nv` generalized velocities (or mobilities) `v` as
///     `V_FM = H_FM * v`.  The method CalcAcrossMobilizerSpatialVelocity()
///     calculates `V_FM`.  Be aware that Drake's spatial velocities are not the
///     Plücker vectors defined in [Featherstone 2008, Ch. 2].
///     Note: `H_FM` is only a function of the `nq` generalized positions `q`.
/// - H_FMᵀ(q):
///     H_FMᵀ is the `nv x 6` matrix transpose of `H_FM`.  It relates the `nv`
///     generalized forces `tau` to `F_Mo_F` (the spatial force on frame M at
///     point Mo, expressed in F) as `tau = H_FMᵀ ⋅ F_Mo_F`
///     The %Mobilizer method ProjectSpatialForce() calculates `tau`.
///     Be aware that Drake's spatial forces are not the Plücker vectors defined
///     in [Featherstone 2008, Ch. 2].
/// - Hdot_FM(q, v):
///     The time derivative of the mobilizer hinge matrix `H_FM` is used in the
///     calculation of `A_FM(q, v, v̇)` (outboard frame M's spatial acceleration
///     in its inboard frame F, expressed in F) as
///     `A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q, v) * v`.  The %Mobilizer method
///     CalcAcrossMobilizerSpatialAcceleration() calculates `A_FM`.
/// - N(q):
///     This `nq x nv` kinematic coupling matrix relates q̇ (the time-derivative
///     of the nq mobilizer's generalized positions) to `v` (the mobilizer's
///     generalized velocities) as `q̇ = N(q) * v`, [Seth 2010].
///     The %Mobilizer method MapVelocityToQDot() calculates `N(q)`.
/// - N⁺(q):
///     The left pseudo-inverse of `N(q)`. `N⁺(q)` can be used to invert the
///     relationship `q̇ = N(q) * v` without residual error, provided that `q̇` is
///     in the range space of `N(q)` (that is, if it *could* have been produced
///     as `q̇ = N(q) * v` for some `v`). The application `v = N⁺(q) * q̇` is
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
/// [Seth 2010]. The mobilizer "hinge" matrix `H_FM(q)` is introduced in
/// [Jain 2010], though be aware that what [Jain 2010] calls the hinge matrix is
/// the transpose of the mobilizer hinge matrix H_FM matrix here in Drake.
/// For details in the monogram notation used above please refer to
/// @ref multibody_spatial_algebra.
///
/// %Mobilizer is an abstract base class defining the minimum functionality that
/// derived %Mobilizer objects must implement in order to fully define the
/// kinematic relationship between the two frames they connect.
///
/// <h4>Relation between hinge matrix and Jacobians</h4>
///
/// The relationship between the across-mobilizer spatial velocity `V_FM` and
/// the time derivative of the across-mobilizer transform `X_FM` is similar to
/// the relationship between the rigid transform Jacobian Jq_X_VM (partial
/// derivatives of rigid transform X_FM with respect to generalized positions q)
/// and the Drake mobilizer hinge matrix `H_FM` (partial derivatives of
/// across-mobilizer q̇ with respect to generalized velocities v).
///
/// The translational velocity v_FM component of the spatial velocity `V_FM` is
/// defined as the time derivative of the position vector p_FM in `X_FM`. <pre>
///   v_FM = dp_FM/dt = ∂p_FM/∂q * q̇ = ∂p_FM/∂q * N(q) * v = Hv_FM * v
/// </pre>
/// where `Hv_FM = ∂p_FM/∂q * N(q)` is the last three rows in `H_FM`.
///
/// The angular velocity w_FM component of the spatial velocity `V_FM` can be
/// related to the time derivative of the rotation matrix R_FM in `X_FM`. This
/// complicated relationship can be written in terms of the skew symmetric
/// angular velocity matrix [w_FM] as <pre>
///  [w_FM] = d(R_FM)/dt * (R_FM)ᵀ
/// </pre>
/// The ordinary time-derivative of the rotation matrix R_FM is <pre>
///   d(R_FM)/dt = ∂R/∂q * q̇ = ∂R/∂q * N(q) * v
/// </pre>
/// Combining the previous two equations leads to <pre>
///  [w_FM] = ∂R/∂q * N(q) * v * (R_FM)ᵀ
/// </pre>
/// Post-multiplying both sides of the previous equation by R_FM gives <pre>
///  [w_FM] * R_FM = ∂R/∂q * N(q) * v
/// </pre>
/// `Hw_FM` is the first three rows in `H_FM`, defined by context as <pre>
///  Hw_FM * R_FM = ∂R/∂q * N(q)
/// </pre>
///
/// <h4>Active forces and power</h4>
///
/// The power generated by a mobilizer can be computed in two equivalent ways.
/// That is, the power can be computed in terms of the spatial force `F_Mo` and
/// the spatial velocity `V_FM` as: <pre>
///   P = F_Moᵀ * V_FM
/// </pre>
/// or in terms of the generalized forces `tau = H_FMᵀ(q) ⋅ F_Mo` and the
/// generalized velocities v as: <pre>
///   P = tauᵀ * v
/// </pre>
/// Notice that spatial forces in the null space of `H_FM(q)` do not perform any
/// work.  Since the result from the previous two expressions must be equal, the
/// mobilizer hinge matrix `H_FM(q)` and its transpose `H_FMᵀ(q)` are
/// constrained by: <pre>
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
/// @tparam_default_scalar
template <typename T>
class Mobilizer : public MultibodyElement<Mobilizer, T, MobilizerIndex> {
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
  /// `num_positions() == 1` since RevoluteMobilizer adds a single
  /// generalized coordinate representing the rotational degree of freedom about
  /// a given axis between the inboard and outboard frames. Another example
  /// would be a 6 DOF "free" mobilizer internally using a quaternion
  /// representation to parameterize free rotations and a position vector to
  /// parameterize free translations; this method would return 7 (a quaternion
  /// plus a position vector).
  /// @see num_velocities()
  virtual int num_positions() const = 0;

  /// Returns the number of generalized velocities granted by this mobilizer.
  /// Given that all physics occurs in the generalized velocities space, the
  /// number of generalized velocities exactly matches the number of degrees of
  /// freedom granted by the mobilizer.
  /// As an example, consider RevoluteMobilizer, for which
  /// `num_velocities() == 1` since for RevoluteMobilizer its one and only
  /// generalized velocity describes the magnitude of the angular velocity about
  /// a given axis between the inboard and outboard frames.
  /// @see num_positions()
  virtual int num_velocities() const = 0;

  /// Returns the index to the first generalized position for this mobilizer
  /// within the vector q of generalized positions for the full multibody
  /// system.
  int position_start_in_q() const {
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return topology_.positions_start;
  }

  /// Returns the index to the first generalized velocity for this mobilizer
  /// within the vector v of generalized velocities for the full multibody
  /// system.
  int velocity_start_in_v() const {
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return topology_.velocities_start_in_v;
  }

  /// Returns a constant reference to the inboard frame.
  const Frame<T>& inboard_frame() const {
    return inboard_frame_;
  }

  /// Returns a constant reference to the outboard frame.
  const Frame<T>& outboard_frame() const {
    return outboard_frame_;
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's inboard frame.
  const Body<T>& inboard_body() const {
    return inboard_frame().body();
  }

  /// Returns a constant reference to the body associated with `this`
  /// mobilizer's outboard frame.
  const Body<T>& outboard_body() const {
    return outboard_frame().body();
  }

  /// Returns `true` if `this` mobilizer grants 6-dofs to the outboard frame.
  virtual bool is_floating() const { return false; }

  /// Returns `true` if `this` uses a quaternion parametrization of rotations.
  virtual bool has_quaternion_dofs() const { return false; }

  /// Returns the topology information for this mobilizer. Users should not
  /// need to call this method since MobilizerTopology is an internal
  /// bookkeeping detail.
  const MobilizerTopology& get_topology() const { return topology_; }

  /// @name Methods that define a %Mobilizer
  /// @{

  /// Sets the `state` to what will be considered to be the _zero_ state
  /// (position and velocity) for `this` mobilizer. For most mobilizers the
  /// _zero_ position corresponds to the value of generalized positions at
  /// which the inboard frame F and the outboard frame coincide or, in other
  /// words, when `X_FM = Id` is the identity pose. In the general case
  /// however, the zero position will correspond to a value of the
  /// generalized positions for which `X_FM = X_FM_ref` where `X_FM_ref` may
  /// generally be different from the identity transformation.
  /// In other words, `X_FM_ref = CalcAcrossMobilizerTransform(ref_context)`
  /// where `ref_context` is a Context storing a State set to the zero
  /// configuration with set_zero_state().
  /// In addition, all generalized velocities are set to zero in the _zero_
  /// state.
  ///
  /// Most often the _zero_ position will correspond to setting
  /// the vector of generalized positions related to this mobilizer to zero.
  /// However, in the general case, setting all generalized coordinates to zero
  /// does not correspond to the _zero_ position and it might even not
  /// represent a mathematicaly valid one. Consider for instance a quaternion
  /// mobilizer, for which its _zero_ position corresponds to the quaternion
  /// [1, 0, 0, 0].
  ///
  /// Note that the zero state may fall outside of the limits for any joints
  /// associated with this mobilizer.
  /// @see set_default_state().
  virtual void set_zero_state(const systems::Context<T>& context,
                              systems::State<T>* state) const = 0;

  /// Sets the `state` to the _default_ state (position and velocity) for
  /// `this` mobilizer.  For example, the zero state for our standard IIWA
  /// model has the arm pointing directly up; this is the correct definition of
  /// the zero state (it is where our joint angles measure zero).  But we also
  /// support a default state (perhaps a more comfortable initial configuration
  /// of the IIWA), which need not be the zero state, that describes a state of
  /// the Mobilizer to be used in e.g. MultibodyPlant::SetDefaultContext().
  virtual void set_default_state(const systems::Context<T>& context,
                                 systems::State<T>* state) const = 0;

  /// Sets the `state` to a (potentially) random position and velocity, by
  /// evaluating any random distributions that were declared (via e.g.
  /// MobilizerImpl::set_random_position_distribution() and/or
  /// MobilizerImpl::set_random_velocity_distribution(), or calling
  /// set_zero_state() if none have been declared. Note that the intended
  /// caller of this method is `MultibodyTree::SetRandomState()` which treats
  /// the independent samples returned from this sample as an initial guess,
  /// but may change the value in order to "project" it onto a constraint
  /// manifold.
  virtual void set_random_state(const systems::Context<T>& context,
                                systems::State<T>* state,
                                RandomGenerator* generator) const = 0;

  /// Computes the across-mobilizer transform `X_FM(q)` between the inboard
  /// frame F and the outboard frame M as a function of the vector of
  /// generalized positions `q`.
  /// %Mobilizer subclasses implementing this method can retrieve the fixed-size
  /// vector of generalized positions for `this` mobilizer from `context` with:
  ///
  /// @code
  /// auto q = this->get_positions(context);
  /// @endcode
  ///
  /// Additionally, `context` can provide any other parameters the mobilizer
  /// could depend on.
  virtual math::RigidTransform<T> CalcAcrossMobilizerTransform(
      const systems::Context<T>& context) const = 0;

  /// Computes the across-mobilizer spatial velocity `V_FM(q, v)` of the
  /// outboard frame M in the inboard frame F.
  /// This method can be thought of as the application of the operator `H_FM(q)`
  /// to the input vector of generalized velocities `v`, i.e. the output of this
  /// method is the application `v ∈ ℝⁿᵛ → M⁶: V_FM(q, v) = H_FM(q) * v`, where
  /// `nv` is the number of generalized velocities of this mobilizer (see
  /// num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  /// aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// velocities in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore we say this method is the _operator form_ of the mobilizer hinge
  /// matrix `H_FM(q)`.
  /// This method aborts in Debug builds if the dimension of the input vector of
  /// generalized velocities has a size different from num_velocities().
  ///
  /// @param[in] context The context of the parent tree that owns this
  /// mobilizer. This mobilizer's generalized positions q are inferred from this
  /// context.
  /// @param[in] v A vector of generalized velocities. It must live in ℝⁿᵛ.
  /// @retval V_FM The across-mobilizer spatial velocity of the outboard frame
  /// M measured and expressed in the inboard frame F.
  virtual SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const = 0;

  /// Computes the across-mobilizer spatial accelerations `A_FM(q, v, v̇)` of the
  /// outboard frame M in the inboard frame F.
  /// This method can be thought of as the application of the operation
  /// `v̇ ∈ ℝⁿᵛ → M⁶: A_FM(q, v, v̇) = H_FM(q) * v̇ + Ḣ_FM(q) * v`, where
  /// `nv` is the number of generalized velocities of this mobilizer (see
  /// num_velocities()) and M⁶ is the vector space of "motion vectors" (be
  /// aware that while M⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// vectors in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore, we say this method is in its _operator form_; the mobilizer
  /// hinge matrix `H_FM(q)` is not explicitly formed.
  /// This method aborts in Debug builds if the dimension of the input vector of
  /// generalized accelerations has a size different from num_velocities().
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
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const = 0;

  /// Calculates a mobilizer's generalized forces `tau = H_FMᵀ(q) ⋅ F_Mo_F`,
  /// where `H_FMᵀ` is the transpose of frame M's mobilizer hinge matrix and
  /// `F_Mo_F` is the spatial force on frame M at Mo, expressed in F.
  /// @see CalcAcrossMobilizerSpatialVelocity() and this class' documentation
  /// for the definition of the mobilizer hinge matrix `H_FM`.
  ///
  /// This method can be thought of as the application of the transpose operator
  /// `H_FMᵀ(q)` to the input spatial force `F_Mo_F`, i.e. the output of this
  /// method is the application `F_Mo_F ∈ F⁶ → ℝⁿᵛ: tau = H_FMᵀ(q) * F_Mo_F`,
  /// where `nv` is the number of generalized velocities of this mobilizer (see
  /// num_velocities()) and F⁶ is the vector space of "force vectors" (be
  /// aware that while F⁶ is introduced in [Featherstone 2008, Ch. 2] spatial
  /// forces in Drake are not Plücker vectors as in Featherstone's book).
  /// Therefore we say this method is the _operator form_ of the mobilizer
  /// hinge matrix transpose `H_FMᵀ(q)`.
  /// This method aborts in Debug builds if the dimension of the output vector
  /// of generalized forces has a size different from num_velocities().
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
      const systems::Context<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const = 0;

  /// Computes the kinematic mapping matrix `N(q)` that maps generalized
  /// velocities for this mobilizer to time derivatives of the generalized
  /// positions for this mobilizer according to `q̇ = N(q)⋅v`.
  /// @param[in] context
  ///   The context for the parent tree that owns this mobilizer storing the
  ///   generalized positions q.
  /// @param[out] N
  ///   The kinematic mapping matrix `N(q)`. On input it must have size
  ///   `nq x nv` with nq and nv the number of generalized positions and the
  ///   number of generalized velocities for this mobilizer, respectively.
  /// @see MapVelocityToQDot().
  void CalcNMatrix(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> N) const {
    DRAKE_DEMAND(N != nullptr);
    DRAKE_DEMAND(N->rows() == num_positions());
    DRAKE_DEMAND(N->cols() == num_velocities());
    DoCalcNMatrix(context, N);
  }

  /// Computes the kinematic mapping matrix `N⁺(q)` that maps time
  /// derivatives of the generalized positions to generalized velocities
  /// according to `v = N⁺(q)⋅q̇`. `N⁺(q)` is the left pseudoinverse of the
  /// kinematic mapping `N(q)`, see CalcNMatrix().
  /// @param[in] context
  ///   The context for the parent tree that owns this mobilizer storing the
  ///   generalized positions q.
  /// @param[out] Nplus
  ///   The kinematic mapping matrix `N⁺(q)`. On input it must have size
  ///   `nv x nq` with nq the number of generalized positions and nv the
  ///   number of generalized velocities.
  /// @see MapVelocityToQDot().
  void CalcNplusMatrix(
      const systems::Context<T>& context,
      EigenPtr<MatrixX<T>> Nplus) const {
    DRAKE_DEMAND(Nplus != nullptr);
    DRAKE_DEMAND(Nplus->rows() == num_velocities());
    DRAKE_DEMAND(Nplus->cols() == num_positions());
    DoCalcNplusMatrix(context, Nplus);
  }

  /// Computes the kinematic mapping `q̇ = N(q)⋅v` between generalized
  /// velocities v and time derivatives of the generalized positions `qdot`.
  /// The generalized positions vector is stored in `context`.
  virtual void MapVelocityToQDot(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const = 0;

  /// Computes the mapping `v = N⁺(q)⋅q̇` from time derivatives of the
  /// generalized positions `qdot` to generalized velocities v, where `N⁺(q)` is
  /// the left pseudo-inverse of `N(q)` defined by MapVelocityToQDot().
  /// The generalized positions vector is stored in `context`.
  virtual void MapQDotToVelocity(
      const systems::Context<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const = 0;
  /// @}

  /// Returns a const Eigen expression of the vector of generalized positions
  /// for `this` mobilizer from a vector `q_array` of generalized positions for
  /// the entire MultibodyTree model.
  /// @pre @p q_array is of size MultibodyTree::num_positions().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_positions_from_array(const Eigen::Ref<const VectorX<T>>& q_array) const {
    DRAKE_DEMAND(
        q_array.size() == this->get_parent_tree().num_positions());
    return q_array.segment(topology_.positions_start,
                           topology_.num_positions);
  }

  /// Mutable version of get_positions_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> get_mutable_positions_from_array(
      EigenPtr<VectorX<T>> q_array) const {
    DRAKE_DEMAND(q_array != nullptr);
    DRAKE_DEMAND(
        q_array->size() == this->get_parent_tree().num_positions());
    return q_array->segment(topology_.positions_start,
                            topology_.num_positions);
  }

  /// Returns a const Eigen expression of the vector of generalized velocities
  /// for `this` mobilizer from a vector `v_array` of generalized velocities for
  /// the entire MultibodyTree model.
  /// @pre @p v_array is of size MultibodyTree::num_velocities().
  Eigen::VectorBlock<const Eigen::Ref<const VectorX<T>>>
  get_velocities_from_array(const Eigen::Ref<const VectorX<T>>& v_array) const {
    DRAKE_DEMAND(
        v_array.size() == this->get_parent_tree().num_velocities());
    return v_array.segment(topology_.velocities_start_in_v,
                           topology_.num_velocities);
  }

  /// Mutable version of get_velocities_from_array().
  Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> get_mutable_velocities_from_array(
      EigenPtr<VectorX<T>> v_array) const {
    DRAKE_DEMAND(v_array != nullptr);
    DRAKE_DEMAND(
        v_array->size() == this->get_parent_tree().num_velocities());
    return v_array->segment(topology_.velocities_start_in_v,
                            topology_.num_velocities);
  }

  /// Returns a const Eigen expression of the vector of generalized
  /// accelerations for `this` mobilizer from a vector `vdot_array` of
  /// generalized accelerations for the entire MultibodyTree model.
  /// This method aborts if the input array is not of size
  /// MultibodyTree::num_velocities().
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
  /// MultibodyTree::num_velocities().
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
  /// NVI to CalcNMatrix(). Implementations can safely assume that N is not the
  /// nullptr and that N has the proper size.
  virtual void DoCalcNMatrix(
      const systems::Context<T>& context, EigenPtr<MatrixX<T>> N) const = 0;

  /// NVI to CalcNplusMatrix(). Implementations can safely assume that Nplus is
  /// not the nullptr and that Nplus has the proper size.
  virtual void DoCalcNplusMatrix(
      const systems::Context<T>& context,
      EigenPtr<MatrixX<T>> Nplus) const = 0;

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

  virtual std::unique_ptr<Mobilizer<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const = 0;
  /// @}

 private:
  // Implementation for MultibodyElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each mobilizer retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_mobilizer(this->index());
  }

  const Frame<T>& inboard_frame_;
  const Frame<T>& outboard_frame_;
  MobilizerTopology topology_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
