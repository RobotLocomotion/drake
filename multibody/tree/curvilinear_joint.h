#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
#include "drake/multibody/tree/curvilinear_mobilizer.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"

namespace drake {
namespace multibody {

/** A Joint that allows a body to move along a piecewise constant curvature path
 contained in a plane.

 The path is specified as a PiecewiseConstantCurvatureTrajectory, refer to that
 class documentation for further details on parameterization, conventions and
 notation used.

 This joint grants a single degree of freedom q that corresponds to the length s
 (in meters) along the path. The generalized velocity v = q̇ corresponds to the
 magnitude of the tangential velocity. We denote with F a frame on a "parent"
 body and with M a frame on a "child" body. For a given trajectory, this joint
 prescribes X_FM(q) = PiecewiseConstantCurvatureTrajectory::CalcPose(q).

 Frame M is defined according to the convention documented in
 PiecewiseConstantCurvatureTrajectory. That is, axis Mx is the tangent to the
 trajectory, Mz equals the (constant) normal p̂ to the plane, and My = Mz x Mx.
 It is not required that M coincides with F at distance q = 0.

 If the specified trajectory is periodic, the joint prescribes a trajectory
 of cycle length L that satisfies X_FP(s) = X_FP(s + k⋅L) ∀ k ∈ ℤ.

 By default, the joint position limits are the endpoints for aperiodic paths,
 and (-∞, ∞) for periodic paths.

 @see trajectories::PiecewiseConstantCurvatureTrajectory

 @tparam_default_scalar */
template <typename T>
class CurvilinearJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CurvilinearJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /** Constructor to create a curvilinear joint between two bodies so that
   frame F attached to the parent body P and frame M attached to the child
   body B, move relatively to one another along a planar curvilinear path.
   See this class's documentation for further details on the definition of
   these frames and the curvilinear path.

   This constructor signature creates a joint where the joint velocity and
   acceleration limits are `(-∞, ∞)`. Position limits are `(0, L)` with L the
   length of the trajectory. If the trajectory is periodic, position limits are
   `(-∞, ∞)`.

   The first three arguments to this constructor are those of the Joint class
   constructor. See the Joint class's documentation for details. The additional
   parameters are:

   @param[in] curvilinear_path The curvilinear path for this joint, along which
   the child frame M moves relative to the parent frame F.
   @param[in] damping Viscous damping coefficient, in N⋅s/m, used to model
   losses within the joint. The damping force (in N) is modeled as `f =
   -damping⋅v` along the tangent to the curve, i.e. opposing motion, with v the
   tangential velocity for `this` joint (see get_tangential_velocity()).

   @throws std::exception if damping is negative */
  CurvilinearJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child,
      const trajectories::PiecewiseConstantCurvatureTrajectory<double>&
          curvilinear_path,
      double damping = 0)
      : CurvilinearJoint<T>(name, frame_on_parent, frame_on_child,
                            curvilinear_path,
                            curvilinear_path.is_periodic()
                                ? -std::numeric_limits<double>::infinity()
                                : 0.,
                            curvilinear_path.is_periodic()
                                ? std::numeric_limits<double>::infinity()
                                : curvilinear_path.length(),
                            damping) {}

  /** Constructor to create a curvilinear joint between two bodies so that
   frame F attached to the parent body P and frame M attached to the child
   body B, move relatively to one another along a planar curvilinear path.
   See this class's documentation for further details on the definition of
   these frames and the path.

   The first three arguments to this constructor are those of the Joint class
   constructor. See the Joint class's documentation for details. The additional
   parameters are:

   @param[in] curvilinear_path The curvilinear path for this joint, along which
   the child frame M moves relative to the parent frame F.
   @param[in] pos_lower_limit Lower position limit, in meters, for the distance
   coordinate (see get_distance()).
   @param[in] pos_upper_limit Upper position limit, in meters, for the distance
   coordinate (see get_distance()).
   @param[in] damping Viscous damping coefficient, in N⋅s/m, used to model
   losses within the joint. The damping force (in N) is modeled as `f =
   -damping⋅v` along the tangent to the curve, i.e. opposing motion, with v the
   tangential velocity for `this` joint (see get_tangential_velocity()).

   @throws std::exception if damping is negative.
   @throws std::exception if pos_lower_limit > pos_upper_limit. */
  CurvilinearJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child,
      const trajectories::PiecewiseConstantCurvatureTrajectory<double>
          curvilinear_path,
      double pos_lower_limit, double pos_upper_limit, double damping = 0);

  ~CurvilinearJoint() final;

  const std::string& type_name() const final;

  /** Returns `this` joint's default damping constant in N⋅s/m. */
  double default_damping() const { return this->default_damping_vector()[0]; }

  /** Sets the default value of viscous damping for this joint, in N⋅s/m.
   @throws std::exception if damping is negative.
   @throws std::exception if this element is not associated with a
   MultibodyPlant.
   @pre the MultibodyPlant must not be finalized. */
  void set_default_damping(double damping) {
    DRAKE_THROW_UNLESS(damping >= 0);
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    DRAKE_DEMAND(!this->get_parent_tree().is_finalized());
    this->set_default_damping_vector(Vector1d(damping));
  }

  /** Returns the position lower limit for `this` joint in m. */
  double position_lower_limit() const {
    return this->position_lower_limits()[0];
  }

  /** Returns the position upper limit for `this` joint in m. */
  double position_upper_limit() const {
    return this->position_upper_limits()[0];
  }

  /** Returns the velocity lower limit for `this` joint in m/s. */
  double velocity_lower_limit() const {
    return this->velocity_lower_limits()[0];
  }

  /** Returns the velocity upper limit for `this` joint in m/s. */
  double velocity_upper_limit() const {
    return this->velocity_upper_limits()[0];
  }

  /** Returns the acceleration lower limit for `this` joint in m/s². */
  double acceleration_lower_limit() const {
    return this->acceleration_lower_limits()[0];
  }

  /** Returns the acceleration upper limit for `this` joint in m/s². */
  double acceleration_upper_limit() const {
    return this->acceleration_upper_limits()[0];
  }

  /** Gets the travel distance of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @returns The distance coordinate of the joint stored in the context. */
  const T& get_distance(const systems::Context<T>& context) const {
    return get_mobilizer().get_distance(context);
  }

  /** Sets the travel distance of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @param[in] distance The travel distance to be set in the context.
   @returns const reference to this joint. */
  const CurvilinearJoint<T>& set_distance(systems::Context<T>* context,
                                          const T& distance) const {
    get_mobilizer().SetDistance(context, distance);
    return *this;
  }

  /** Sets the random distribution for the distance along the path.
   @param[in] distance Expression defining the random distance distribution. */
  void set_random_distance_distribution(const symbolic::Expression& distance) {
    get_mutable_mobilizer().set_random_position_distribution(
        Vector1<symbolic::Expression>{distance});
  }

  /** Gets the tangential velocity in meters per second, i.e. the rate of change
   of this joint's travel distance (see get_distance()) from the provided
   context.
   @param[in] context The context of the model this joint belongs to.
   @returns The tangential velocity as stored in the provided context. */
  const T& get_tangential_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_tangential_velocity(context);
  }

  /** Sets the tangential velocity of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @param[in] tangential_velocity The tangential velocity to be set in the
   context.
   @returns const reference to this joint. */
  const CurvilinearJoint<T>& set_tangential_velocity(
      systems::Context<T>* context, const T& tangential_velocity) const {
    get_mobilizer().SetTangentialVelocity(context, tangential_velocity);
    return *this;
  }

  /** Returns the Context dependent damping coefficient stored as a parameter in
  `context`. Refer to default_damping() for details.
   @param[in] context The context of the model this joint belongs to.
   @returns The damping coefficient stored in the context, in N⋅s/m. */
  const T& GetDamping(const systems::Context<T>& context) const {
    return this->GetDampingVector(context)[0];
  }

  /**  Sets the value of the viscous damping coefficient for this joint, stored
   as a parameter in `context`. Refer to default_damping() for details.
   @param[out] context The context of the model this joint belongs to.
   @param[in] damping The damping coefficient to be set in N⋅s/m.
   @throws std::exception if `damping` is negative. */
  void SetDamping(systems::Context<T>* context, const T& damping) const {
    DRAKE_THROW_UNLESS(damping >= 0);
    this->SetDampingVector(context, Vector1<T>(damping));
  }

  /** Gets the default travel distance along the path. */
  double get_default_distance() const { return this->default_positions()[0]; }

  /** Sets the default travel distance of this joint.
   @param[in] distance The desired default distance of the joint in meters. */
  void set_default_distance(double distance) {
    this->set_default_positions(Vector1d{distance});
  }

  /** Adds into a MultibodyForces a generalized force on this joint.

   A generalized force for a curvilinear joint is equivalent to a force in
   Newtons applied along the path tangent direction (x-axis of frame M).
   @param[in] context The context of the model this joint belongs to.
   @param[in] force The force to be applied, in Newtons.
   @param[out] forces The MultibodyForces object to which the generalized force
   is added. */
  void AddInForce(const systems::Context<T>& context, const T& force,
                  MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(this->has_parent_tree());
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    this->AddInOneForce(context, 0, force, forces);
  }

  /** @returns A reference to the underlying trajectory. */
  const trajectories::PiecewiseConstantCurvatureTrajectory<double>&
  get_trajectory() const {
    return curvilinear_path_;
  }

 protected:
  /** Joint<T> override called through public NVI, Joint::AddInForce().
   Arguments already checked to be valid by Joint::AddInForce().
   @param[in] joint_dof The joint degree of freedom index, on which the force is
   added, which must be 0.
   @param[in] joint_tau The force along the path's tangential axis to be added,
   in Newtons, applied to the child body.
   @param[out] forces The MultibodyForces object to which the force is added.
   @see The public NVI AddInOneForce() for details. */
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const final {
    DRAKE_DEMAND(joint_dof == 0);
    Eigen::Ref<VectorX<T>> tau_mob =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /** Joint<T> override called through public NVI, Joint::AddInDamping().
   Arguments already checked to be valid by Joint::AddInDamping().

   Adds a dissipative force according to the viscous law `f = -d⋅v`, where
   d is the damping coefficient (see default_damping()) and v the tangential
   velocity along the path.

   @param[in] context The context of the model this joint belongs to.
   @param[out] forces The MultibodyForces object to which the damping force is
   added. */
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    const T damping_force =
        -this->GetDamping(context) * get_tangential_velocity(context);
    AddInForce(context, damping_force, forces);
  }

 private:
  int do_get_velocity_start() const final {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 1; }

  int do_get_position_start() const final {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const final { return 1; }

  std::string do_get_position_suffix(int index) const final {
    return get_mobilizer().position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const final {
    return get_mobilizer().velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) final {
    if (this->has_mobilizer()) {
      get_mutable_mobilizer().set_default_position(default_positions);
    }
  }

  const T& DoGetOnePosition(const systems::Context<T>& context) const final {
    return get_distance(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const final {
    return get_tangential_velocity(context);
  }

  // Joint<T> overrides:
  std::unique_ptr<internal::Mobilizer<T>> MakeMobilizerForJoint(
      const internal::SpanningForest::Mobod& mobod,
      internal::MultibodyTree<T>* tree) const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const final;

  std::unique_ptr<Joint<T>> DoShallowClone() const final;

  // Make CurvilinearJoint templated on every other scalar type a friend of
  // CurvilinearJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of CurvilinearJoint<T>.
  template <typename>
  friend class CurvilinearJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::CurvilinearMobilizer<T>& get_mobilizer() const {
    return this
        ->template get_mobilizer_downcast<internal::CurvilinearMobilizer>();
  }

  internal::CurvilinearMobilizer<T>& get_mutable_mobilizer() {
    return this->template get_mutable_mobilizer_downcast<
        internal::CurvilinearMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  trajectories::PiecewiseConstantCurvatureTrajectory<double> curvilinear_path_;
};

template <typename T>
const char CurvilinearJoint<T>::kTypeName[] = "curvilinear";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::CurvilinearJoint);
