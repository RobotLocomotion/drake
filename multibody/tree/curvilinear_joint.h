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

/** A Joint which allows a child body to move relative to its parent along a
 curvilinear path composed of line segments and circular arcs within a plane.

 Given a frame F attached to the parent body P and a frame M attached to the
 child body B, this Joint allows frames F and M to have a relative transform
 X_FM defined by a PiecewiseConstantCurvatureTrajectory.

 The joint may be specified to be periodic, representing a path shaped as a
 closed loop. In this case, the path must return to the starting pose at its end
 distance s_f [m], and the transfrom X_FM(q) is then periodic with period
 s_f (X_FM(q) = X_FM(q + s_f)).

 This Joint has a single degree of freedom defined as the distance of travel
 along the path, with the transform X_FM defined as the path-aligned frame
 available through PiecewiseConstantCurvatureTrajectory::CalcPose.

 The path lies within a plane with normal axis p̂, equal to the z axis of frame
 M. p̂ is not necessarily equal to Fz, but p̂_F is constant.

 By default, the joint limits are the endpoints for aperiodic paths, and `(-∞,
 ∞)` for periodic paths.

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

   This constructor signature creates a joint with natural joint limits, i.e.
   the joint velocity and acceleration limits are the pair `(-∞, ∞)`; and the
   position limits are `(-∞, ∞)` if the path is a periodic loop, or its
   endpoints if not.

   The first three arguments to this constructor are those of the Joint class
   constructor. See the Joint class's documentation for details.
   The additional parameters are:
   @param[in] curvilinear_path The curvilinear path for this joint, along which
   the child frame M moves relative to the parent frame F.
   @param[in] damping Viscous damping coefficient, in N⋅s/m, used to model
   losses within the joint. The damping force (in N) is modeled as `f =
   -damping⋅v`, i.e. opposing motion, with v the tangential velocity for `this`
   joint (see get_tangential_velocity()).
   @throws std::exception if damping is negative */
  CurvilinearJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child,
      const trajectories::PiecewiseConstantCurvatureTrajectory<double>&
          trajectory,
      double damping = 0)
      : CurvilinearJoint<T>(
            name, frame_on_parent, frame_on_child, trajectory,
            trajectory.is_periodic() ? -std::numeric_limits<double>::infinity()
                                     : 0.,
            trajectory.is_periodic() ? std::numeric_limits<double>::infinity()
                                     : trajectory.length(),
            damping) {}

  /** Constructor to create a curvilinear joint between two bodies so that
   frame F attached to the parent body P and frame M attached to the child
   body B, move relatively to one another along a planar curvilinear path.
   See this class's documentation for further details on the definition of
   these frames and the path.
   The first three arguments to this constructor are those of the Joint class
   constructor. See the Joint class's documentation for details.
   The additional parameters are:
   @param[in] trajectory The curvilinear path for this joint, along which
   the child frame M moves relative to the parent frame F.
   @param[in] pos_lower_limit Lower position limit, in meters, for the distance
   coordinate (see get_distance()).
   @param[in] pos_upper_limit Upper position limit, in meters, for the distance
   coordinate (see get_distance()).
   @param[in] damping Viscous damping coefficient, in N⋅s/m, used to model
   losses within the joint. The damping force (in N) is modeled as `f =
   -damping⋅v`, i.e. opposing motion, with v the tangential velocity for `this`
   joint (see get_tangential_velocity()).
   @throws std::exception if damping is negative.
   @throws std::exception if pos_lower_limit > pos_upper_limit. */
  CurvilinearJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child,
      const trajectories::PiecewiseConstantCurvatureTrajectory<double>
          trajectory,
      double pos_lower_limit, double pos_upper_limit, double damping = 0);

  ~CurvilinearJoint() override;

  const std::string& type_name() const override;

  /** Returns `this` joint's default damping constant in N⋅s/m. */
  double default_damping() const { return this->default_damping_vector()[0]; }

  /** Sets the default value of viscous damping for this joint, in N⋅s/m.
   @throws std::exception if damping is negative.
   @pre the MultibodyPlant must not be finalized. */
  void set_default_damping(double damping) {
    DRAKE_THROW_UNLESS(damping >= 0);
    DRAKE_DEMAND(!this->get_parent_tree().topology_is_valid());
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
  /** Context-dependent joint coordinate value access.

   Gets the travel distance of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @returns The distance coordinate of the joint stored in the context. */
  const T& get_distance(const systems::Context<T>& context) const {
    return get_mobilizer().get_distance(context);
  }

  /** Context-dependent joint coordinate setter.

   Sets the travel distance of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @param[in] distance The travel distance to be set in the context.
   @returns const reference to this joint.
  */
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

  /** Context-dependent joint velocity value access.

   Gets the tangential velocity in meters per second, i.e. the rate of change of
   this joint's travel distance (see get_distance()) from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @returns The tangential velocity as stored in the provided context. */
  const T& get_tangential_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_tangential_velocity(context);
  }

  /** Context-dependent joint velocity setter.

   Sets the tangential velocity of the joint from the provided context.
   @param[in] context The context of the model this joint belongs to.
   @param[in] tangential_velocity The tangential velocity to be set in the
   context.
   @returns const reference to this joint. */
  const CurvilinearJoint<T>& set_tangential_velocity(
      systems::Context<T>* context, const T& tangential_velocity) const {
    get_mobilizer().SetTangentialVelocity(context, tangential_velocity);
    return *this;
  }

  /** Context-dependent damping coefficient getter.

   Refer to default_damping() for details.
   @param[in] context The context of the model this joint belongs to.
   @returns The damping coefficient stored in the context, in N⋅s/m. */
  const T& GetDamping(const systems::Context<T>& context) const {
    return this->GetDampingVector(context)[0];
  }

  /** Context-dependent damping coefficient setter.

   Refer to default_damping() for details.
   @param[out] context The context of the model this joint belongs to.
   @param[in] damping The damping coefficient to be set in N⋅s/m.
   @throws std::exception if `damping` is negative. */
  void SetDamping(systems::Context<T>* context, const T& damping) const {
    DRAKE_THROW_UNLESS(damping >= 0);
    this->SetDampingVector(context, Vector1<T>(damping));
  }

  /** Gets the default travel distance along the path.

   Wrapper for the more general Joint::default_positions.

   @returns The default distance along the path of `this` stored in
   `default_positions_` */
  double get_default_distance() const { return this->default_positions()[0]; }

  /** Sets the default travel distance of this joint.
   @param[in] distance The desired default distance of the joint in meters. */
  void set_default_distance(double distance) {
    this->set_default_positions(Vector1d{distance});
  }

  /** Adds into a MultibodyForces a generalized force on this joint.

   A generalized force for a curvilinear joint is equivalent to a force in
   Newtons applied on the path, in the path tangent direction (x-axis of the
   mobilized frame M).
   @param[in] context The context of the model this joint belongs to.
   @param[in] force The force to be applied, in Newtons.
   @param[out] forces The MultibodyForces object to which the generalized force
   is added.
  */
  void AddInForce(const systems::Context<T>& context, const T& force,
                  MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    this->AddInOneForce(context, 0, force, forces);
  }

 protected:
  /** Joint<T> override called through Joint::AddInForce.

   Arguments already checked to be valid by Joint::AddInForce.

   @param[in] joint_dof The joint degree of freedom index, on which the force is
   added, which must be 0.
   @param[in] joint_tau The force along the path's tangential axis to be added,
   in Newtons, applied to the child body.
   @param[out] forces The MultibodyForces object to which the force is added.
   @see The public NVI AddInOneForce() for details. */
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const override {
    DRAKE_DEMAND(joint_dof == 0);
    Eigen::Ref<VectorX<T>> tau_mob =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /** Joint<T> override called through Joint::AddInDamping.

   Arguments already checked to be valid by Joint::AddInDamping.

   Adds a dissipative force according to the viscous law `F = -d⋅v`, where
   d is the damping coefficient (see default_damping()) and v the tangential
   velocity along the path.

   @param[in] context The context of the model this joint belongs to.
   @param[out] forces The MultibodyForces object to which the damping force is
   added. */
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const override {
    const T damping_force =
        -this->GetDamping(context) * get_tangential_velocity(context);
    AddInForce(context, damping_force, forces);
  }

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const override { return 1; }

  int do_get_position_start() const override {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const override { return 1; }

  std::string do_get_position_suffix(int index) const override {
    return get_mobilizer().position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const override {
    return get_mobilizer().velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) override {
    if (this->has_implementation()) {
      get_mutable_mobilizer().set_default_position(default_positions);
    }
  }

  const T& DoGetOnePosition(const systems::Context<T>& context) const override {
    return get_distance(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const override {
    return get_tangential_velocity(context);
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint(
      const internal::SpanningForest::Mobod& mobod) const override {
    auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
    const auto [inboard_frame, outboard_frame] =
        this->tree_frames(mobod.is_reversed());
    // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
    auto curvilinear_mobilizer =
        std::make_unique<internal::CurvilinearMobilizer<T>>(
            mobod, *inboard_frame, *outboard_frame, curvilinear_path_);
    curvilinear_mobilizer->set_default_position(this->default_positions());
    blue_print->mobilizer = std::move(curvilinear_mobilizer);
    return blue_print;
  }

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  // Make CurvilinearJoint templated on every other scalar type a friend of
  // CurvilinearJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of CurvilinearJoint<T>.
  template <typename>
  friend class CurvilinearJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

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
