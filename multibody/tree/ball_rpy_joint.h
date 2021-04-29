#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to rotate freely relative to one another.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frame M to rotate freely with respect to F, while the
/// origins, Mo and Fo, of frames M and F respectively remain coincident. The
/// orientation of M relative to F is parameterized with space `x-y-z` Euler
/// angles.
///
/// @tparam_default_scalar
template <typename T>
class BallRpyJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BallRpyJoint)

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.  It resolves to "ball_rpy".
  static const char kTypeName[];

  /// Constructor to create a ball rpy joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// rotate freely relative to one another. See this class's documentation
  /// for further details on the definition of these frames, get_angles() for an
  /// explanation of the angles defining orientation, and get_angular_velocity()
  /// for an explanation of the generalized velocities.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits().
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. See documentation of damping() for details on modelling of the
  ///   damping torque.
  /// @throws std::exception if damping is negative.
  BallRpyJoint(const std::string& name, const Frame<T>& frame_on_parent,
               const Frame<T>& frame_on_child, double damping = 0)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(damping >= 0);
    damping_ = damping;
  }

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's damping constant in N⋅m⋅s. The damping torque
  /// (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e. opposing motion, with ω the
  /// angular velocity of frame M in F (see get_angular_velocity()) and τ the
  /// torque on child body B (to which M is rigidly attached).
  double damping() const { return damping_; }

  /// @name Context-dependent value access
  /// @{

  /// Gets the rotation angles of `this` joint from `context`.
  ///
  /// The orientation `R_FM` of the child frame M in parent frame F is
  /// parameterized with space `x-y-z` Euler angles (also known as extrinsic
  /// angles). That is, the angles θr, θp, θy, correspond to a sequence of
  /// rotations about the x̂, ŷ, ẑ axes of parent frame F, respectively.
  /// Mathematically, rotation `R_FM` is given in terms of angles θr, θp, θy
  /// by: <pre>
  ///   R_FM(q) = Rz(θy) * Ry(θp) * Rx(θr)
  /// </pre>
  /// where `Rx(θ)`, `Ry(θ)` and `Rz(θ)` correspond to the elemental rotations
  /// in amount of θ about the x, y and z axes respectively.
  /// Zero θr, θp, θy angles corresponds to frames F and M being coincident.
  /// Angles θr, θp, θy are defined to be positive according to the
  /// right-hand-rule with the thumb aligned in the direction of their
  /// respective axes.
  ///
  /// @note Space `x-y-z` angles (extrinsic) are equivalent to Body `z-y-x`
  /// angles (intrinsic).
  ///
  /// @note This particular choice of angles θr, θp, θy for this joint are
  /// many times referred to as the roll, pitch and yaw angles by many
  /// dynamicists. They are also known as the Tait-Bryan angles or Cardan
  /// angles.
  ///
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @returns The angle coordinates of `this` joint stored in the `context`
  ///          ordered as θr, θp, θy.
  Vector3<T> get_angles(const Context<T>& context) const {
    return get_mobilizer()->get_angles(context);
  }

  /// Sets the `context` so that the generalized coordinates corresponding to
  /// the rotation angles of `this` joint equals `angles`.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @param[in] angles
  ///   The desired angles in radians to be stored in `context` ordered as θr,
  ///   θp, θy. See get_angles() for details.
  /// @returns a constant reference to `this` joint.
  const BallRpyJoint<T>& set_angles(Context<T>* context,
                                    const Vector3<T>& angles) const {
    get_mobilizer()->set_angles(context, angles);
    return *this;
  }

  /// Sets the random distribution that angles of this joint will be randomly
  /// sampled from. See get_angles() for details on the angle representation.
  void set_random_angles_distribution(
      const Vector3<symbolic::Expression>& angles) {
    get_mutable_mobilizer()->set_random_position_distribution(
        Vector3<symbolic::Expression>{angles});
  }

  /// Retrieves from `context` the angular velocity `w_FM` of the child frame
  /// M in the parent frame F, expressed in F.
  ///
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @retval w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer()->get_angular_velocity(context);
  }

  /// Sets in `context` the state for `this` joint so that the angular velocity
  /// of the child frame M in the parent frame F is `w_FM`.
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  /// @returns a constant reference to `this` joint.
  const BallRpyJoint<T>& set_angular_velocity(systems::Context<T>* context,
                                              const Vector3<T>& w_FM) const {
    get_mobilizer()->set_angular_velocity(context, w_FM);
    return *this;
  }

  /// @}

  /// Gets the default angles for `this` joint. Wrapper for the more general
  /// `Joint::default_positions()`.
  /// @returns The default angles of `this` stored in `default_positions_`
  Vector3<double> get_default_angles() const {
    return this->default_positions();
  }

  /// Sets the default angles of this joint.
  /// @param[in] angles
  ///   The desired default angles of the joint
  void set_default_angles(const Vector3<double>& angles) {
    this->set_default_positions(angles);
  }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Adding forces per-dof makes no physical sense. Therefore, this method
  /// throws an exception if invoked.
  void DoAddInOneForce(const systems::Context<T>&, int, const T&,
                       MultibodyForces<T>*) const override {
    throw std::logic_error(
        "Ball RPY joints do not allow applying forces to individual degrees of "
        "freedom.");
  }

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into `forces` a dissipative torque according to the
  /// viscous law `τ = -d⋅ω`, with d the damping coefficient (see damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const override {
    Eigen::Ref<VectorX<T>> t_BMo_F =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    const Vector3<T>& w_FM = get_angular_velocity(context);
    t_BMo_F = -damping() * w_FM;
  }

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const override { return 3; }

  int do_get_position_start() const override {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const override { return 3; }

  std::string do_get_position_suffix(int index) const override {
    return get_mobilizer()->position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const override {
    return get_mobilizer()->velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) override {
    if (this->has_implementation()) {
      get_mutable_mobilizer()->set_default_position(default_positions);
    }
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const override;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  // Make BallRpyJoint templated on every other scalar type a friend of
  // BallRpyJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of BallRpyJoint<T>.
  template <typename>
  friend class BallRpyJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::SpaceXYZMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::SpaceXYZMobilizer<T>* mobilizer =
        dynamic_cast<const internal::SpaceXYZMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::SpaceXYZMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::SpaceXYZMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This joint's damping constant in N⋅m⋅s.
  double damping_{0};
};

template <typename T>
const char BallRpyJoint<T>::kTypeName[] = "ball_rpy";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::BallRpyJoint)
