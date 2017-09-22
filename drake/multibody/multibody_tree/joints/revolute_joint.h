#pragma once

#include <limits>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

template <typename T>
class RevoluteJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame M only rotates with respect to F and their origins are
  ///   coincident at all times, the measures of `axis` in either frame F or M
  ///   are exactly the same, that is, `axis_F = axis_M`. In other words,
  ///   `axis_F` (or `axis_M`) is the eigenvector of `R_FM` with eigenvalue
  ///   equal to one.
  RevoluteJoint(const std::string& name,
                const RigidBody<T>& inboard_body, const Isometry3<double> X_PF,
                const RigidBody<T>& outboard_body, const Isometry3<double> X_BM,
                const Vector3<double>& axis) :
      Joint<T>(name, inboard_body, X_PF, outboard_body, X_BM) {
    // DRAKE_DEMAND axis is not the zero vector!!!
    axis_ = axis.normalized();
  }

  /// Returns the axis of revolution of `this` joint as a unit vector expressed
  /// in the frame `Jp` attached on the parent body P.
  const Vector3<double>& get_revolute_axis() const {
    return axis_;
  }

  /// Gets the rotation angle of `this` mobilizer from `context`. See class
  /// documentation for sign convention.
  /// @throws std::logic_error if the parent MultibodyModeler of `this` joint
  /// was not finalized, @see MultibodyModeler::Finalize().
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @returns The angle coordinate of `this` mobilizer in the `context`.
  const T& get_angle(const Context<T>& context) const {
    return get_mobilizer()->get_angle(context);
  }

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` mobilizer equals `angle`.
  /// @throws std::logic_error if `context` is not a valid
  /// MultibodyTreeContext.
  /// @param[in] context The context of the MultibodyTree this mobilizer
  ///                    belongs to.
  /// @param[in] angle The desired angle in radians.
  /// @returns a constant reference to `this` mobilizer.
  const RevoluteJoint<T>& set_angle(
      Context<T>* context, const T& angle) const {
    get_mobilizer()->set_angle(context, angle);
    return *this;
  }

  const T& get_angular_rate(const Context<T>& context) const {
    return get_mobilizer()->get_angular_rate(context);
  }

  const RevoluteJoint<T>& set_angular_rate(
      Context<T>* context, const T& angle) const {
    get_mobilizer()->set_angular_rate(context, angle);
    return *this;
  }

  // Notice the covariant return.
  //const RevoluteMobilizer<T>& get_mobilizer() const {
//    DRAKE_DEMAND(mobilizer_ != nullptr);
  //  return *mobilizer_;
  //}

 protected:
  void DoMakeModelAndAdd(MultibodyTree<T>* tree) {
    mobilizer_ = &tree->template AddMobilizer<RevoluteMobilizer>(
        this->get_inboard_frame(), this->get_outboard_frame(), axis_);
  }

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const;

 private:
  // Make RevoluteJoint templated on every other scalar type a friend of
  // RevoluteJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of RevoluteJoint<T>.
  template <typename> friend class RevoluteJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  const RevoluteMobilizer<T>* get_mobilizer() const {
    DRAKE_DEMAND(mobilizer_ != nullptr);
    return mobilizer_;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  Vector3<double> axis_;
  const RevoluteMobilizer<T>* mobilizer_{nullptr};
};

}  // namespace multibody
}  // namespace drake
