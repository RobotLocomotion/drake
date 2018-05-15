#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/weld_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to rotate relatively to one another around a
/// common axis.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frames F and M to rotate with respect to each other about
/// an axis â. The rotation angle's sign is defined such that child body B
/// rotates about axis â according to the right hand rule, with thumb aligned in
/// the axis direction.
/// Axis â is constant and has the same measures in both frames F and M, that
/// is, `â_F = â_M`.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class WeldJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// Constructor to create a revolute joint between two bodies so that
  /// frame F attached to the parent body P and frame M attached to the child
  /// body B, rotate relatively to one another about a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and rotation angle.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameter `axis` is:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame M only rotates with respect to F and their origins are
  ///   coincident at all times, the measures of `axis` in either frame F or M
  ///   are exactly the same, that is, `axis_F = axis_M`. In other words,
  ///   `axis_F` (or `axis_M`) is the eigenvector of `R_FM` with eigenvalue
  ///   equal to one.
  ///   This vector can have any length, only the direction is used. This method
  ///   aborts if `axis` is the zero vector.
  WeldJoint(const std::string& name,
            const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
            const Isometry3<double>& X_PC) :
      Joint<T>(name, frame_on_parent, frame_on_child), X_PC_(X_PC) {}

  /// Returns the axis of revolution of `this` joint as a unit vector.
  /// Since the measures of this axis in either frame F or M are the same (see
  /// this class's documentation for frames's definitions) then,
  /// `axis = axis_F = axis_M`.
  const Isometry3<double>& X_PC() const {
    return X_PC_;
  }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  void DoAddInOneForce(
      const systems::Context<T>&,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const override {
    throw std::logic_error("Weld joints do not allow applying forces.");
  }

 private:
  int do_get_num_dofs() const override {
    return 0;
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint>
  MakeImplementationBlueprint() const override {
    auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
    blue_print->mobilizers_.push_back(
        std::make_unique<WeldMobilizer<T>>(
            this->frame_on_parent(), this->frame_on_child(), X_PC_));
    return std::move(blue_print);
  }

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const override;

  // Make WeldJoint templated on every other scalar type a friend of
  // WeldJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of WeldJoint<T>.
  template <typename> friend class WeldJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  Isometry3<double> X_PC_;
};

}  // namespace multibody
}  // namespace drake
