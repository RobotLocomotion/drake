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

/// This Joint fixes the relative pose between two frames as if "welding" them
/// together.
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

  /// Constructor for a %WeldJoint between a `parent_frame_P` and a
  /// `child_frame_C` so that their relative pose `X_PC` is fixed as if they
  /// were "welded" together.
  WeldJoint(const std::string& name,
            const Frame<T>& parent_frame_P, const Frame<T>& child_frame_C,
            const Isometry3<double>& X_PC) :
      Joint<T>(name, parent_frame_P, child_frame_C), X_PC_(X_PC) {}

  /// Returns the pose X_PC of frame C in P.
  const Isometry3<double>& X_PC() const {
    return X_PC_;
  }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Since frame P and C are welded together, it is physically not possible to
  /// apply forces between them. Therefore this method throws an exception if
  /// invoked.
  void DoAddInOneForce(
      const systems::Context<T>&, int, const T&,
      MultibodyForces<T>*) const override {
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

  // The pose of frame C in P.
  Isometry3<double> X_PC_;
};

}  // namespace multibody
}  // namespace drake
