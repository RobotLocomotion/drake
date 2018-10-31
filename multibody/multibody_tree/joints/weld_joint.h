#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
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
///
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class WeldJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// Constructs a %WeldJoint between a parent frame `P` and a child frame `C`
  /// with a constant pose `X_PC`, as if `P` and `C` were "welded" together.
  /// @param[in] name Weld joint's name.
  /// @param[in] P Weld joint's parent frame.
  /// @param[in] C Weld joint's child frame.
  /// @param[in] X_PC Pose relating parent frame P and child frame C.
  WeldJoint(const std::string& name, const Frame<T>& P, const Frame<T>& C,
            const Isometry3<double>& X_PC) :
      Joint<T>(name, P, C, VectorX<double>() /* no lower limits */,
                           VectorX<double>() /* no upper limits */),
                           X_PC_(X_PC) {}

  /// Constructs a %WeldJoint between a parent frame `P` and a child frame `C`
  /// with a constant pose `X_PC`, as if `P` and `C` were "welded" together.
  /// @param[in] name Weld joint's name.
  /// @param[in] P Weld joint's parent frame.
  /// @param[in] C Weld joint's child frame.
  /// @param[in] X_PC Pose relating parent frame P and child frame C.
  WeldJoint(const std::string& name, const Frame<T>& P, const Frame<T>& C,
            const math::RigidTransform<double>& X_PC) :
      Joint<T>(name, P, C, VectorX<double>() /* no lower limits */,
                           VectorX<double>() /* no upper limits */),
                           X_PC_(X_PC.GetAsIsometry3()) {}

  /// Returns the pose X_PC relating parent frame P and child frame C.
  const Isometry3<double>& X_PC() const { return X_PC_; }

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
  int do_get_velocity_start() const override {
    // Since WeldJoint has no state, the start index has no meaning. However,
    // we let its decide the return value for this case (this has to do with
    // allowing zero sized Eigen blocks).
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const override {
    return 0;
  }

  int do_get_position_start() const override {
    // Since WeldJoint has no state, the start index has no meaning. However,
    // we let it decide the return value for this case (this has to do with
    // allowing zero sized Eigen blocks).
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const override {
    return 0;
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint>
  MakeImplementationBlueprint() const override;

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

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const WeldMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const WeldMobilizer<T>* mobilizer =
        dynamic_cast<const WeldMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // The pose of frame C in P.
  const Isometry3<double> X_PC_;
};

}  // namespace multibody
}  // namespace drake
