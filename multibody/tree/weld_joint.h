#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/weld_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint fixes the relative pose between two frames as if "welding" them
/// together.
///
/// @tparam_default_scalar
template <typename T>
class WeldJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /// Constructor for a %WeldJoint between a `parent_frame_P` and a
  /// `child_frame_C` so that their relative pose `X_PC` is fixed as if they
  /// were "welded" together.
  WeldJoint(const std::string& name, const Frame<T>& parent_frame_P,
            const Frame<T>& child_frame_C,
            const math::RigidTransform<double>& X_PC)
      : Joint<T>(name, parent_frame_P, child_frame_C,
                 VectorX<double>() /* no pos lower limits */,
                 VectorX<double>() /* no pos upper limits */,
                 VectorX<double>() /* no vel lower limits */,
                 VectorX<double>() /* no vel upper limits */,
                 VectorX<double>() /* no acc lower limits */,
                 VectorX<double>() /* no acc upper limits */),
        X_PC_(X_PC) {}

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns the pose X_PC of frame C in P.
  const math::RigidTransform<double>& X_PC() const {
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
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&x) const override;

  // Make WeldJoint templated on every other scalar type a friend of
  // WeldJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of WeldJoint<T>.
  template <typename> friend class WeldJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::WeldMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::WeldMobilizer<T>* mobilizer =
        dynamic_cast<const internal::WeldMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::WeldMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::WeldMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // The pose of frame C in P.
  const math::RigidTransform<double> X_PC_;
};

template <typename T> const char WeldJoint<T>::kTypeName[] = "weld";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::WeldJoint)
