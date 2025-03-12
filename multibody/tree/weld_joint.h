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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  // Note: we're calling these frames F and M for the user because that's how
  // we document joints. However, we should have called those Jp (for joint's
  // parent frame) and Jc (child frame) since we use F and M internally for
  // the _mobilizer_ inboard and outboard frames, resp. Normally F=Jp and
  // M=Jc, but mobilizers can be reversed so that F=Jc and M=Jp. We'll switch
  // notation internally so that we can use F and M for mobilizers.

  /// Constructor for a %WeldJoint between a `frame_on_parent_F` and a
  /// `frame_on_child_M` so that their relative pose `X_FM` is fixed as if
  /// they were "welded" together.
  WeldJoint(const std::string& name, const Frame<T>& frame_on_parent_F,
            const Frame<T>& frame_on_child_M,
            const math::RigidTransform<double>& X_FM)
      : Joint<T>(name, frame_on_parent_F, frame_on_child_M,
                 VectorX<double>() /* no pos lower limits */,
                 VectorX<double>() /* no pos upper limits */,
                 VectorX<double>() /* no vel lower limits */,
                 VectorX<double>() /* no vel upper limits */,
                 VectorX<double>() /* no acc lower limits */,
                 VectorX<double>() /* no acc upper limits */),
        X_JpJc_(X_FM) {}

  ~WeldJoint() final;

  const std::string& type_name() const final;

  // See note above. We're returning X_JpJc here.
  /// Returns the pose X_FM of frame M in F.
  const math::RigidTransform<double>& X_FM() const { return X_JpJc_; }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Since frame F and M are welded together, it is physically not possible to
  /// apply forces between them. Therefore this method throws an exception if
  /// invoked.
  void DoAddInOneForce(const systems::Context<T>&, int, const T&,
                       MultibodyForces<T>*) const final {
    throw std::logic_error("Weld joints do not allow applying forces.");
  }

 private:
  int do_get_velocity_start() const final {
    // Since WeldJoint has no state, the start index has no meaning. However,
    // we let its decide the return value for this case (this has to do with
    // allowing zero sized Eigen blocks).
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 0; }

  int do_get_position_start() const final {
    // Since WeldJoint has no state, the start index has no meaning. However,
    // we let it decide the return value for this case (this has to do with
    // allowing zero sized Eigen blocks).
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const final { return 0; }

  std::string do_get_position_suffix(int index) const final {
    return get_mobilizer().position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const final {
    return get_mobilizer().velocity_suffix(index);
  }

  void do_set_default_positions(const VectorX<double>&) final { return; }

  // Joint<T> overrides:
  std::unique_ptr<internal::Mobilizer<T>> MakeMobilizerForJoint(
      const internal::SpanningForest::Mobod& mobod,
      internal::MultibodyTree<T>* tree) const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>& x) const final;

  std::unique_ptr<Joint<T>> DoShallowClone() const final;

  // Make WeldJoint templated on every other scalar type a friend of
  // WeldJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of WeldJoint<T>.
  template <typename>
  friend class WeldJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::WeldMobilizer<T>& get_mobilizer() const {
    return this->template get_mobilizer_downcast<internal::WeldMobilizer>();
  }

  internal::WeldMobilizer<T>& get_mutable_mobilizer() {
    return this
        ->template get_mutable_mobilizer_downcast<internal::WeldMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // The pose of frame M in F.
  const math::RigidTransform<double> X_JpJc_;
};

template <typename T>
const char WeldJoint<T>::kTypeName[] = "weld";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::WeldJoint);
