#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/multibody_tree_context.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/// This mobilizer fixes the relative pose `X_FM` of an outboard frame M in an
/// inboard frames F as if "welding" them together at this fixed relative pose.
/// Therefore, this mobilizer has no associated state with it.
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
class WeldMobilizer final : public MobilizerImpl<T, 0, 0> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WeldMobilizer)

  /// Constructor for a %WeldMobilizer between the `inboard_frame_F` and
  /// `outboard_frame_M`.
  /// @param[in] X_FM Pose of `outboard_frame_M` in the `inboard_frame_F`.
  WeldMobilizer(const Frame<T>& inboard_frame_F,
                const Frame<T>& outboard_frame_M,
                const Isometry3<double>& X_FM) :
      MobilizerBase(inboard_frame_F, outboard_frame_M), X_FM_(X_FM) {}

  /// @retval X_FM The pose of the outboard frame M in the inboard frame F.
  const Isometry3<double>& get_X_FM() const { return X_FM_; }

  /// This override is a no-op for this mobilizer since it has no state
  /// associated with it.
  void set_zero_state(const systems::Context<T>& context,
                      systems::State<T>* state) const final;

  /// Computes the across-mobilizer transform `X_FM`, which for this mobilizer
  /// is independent of the state stored in `context`.
  Isometry3<T> CalcAcrossMobilizerTransform(
      const MultibodyTreeContext<T>& context) const final;

  /// Computes the across-mobilizer velocity `V_FM` which for this mobilizer is
  /// always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialVelocity<T> CalcAcrossMobilizerSpatialVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v) const final;

  /// Computes the across-mobilizer acceleration `A_FM` which for this mobilizer
  /// is always zero since the outboard frame M is fixed to the inboard frame F.
  SpatialAcceleration<T> CalcAcrossMobilizerSpatialAcceleration(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& vdot) const final;

  /// Since this mobilizer has no generalized velocities associated with it,
  /// this override is a no-op.
  void ProjectSpatialForce(
      const MultibodyTreeContext<T>& context,
      const SpatialForce<T>& F_Mo_F,
      Eigen::Ref<VectorX<T>> tau) const final;

  /// This override is a no-op since this mobilizer has no generalized
  /// velocities associated with it.
  void MapVelocityToQDot(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& v,
      EigenPtr<VectorX<T>> qdot) const final;

  /// This override is a no-op since this mobilizer has no generalized
  /// velocities associated with it.
  void MapQDotToVelocity(
      const MultibodyTreeContext<T>& context,
      const Eigen::Ref<const VectorX<T>>& qdot,
      EigenPtr<VectorX<T>> v) const final;

 protected:
  std::unique_ptr<Mobilizer<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Mobilizer<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

 private:
  typedef MobilizerImpl<T, 0, 0> MobilizerBase;
  // Bring the handy number of position and velocities MobilizerImpl enums into
  // this class' scope. This is useful when writing mathematical expressions
  // with fixed-sized vectors since we can do things like Vector<T, nq>.
  // Operations with fixed-sized quantities can be optimized at compile time
  // and therefore they are highly preferred compared to the very slow dynamic
  // sized quantities.
  using MobilizerBase::kNq;
  using MobilizerBase::kNv;

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Mobilizer<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // Pose of the outboard frame M in the inboard frame F.
  Isometry3<double> X_FM_;
};

}  // namespace multibody
}  // namespace drake
