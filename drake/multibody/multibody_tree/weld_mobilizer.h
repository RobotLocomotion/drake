#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/mobilizer_impl.h"
#include "drake/multibody/multibody_tree/math/spatial_algebra_old.h"

namespace drake {
namespace multibody {

template <typename T>
class WeldMobilizer : public MobilizerImpl<T, 0, 0> {
  typedef MobilizerImpl<T, 0, 0> MobilizerBase;
  using MobilizerBase::nq;
  using MobilizerBase::nv;
  using typename MobilizerBase::HMatrix;
  using MobilizerBase::get_id;
 public:
  /// Creates a mobilizer that welds together an outboard frame `M` to an
  /// inboard frame `F` with a fixed pose `X_FM`.
  WeldMobilizer(const BodyFrame<T>& inboard_frame,
                const BodyFrame<T>& outboard_frame,
                const Isometry3<T>& X_FM) :
      MobilizerBase(inboard_frame, outboard_frame), X_FM_(X_FM) {}

#if 0
  void CalcAcrossMobilizerTransform(
      const MobilizerContext<T>& context,
      MobilizerPositionKinematics<T>* pc) const final {
    pc->get_mutable_X_FM() = X_FM_;
  }

  // No-op.
  void CalcAcrossMobilizerVelocityJacobian(
      const MobilizerContext<T>& context,
      MobilizerPositionKinematics<T>* pc) const final {}
#endif

 private:
  // static constexpr int i = 42; discouraged.
  // See answer in: http://stackoverflow.com/questions/37259807/static-constexpr-int-vs-old-fashioned-enum-when-and-why
  //enum : int {nq = 0, nv = 0};

  // Pose of frame `M` in frame `F`.
  const Isometry3<T> X_FM_;
};

}  // namespace multibody
}  // namespace drake
