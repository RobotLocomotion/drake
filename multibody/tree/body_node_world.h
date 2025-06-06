#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/multibody/tree/body_node.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Can we just get rid of this class and use a 0-dof BodyNodeImpl
//  instead?

// This class represents a BodyNode for the world body.
// Base class BodyNode methods are sufficient for this zero-dof node.
template <typename T>
class BodyNodeWorld final : public BodyNode<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNodeWorld);

  explicit BodyNodeWorld(const RigidBody<T>* rigid_body,
                         const Mobilizer<T>* mobilizer)
      : BodyNode<T>(nullptr, rigid_body, mobilizer) {
    DRAKE_DEMAND(rigid_body != nullptr && mobilizer != nullptr);
  }

  void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>&, const T*,
      PositionKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      std::vector<Vector6<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcVelocityKinematicsCache_BaseToTip(
      const T*, const PositionKinematicsCache<T>&,
      const std::vector<Vector6<T>>&, const T*,
      VelocityKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcMassMatrixContributionViaWorld_TipToBase(
      const PositionKinematicsCache<T>&, const std::vector<SpatialInertia<T>>&,
      const std::vector<Vector6<T>>&, EigenPtr<MatrixX<T>>) const final {
    DRAKE_UNREACHABLE();
  }

#define DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(Bnv)                      \
  void CalcMassMatrixOffDiagonalBlockViaWorld##Bnv(                         \
      int, const std::vector<Vector6<T>>&, const Eigen::Matrix<T, 6, Bnv>&, \
      EigenPtr<MatrixX<T>>) const final {                                   \
    DRAKE_UNREACHABLE();                                                    \
  }

  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(1)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(2)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(3)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(4)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(5)
  DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD(6)

#undef DEFINE_DUMMY_OFF_DIAGONAL_BLOCK_VIA_WORLD

  void CalcSpatialAcceleration_BaseToTip(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      const T*, const VelocityKinematicsCache<T>*, const T*,
      std::vector<SpatialAcceleration<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcInverseDynamics_TipToBase(const FrameBodyPoseCache<T>&, const T*,
                                     const PositionKinematicsCache<T>&,
                                     const std::vector<SpatialInertia<T>>&,
                                     const std::vector<SpatialForce<T>>*,
                                     const std::vector<SpatialAcceleration<T>>&,
                                     const std::vector<SpatialForce<T>>&,
                                     const Eigen::Ref<const VectorX<T>>&,
                                     std::vector<SpatialForce<T>>*,
                                     EigenPtr<VectorX<T>>) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyInertiaCache_TipToBase(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&, const SpatialInertia<T>&,
      const VectorX<T>&, ArticulatedBodyInertiaCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyForceCache_TipToBase(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const VelocityKinematicsCache<T>*, const SpatialForce<T>&,
      const ArticulatedBodyInertiaCache<T>&, const SpatialForce<T>&,
      const SpatialForce<T>&, const Eigen::Ref<const VectorX<T>>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&,
      ArticulatedBodyForceCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcArticulatedBodyAccelerations_BaseToTip(
      const systems::Context<T>&, const PositionKinematicsCache<T>&,
      const ArticulatedBodyInertiaCache<T>&,
      const ArticulatedBodyForceCache<T>&,
      const Eigen::Ref<const MatrixUpTo6<T>>&, const SpatialAcceleration<T>&,
      AccelerationKinematicsCache<T>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcCompositeBodyInertiaInWorld_TipToBase(
      const PositionKinematicsCache<T>&, const std::vector<SpatialInertia<T>>&,
      std::vector<SpatialInertia<T>>*) const final {
    DRAKE_UNREACHABLE();
  }

  void CalcSpatialAccelerationBias(
      const FrameBodyPoseCache<T>&, const T*, const PositionKinematicsCache<T>&,
      const T*, const VelocityKinematicsCache<T>&,
      std::vector<SpatialAcceleration<T>>*) const final {
    DRAKE_UNREACHABLE();
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
