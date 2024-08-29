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

  explicit BodyNodeWorld(const RigidBody<T>* body)
      : BodyNode<T>(nullptr, body, nullptr) {}

  void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>& /* frame_body_pose_cache */,
      const T* /* positions */,
      PositionKinematicsCache<T>* /* pc */) const override {
    DRAKE_UNREACHABLE();
  }

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& /* context */,
      const FrameBodyPoseCache<T>& /* frame_body_pose_cache */,
      const PositionKinematicsCache<T>& /* pc */,
      std::vector<Vector6<T>>* /* H_PB_W_cache */) const override {
    DRAKE_UNREACHABLE();
  }

  void CalcVelocityKinematicsCache_BaseToTip(
      const systems::Context<T>& /* context */,
      const PositionKinematicsCache<T>& /* pc */,
      const std::vector<Vector6<T>>& /* H_PB_W_cache */,
      const T* /* velocities */,
      VelocityKinematicsCache<T>* /* vc */) const override {
    DRAKE_UNREACHABLE();
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
