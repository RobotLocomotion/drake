#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/body_node.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {
namespace internal {

// TODO(sherm1) Most of the across-mobilizer code for kinematics and
//  dynamics should live in this class since the hinge matrices
//  and related downstream calculations have a compile-time fixed size.
//  Kinematics are here now; move the rest.

// For internal use only of the MultibodyTree implementation.
// While all code that is common to any node _could_ be placed in the BodyNode
// class, BodyNodeImpl is templatized on the concrete Mobilizer type so
// implementations here can use fixed-size objects and mobilizer-specific
// inline implementations for maximum speed. For a more detailed discussion of
// the role of a BodyNode in a MultibodyTree refer to the class documentation
// for BodyNode.
template <typename T, template <typename> class ConcreteMobilizer>
class BodyNodeImpl final : public BodyNode<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BodyNodeImpl);

  // Inherit sizes from the concrete mobilizer.
  enum : int {
    kNq = ConcreteMobilizer<T>::kNq,
    kNv = ConcreteMobilizer<T>::kNv,
    kNx = ConcreteMobilizer<T>::kNx
  };
  using QVector = typename ConcreteMobilizer<T>::QVector;
  using VVector = typename ConcreteMobilizer<T>::VVector;
  using HMatrix = typename ConcreteMobilizer<T>::HMatrix;

  // Given a body and its inboard mobilizer in a MultibodyTree this constructor
  // creates the corresponding %BodyNode. See the BodyNode class documentation
  // for details on how a BodyNode is defined.
  // @param[in] parent_node
  //   A const pointer to the parent BodyNode object in the tree structure of
  //   the owning MultibodyTree.
  // @param[in] body The body B associated with `this` node. Can't be World.
  // @param[in] mobilizer The mobilizer associated with this `node`.
  //
  // @note World is handled specially and does not go through this path. There
  //   is a world BodyNode and dummy world Mobilizer 0; see MultibodyTree
  //   CreateJointImplementations() and CreateBodyNode(). The BodyNode
  //   constructor used here will abort if we try this with World.
  BodyNodeImpl(const internal::BodyNode<T>* parent_node,
               const RigidBody<T>* body, const Mobilizer<T>* mobilizer);

  ~BodyNodeImpl() final;

  // TODO(sherm1) Just a warm up -- move the rest of the kernel computations
  //  here also.

  void CalcPositionKinematicsCache_BaseToTip(
      const FrameBodyPoseCache<T>& frame_body_pose_cache,
      const T* positions,
      PositionKinematicsCache<T>* pc) const final;

  void CalcAcrossNodeJacobianWrtVExpressedInWorld(
      const systems::Context<T>& context,
      const FrameBodyPoseCache<T>& frame_body_pose_cache,
      const PositionKinematicsCache<T>& pc,
      std::vector<Vector6<T>>* H_PB_W_cache) const final;

  void CalcVelocityKinematicsCache_BaseToTip(
      const systems::Context<T>& context,
      const PositionKinematicsCache<T>& pc,
      const std::vector<Vector6<T>>& H_PB_W_cache,
      const T* velocities,
      VelocityKinematicsCache<T>* vc) const final;

 private:
  // Given a pointer to the contiguous array of all q's in this system, returns
  // a pointer to the ones for this mobilizer.
  // @pre `positions` is the full set of q's for this system
  const T* get_q(const T* positions) const {
    return  &positions[mobilizer_->position_start_in_q()];
  }

  // Given a pointer to the contiguous array of all v's in this system, returns
  // a pointer to the ones for this mobilizer.
  // @pre `velocities` is the full set of v's for this system
  const T* get_v(const T* velocities) const {
    return &velocities[mobilizer_->velocity_start_in_v()];
  }

  // Given a complete array of hinge matrices H stored by contiguous columns,
  // returns a const reference to H for this mobilizer, as a 6xnv fixed-size
  // matrix. This matrix is 16-byte aligned because it is composed of
  // columns of Eigen::Vector6 objects which Eigen aligns.
  Eigen::Map<const HMatrix, Eigen::Aligned16> get_H(
      const std::vector<Vector6<T>>& H_cache) const {
    return Eigen::Map<const HMatrix, Eigen::Aligned16>(
        H_cache[mobilizer_->velocity_start_in_v()].data());
  }

  // Given a pointer to a mutable complete array of hinge matrices H stored by
  // contiguous columns, returns a mutable reference to H for this mobilizer,
  // as a 6xnv fixed-size matrix. This matrix is 16-byte aligned because it is
  // composed of columns of Eigen::Vector6 objects which Eigen aligns.
  Eigen::Map<HMatrix, Eigen::Aligned16> get_mutable_H(
      std::vector<Vector6<T>>* H_cache) const {
    DRAKE_ASSERT(H_cache != nullptr);
    return Eigen::Map<HMatrix, Eigen::Aligned16>(
        (*H_cache)[mobilizer_->velocity_start_in_v()].data());
  }

  SpatialVelocity<T>& get_mutable_V_WB(VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_WB(this->index());
  }

  SpatialVelocity<T>& get_mutable_V_FM(
      VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_FM(this->index());
  }

  SpatialVelocity<T>& get_mutable_V_PB_W(
      VelocityKinematicsCache<T>* vc) const {
    return vc->get_mutable_V_PB_W(this->index());
  }

  const ConcreteMobilizer<T>* const mobilizer_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
