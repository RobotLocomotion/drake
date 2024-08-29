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

// For internal use only of the MultibodyTree implementation.
// While all code that is common to any node can be placed in the BodyNode
// class, %BodyNodeImpl provides compile-time fixed-size BodyNode
// implementations so that all operations can be performed with fixed-size
// stack-allocated Eigen variables. For a more detailed discussion of the role
// of a BodyNode in a MultibodyTree refer to the class documentation for
// BodyNode.
template <typename T, template <typename> class ConcreteMobilizer>
class BodyNodeImpl final : public BodyNode<T> {
 public:
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
  //   the owning MultibodyTree. It can be a `nullptr` only when `body` **is**
  //   the **world** body, otherwise the parent class constructor will abort.
  // @param[in] body The body B associated with `this` node.
  // @param[in] mobilizer The mobilizer associated with this `node`.
  BodyNodeImpl(const internal::BodyNode<T>* parent_node,
               const RigidBody<T>* body, const Mobilizer<T>* mobilizer)
      : BodyNode<T>(parent_node, body, mobilizer),
        mobilizer_(dynamic_cast<const ConcreteMobilizer<T>*>(mobilizer)) {
    DRAKE_DEMAND(mobilizer_ != nullptr);
  }

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
  // a reference to just the ones for this mobilizer, as a fixed-size vector.
  const QVector& my_q(const T* positions) const {
    return ConcreteMobilizer<T>::to_q_vector(
        &positions[mobilizer_->position_start_in_q()]);
  }

  // Given a pointer to the contiguous array of all v's in this system, returns
  // a reference to just the ones for this mobilizer, as a fixed-size vector.
  const VVector& my_v(const T* velocities) const {
    return ConcreteMobilizer<T>::to_v_vector(
        &velocities[mobilizer_->velocity_start_in_v()]);
  }

  // Given a complete array of hinge matrices H stored by contiguous columns,
  // returns a const reference to H for this mobilizer, as a 6xnv fixed-size
  // matrix.
  const HMatrix& my_H(const std::vector<Vector6<T>>& H_cache) const {
    return ConcreteMobilizer<T>::to_h_matrix(
        &H_cache[mobilizer_->velocity_start_in_v()]);
  }

  // Given a pointer to a mutable complete array of hinge matrices H stored by
  // contiguous columns, returns a mutable reference to H for this mobilizer,
  // as a 6xnv fixed-size matrix.
  HMatrix& my_mutable_H(std::vector<Vector6<T>>* H_cache) const {
    DRAKE_ASSERT(H_cache != nullptr);
    return ConcreteMobilizer<T>::to_mutable_h_matrix(
        &(*H_cache)[mobilizer_->velocity_start_in_v()]);
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

  const ConcreteMobilizer<T>* mobilizer_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
