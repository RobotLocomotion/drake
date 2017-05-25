#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/body.h"
#include "drake/multibody/multibody_tree/mobilizer.h"
#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;
#define PRINT_VARn(x) std::cout <<  #x ":\n" << x << std::endl;

namespace drake {
namespace multibody {

// Forward declaration.
template<typename T> class MultibodyTree;

template <typename T>
class BodyNode : public MultibodyTreeElement<BodyNode<T>, BodyNodeIndex> {
 public:
  BodyNode(const Body<T>* body, const Mobilizer<T>* mobilizer) :
      body_(body), mobilizer_(mobilizer) {
    DRAKE_DEMAND(body != nullptr);
    DRAKE_DEMAND(!(mobilizer == nullptr && body->get_index() != world_index()));
  }

  const Body<T>& get_body() const {
    return *body_;
  }

  const Body<T>& get_parent_body() const {
    DRAKE_ASSERT(get_parent_body_index().is_valid());
    return this->get_parent_tree().get_body(get_parent_body_index());
  }

  const Mobilizer<T>& get_mobilizer() const {
    DRAKE_ASSERT(get_mobilizer_index().is_valid());
    return this->get_parent_tree().get_mobilizer(get_mobilizer_index());
  }

  MobilizerIndex get_mobilizer_index() const { return topology_.mobilizer;}

  BodyIndex get_body_index() const { return topology_.body;}

  BodyIndex get_parent_body_index() const { return topology_.parent_body;}

  /// This method can anly be called within a base-to-tip loop.
  void CalcPositionKinematicsCache_BaseToTip(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {
    // This method should not be called for the "world" body node.
    DRAKE_ASSERT(topology_.body != world_index());

    // This computes into the PositionKinematicsCache:
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    // It assumes:
    // - The body for this node updates its attached frame poses X_BF(qf_B).
    // - We are in a base-to-tip recursion and therefore X_PF(qf_P) and X_WP are
    //   available.
    CalcAcrossMobilizerBodyPoses_BaseToTip(context, pc);

    // TODO(amcastro-tri):
    // Update Body specific kinematics. These are:
    // - phi_PB_W: shift operator from P to B.
    // - com_W: center of mass.
    // - M_Bo_W: Spatial inertia.

    // TODO(amcastro-tri):
    // With H_FM(qr) already in the cache (computed by
    // Mobilizer::UpdatePositionKinematicsCache()) update the cache
    // entries for H_PB_W, the Jacobian for the SpatialVelocity jump between
    // body B and its parent body P expressed in the world frame W.
  }

  /// Returns the topology information for this body node.
  const BodyNodeTopology& get_topology() const { return topology_; }

 protected:
  BodyNodeTopology topology_;
  // Pointers for fast access.
  const Body<T>* body_{nullptr};
  const Mobilizer<T>* mobilizer_{nullptr};

  /// This computes into the PositionKinematicsCache:
  /// - X_PB(qf_P, qr_B, qf_B)
  /// - X_WB(q(W:P), qf_P, qr_B, qf_B)
  /// It assumes:
  /// - The body for this node updates its attached frame poses X_BF(qf_B).
  /// - We are in a base-to-tip recursion and therefore X_PF(qf_P) and X_WP are
  ///   available.
  void CalcAcrossMobilizerBodyPoses_BaseToTip(
      const MultibodyTreeContext<T>& context,
      PositionKinematicsCache<T>* pc) const {

    // Body for this node.
    const Body<T>& BodyB = get_body();

    // Body for this node's parent, or the parent body P.
    const Body<T>& BodyP = get_parent_body();

    // Inboard/Outboard frames of this node's mobilizer.
    const Frame<T>& FrameF = get_mobilizer().get_inboard_frame();
    DRAKE_ASSERT(FrameF.get_body().get_index() == BodyP.get_index());
    const Frame<T>& FrameM = get_mobilizer().get_outboard_frame();
    DRAKE_ASSERT(FrameM.get_body().get_index() == BodyB.get_index());

    // Input (const):
    // - X_PF(qb_P)
    // - X_MB(qb_B)
    // - X_FM(qm_B)
    // - X_WP(q(W:B)), where q(W:B) includes all positions in the kinematics
    //                 path from body B to the world W.
    const Isometry3<T> X_MB = FrameM.CalcBodyPoseInThisFrame(context);
    const Isometry3<T>& X_FM = get_X_FM(pc);  // mobilizer.Eval_X_FM(ctx)
    const Isometry3<T>& X_WP = get_X_WP(pc);  // BodyP.EvalPoseInWorld(ctx)

    // Output (updating a cache entry):
    // - X_PB(qf_P, qr_B, qf_B)
    // - X_WB(q(W:P), qf_P, qr_B, qf_B)
    Isometry3<T>& X_PB = get_mutable_X_PB(pc);  // BodyP.EvalPoseInWorld(ctx)
    Isometry3<T>& X_WB = get_mutable_X_WB(pc);

    // TODO(amcastro-tri): Consider logic for the common case B = M.
    // In that case X_FB = X_FM as suggested by setting X_MB = Id.
    const Isometry3<T> X_FB = X_FM * X_MB;

    // Given the pose X_FB of body frame B measured in the mobilizer inboard
    // frame F, we can ask frame F (who's parent body is P) for the pose of body
    // B measured in the frame of the parent body P.
    // In the particular case F = B, this method directly returns X_FB.
    // For flexible bodies this gives the chance to frame F to pull its pose
    // from the context.
    X_PB = FrameF.CalcOffsetPoseInBody(context, X_FB);

    X_WB = X_WP * X_PB;
  }

 private:

  // =========================================================================
  // PositionKinematicsCache Accessors and Mutators.

  Isometry3<T>& get_mutable_X_WB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.index);
  }

  const Isometry3<T>& get_X_WP(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_WB(topology_.parent_body_node);
  }

  const Isometry3<T>& get_X_FM(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_FM(topology_.index);
  }

  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.index);
  }

  // Implementation for MultibodyTreeElement::DoSetTopology().
  // At MultibodyTree::Finalize() time, each body retrieves its topology
  // from the parent MultibodyTree.
  void DoSetTopology(const MultibodyTreeTopology& tree_topology) final {
    topology_ = tree_topology.get_body_node(this->get_index());
  }

#if 0
  // Get from the position kinematics cache a constant reference to the pose
  // X_PF of the "fixed" frame F as measured and expressed in the inboard
  // (parent) body frame P.
  const Isometry3<T>& get_X_PF(const PositionKinematicsCache<T>& pc) const {
    const auto& pool = pc.get_X_BF_pool();
    DRAKE_ASSERT(topology_.X_PF_index < static_cast<int>(pool.size()));
    return pool[topology_.X_PF_index];
  }

  Isometry3<T>& get_mutable_X_PF(PositionKinematicsCache<T>* pc) const {
    auto& pool = pc->get_mutable_X_BF_pool();
    DRAKE_ASSERT(topology_.X_PF_index < static_cast<int>(pool.size()));
    return pool[topology_.X_PF_index];
  }
#endif

#if 0
  const Isometry3<T>& get_X_FM(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_FM(topology_.id);
  }

  const Isometry3<T>& get_X_WP(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_WB(topology_.parent_body_node);
  }

  const Isometry3<T>& get_X_PB(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_PB(topology_.id);
  }

  Isometry3<T>& get_mutable_X_PB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_PB(topology_.id);
  }

  const Isometry3<T>& get_X_WB(const PositionKinematicsCache<T>* pc) const {
    return pc->get_X_WB(topology_.id);
  }

  Isometry3<T>& get_mutable_X_WB(PositionKinematicsCache<T>* pc) const {
    return pc->get_mutable_X_WB(topology_.id);
  }
#endif
};

}  // namespace multibody
}  // namespace drake
