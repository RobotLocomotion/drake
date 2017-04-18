#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"
#include "drake/multibody/multibody_tree/frame.h"

namespace drake {
namespace multibody {

// Forward declarations.
template <class T> class MultibodyTree;
template <class T> class RigidBody;

/// This class represents a frame F with a fixed pose `X_PF` measured and
/// expressed in another physical frame P. For instance, we could rigidly
/// attach a frame F to move with a rigid body B at a fixed pose `X_BF`
/// measured and expressed in the frame of that body. Thus, the pose of a
/// %FixedOffsetFrame in the world depends only on its constant pose `X_BF`
/// and the state of the associated body's frame B.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class FixedOffsetFrame : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FixedOffsetFrame)

  /// Creates a physical frame with a fixed pose `X_PF` measured and expressed
  /// in another physical frame P.
  ///
  /// @warning This constructor will allow, in future implementations, chaining
  /// multiple frames of type FixedOffsetFrame. However, this chaining is not
  /// yet supported and attempts to attach a %FixedOffsetFrame to a frame with
  /// type other than BodyFrame will throw an exception of type
  /// std::logic_error.
  ///
  /// @param[in] P The physical frame to which this frame is attached with a
  ///              fixed pose.
  /// @param[in] X_PF The fixed pose of the newly created frame measured and
  ///                 expressed in the physical frame P.
  // TODO(amcastro-tri): allow to chain multiple frames of type
  // FixedOffsetFrame. An approach would consist on holding a reference to the
  // parent frame of the root FixedOffsetFrame of the chain and X_PF_ would
  // then be set to (at construction) to the pose of this frame on that parent
  // frame.
  FixedOffsetFrame(const Frame<T>& P, const Isometry3<T>& X_PF);

  /// Creates a physical frame with a fixed pose `X_BF` measured and expressed
  /// in the frame B of the input `body`.
  ///
  /// @param[in] body The body in whose frame B this frame's pose `X_BF` is
  ///                 defined.
  /// @param[in] X_BF The fixed pose of the newly created frame in the frame B
  ///                 of `body`.
  FixedOffsetFrame(const Body<T>& P, const Isometry3<T>& X_PF);

  void Compile(const MultibodyTree<T>& tree) final {
    topology_ = tree.get_topology().get_frame(this->get_index());
    DRAKE_ASSERT(topology_.index == this->get_index());
  }

 private:
  // The fixed pose of this frame F measured and expressed in another physical
  // frame P.
  Isometry3<T> X_PF_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
