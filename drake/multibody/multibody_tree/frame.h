#pragma once

#include "drake/multibody/multibody_tree/frame_base.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// The %Frame class represents a frame whose state depends on the state of a
/// single body and, therefore, moves with the body. The nature of the frame's
/// state's dependency on the body can vary significantly:
///   - the state of a body frame (also referred to as reference frames in some
///     flexible body formulations from continuum mechanics) depends solely on
///     the state of the body,
///   - the state of a frame attached to a body with a fixed pose, depends on
///     the state of the body's frame and its fixed relative pose, and
///   - the state of a frame attached to specific material points on a flexible
///     body would depend on the state of the body's frame and the state of
///     deformation of the body.
/// Like the FrameBase class, %Frame does not store the pose of a frame but it
/// only represents the concept of a frame moving with a body.
/// Specific physical frame classes inheriting from %Frame will
/// typically provide methods to access or compute the pose of the frame
/// instance they represent measured and expressed in specific frames as a
/// function of the of the state of the associated Body.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Frame : public FrameBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Frame)

  /// Returns a constant reference to the body associated to this frame.
  const Body<T>& get_body() const {
    return body_;
  }

 protected:
  // Only derived classes can use this constructor.
  explicit Frame(const Body<T>& body) : body_(body) {}

 protected:
  // Implementation for MultibodyTreeElement::DoCompile().
  void DoCompile(const MultibodyTree<T>& tree) final {
    topology_ = tree.get_topology().get_frame(this->get_index());
    DRAKE_ASSERT(topology_.index == this->get_index());
  }

 private:
  // The body associated with this frame.
  const Body<T>& body_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
