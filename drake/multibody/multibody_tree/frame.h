#pragma once

#include "drake/multibody/multibody_tree/frame_base.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"
#include "drake/multibody/multibody_tree/multibody_tree_topology.h"

namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// %Frame is an abstract class representing a _material frame_ (also called a
/// _physical frame_), meaning that it is associated with a material point of a
/// Body. A material frame can be used to apply forces and torques to a
/// multibody system, and can be used as an attachment point for force-producing
/// elements like joints, actuators, and constraints. Despite its name, %Frame
/// is not the most general frame representation in Drake; see FrameBase for a
/// more-general discussion.
///
/// The pose and motion of a %Frame object is always calculated relative to the
/// BodyFrame of the body to which it is associated, and every %Frame object can
/// report which Body object that is. Concrete classes derived from %Frame
/// differ only in how those kinematic properties are calculated. For soft
/// bodies that calculation may depend on the body's deformation states.
/// A %Frame on a rigid body will usually have a fixed offset from its
/// BodyFrame, but that is not required -- a frame that moves with respect to
/// its BodyFrame can still be a material frame on that rigid body. A contact
/// frame is a typical example.
///
/// As always in Drake, runtime numerical quantities are stored in a Context.
/// A %Frame object provides methods useful for extracting relevant properties
/// from a given Context, but does not directly store runtime values.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Frame : public FrameBase<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Frame)

  /// Returns a const reference to the body associated to this %Frame.
  const Body<T>& get_body() const {
    return body_;
  }

 protected:
  // Only derived classes can use this constructor.
  explicit Frame(const Body<T>& body) : body_(body) {}

 private:
  // Implementation for MultibodyTreeElement::DoCompile().
  void DoCompile(const MultibodyTree<T>& tree) final {
    topology_ = tree.get_topology().get_frame(this->get_index());
    DRAKE_ASSERT(topology_.index == this->get_index());
  }

  // The body associated with this frame.
  const Body<T>& body_;

  // The internal bookkeeping topology struct used by MultibodyTree.
  FrameTopology topology_;
};

}  // namespace multibody
}  // namespace drake
