#pragma once

#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"


namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// A %PhysicalFrame represents a Frame whose pose depends on a physical body.
/// Because this dependence can vary in interesting ways, this class is an
/// abstract class. These frames move with physical bodies. Examples of this
/// kind of frames are body frames (also referred to as _reference frames_ in
/// some flexible body formulations), frames attached to a body with a fixed
/// posed in that body frame (for instance to define the inboard frame for a
/// joint that connects it to another body) and frames attached to specific
/// material points or sections of a flexible body.
/// Like the Frame class, %PhysicalFrame is does not store the pose of a
/// frame but it only represents the concept of a frame coupled with a body.
/// Specific physical frame classes inheriting from %PhysicalFrame will
/// typically provide methods to access or compute the pose of the frame
/// instance they represent measured and expressed in specific frames as a
/// function of the state of the MultibodyTree to which the frame belongs.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class PhysicalFrame : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhysicalFrame)

  /// Returns the unique BodyIndex of the body associated with this frame.
  BodyIndex get_body_index() const { return body_index_;}

  /// Returns a constant reference to the body associated to this frame.
  const Body<T>& get_body() const {
    return this->get_parent_tree().get_body(get_body_index());
  }

 protected:
  // Only derived classes can use this constructor.
  explicit PhysicalFrame(BodyIndex body_index) : body_index_(body_index) {}

 private:
  // The unique index in the parent multibody tree of the body associated with
  // this frame.
  const BodyIndex body_index_;
};

}  // namespace multibody
}  // namespace drake
