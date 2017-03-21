#pragma once

#include "drake/multibody/multibody_tree/frame.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"


namespace drake {
namespace multibody {

// Forward declarations.
template<typename T> class Body;

/// %MaterialFrame is an abstract base class representation for the concept of
/// a frame associated with a physical body.
/// Like the Frame class, %MaterialFrame does not store the pose of a frame but
/// it only represents the frame itself.
/// Specific material frame classes inheriting from %Frame will typically
/// provide methods to access or compute the pose of the frame instance they
/// represent measured and expressed in specific frames as a function of the
/// state of the parent MultibodyTree.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class MaterialFrame : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MaterialFrame)

  /// Returns the unique BodyIndex of the body associated with this frame.
  BodyIndex get_body_index() const { return body_index_;}

  /// Returns a constant reference to the body associated to this frame.
  const Body<T>& get_body() const {
    return this->get_parent_tree().get_body(get_body_index());
  }

 protected:
  // Only derived classes can use this constructor.
  explicit MaterialFrame(BodyIndex body_index) : body_index_(body_index) {}

 private:
  // The unique index in the parent multibody tree of the body associated with
  // this frame.
  const BodyIndex body_index_;
};

}  // namespace multibody
}  // namespace drake
