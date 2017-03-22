#pragma once

#include "drake/multibody/multibody_tree/multibody_tree_element.h"
#include "drake/multibody/multibody_tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

/// %Frame is an abstract base class representation of the concept of a frame
/// in multibody dynamics. Frames can be thought of as a set of three orthogonal
/// axes forming a right-handed orthogonal basis located at a point called the
/// frame's origin.
/// The concept of the _pose_ `X_EF` of a frame F only makes sense when measured
/// and expressed in another frame E. This class does not store the pose `X_EF`
/// of a frame but it only represents the frame itself.
/// Specific frame classes inheriting from %Frame will typically provide methods
/// to access or compute the pose of the frame instance they represent measured
/// and expressed in specific frames as a function of the state of the parent
/// MultibodyTree.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
template <typename T>
class Frame : public MultibodyTreeElement<Frame<T>, FrameIndex> {};

}  // namespace multibody
}  // namespace drake
