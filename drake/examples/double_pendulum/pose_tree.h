#pragma once

#include <map>
#include <string>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Keeps a set of frames and the transforms that relate
/// them.
///
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
template <typename T>
class PoseTree {
 public:
  /// Constructor that takes the @p root_frame of the tree.
  explicit PoseTree(const std::string& root_frame)
      : root_frame_(root_frame) {
    // Root frame transform to itself is the identity.
    rooted_map_[root_frame_] = Isometry3<T>::Identity();
  }

  /// Returns the name of the root frame.
  inline std::string RootFrame() const { return root_frame_; }

  /// Updates the tree with the given @p source_frame
  /// @p transform relative to the given @p target_frame,
  /// @throw std::runtime_error if the target_frame is not
  /// found in the tree.
  void Update(const std::string& target_frame,
              const std::string& source_frame,
              const Isometry3<T> transform) {
    // Make sure target (origin) frame exists.
    DRAKE_THROW_UNLESS(
        rooted_map_.find(target_frame)
        != rooted_map_.end());
    // Update source (point) frame.
    rooted_map_[source_frame] = (
        rooted_map_.at(target_frame) * transform);
  }

  /// Returns the @p transform of the @p source_frame
  /// relative to the @p target_frame.
  /// @throw std::runtime_error if either the target_frame
  /// or the source_frame are not found in the tree.
  Isometry3<T> Transform(const std::string& target_frame,
                         const std::string& source_frame) const {
    // Make sure both target (origin) and source (point)
    // frame exist.
    DRAKE_THROW_UNLESS(
        rooted_map_.find(target_frame)
        != rooted_map_.end());
    DRAKE_THROW_UNLESS(
        rooted_map_.find(source_frame)
        != rooted_map_.end());
    // Return transform of source relative to target.
    return (rooted_map_.at(target_frame).inverse()
            * rooted_map_.at(source_frame));
  }

 private:
  /// Name of the root frame of this tree.
  std::string root_frame_;
  // Map to keep all known frames transforms relative
  // to the root frame.
  std::map<std::string, Isometry3<T>> rooted_map_;
};

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
