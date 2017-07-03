#pragma once

#include <map>
#include <string>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Keeps a set of frames and the transforms that relate them.
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
template <typename T>
class PoseTree {
 public:
  /// Constructor that takes the @p root_frame of the tree.
  explicit PoseTree(const std::string& root_frame);

  /// Returns the name of the root frame.
  inline std::string RootFrame() const { return root_frame_; }

  /// Updates the tree with the given @p source_frame
  /// @p pose in the @p target_frame (X_TS).
  /// @throw std::runtime_error if the @p target_frame is not
  /// found in the tree.
  void Update(const std::string& target_frame,
              const std::string& source_frame,
              const Isometry3<T> pose);

  /// Returns the @p source_frame pose in the @p target_frame (X_TS).
  /// @throw std::runtime_error if either the @p target_frame
  /// or the @p source_frame are not found in the tree.
  Isometry3<T> Transform(const std::string& target_frame,
                         const std::string& source_frame) const;

 private:
  /// Name of the root frame of this tree.
  const std::string root_frame_;
  // Map to keep all known frames transforms relative
  // to the root frame.
  std::map<std::string, Isometry3<T>> rooted_map_;
};

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
