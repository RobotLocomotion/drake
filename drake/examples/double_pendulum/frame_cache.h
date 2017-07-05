#pragma once

#include <map>
#include <string>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace double_pendulum {

/// Keeps a set of frames and the transforms that relate them, using
/// a root or fixed frame to conjoin them all.
///
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
template <typename T>
class FrameCache {
 public:
  /// Constructor that takes the @p root_frame.
  explicit FrameCache(const std::string& root_frame);

  /// Returns the name of the root frame.
  inline const std::string& RootFrame() const { return root_frame_; }

  /// Adds or updates the given @p source_frame within the cache.
  /// @param[in] target_frame the name of the target frame `T`. This
  /// frame \b must already exist in the cache.
  /// @param[in] source_frame the name of the source frame `S`. If it
  /// already exists in the cache, its transform will be updated.
  /// @param[in] X_TS the source frame `S`'s pose in the target frame.`T`.
  /// @throw std::runtime_error if the target frame `T` is not in the cache.
  void Update(const std::string& target_frame,
              const std::string& source_frame,
              const Isometry3<T>& X_TS);

  /// Returns `X_TS`, that is the pose of the @p source_frame `S`
  /// in the @p target_frame `T`.
  /// @throw std::runtime_error if either the target frame `T`
  /// or the source frame `S` are not in the cache.
  Isometry3<T> Transform(const std::string& target_frame,
                         const std::string& source_frame) const;

 private:
  // Name of the root frame of this cache.
  const std::string root_frame_;
  // Map to keep all known frames' transforms relative to the root frame.
  std::map<std::string, Isometry3<T>> rooted_map_;
};

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
