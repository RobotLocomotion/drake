#pragma once

#include <map>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/parsing/sdf/framed_isometry3.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameCache);
  /// Constructor that takes the @p root_frame.
  explicit FrameCache(const std::string& root_frame);

  /// Returns the name of the root frame.
  const std::string& RootFrame() const { return root_frame_; }

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

  /// Returns true if the given @p frame is known
  /// to the cache, false otherwise.
  bool IsFrameKnown(const std::string& frame) const;

 private:
  // Returns `X_RF`, that is the pose of given @p frame `F` in
  // the root frame `R`.
  Isometry3<T> RootTransform(std::string frame) const;

  // Name of the root frame of this cache.
  std::string root_frame_;

  // Map to keep all known frames' transforms.
  std::map<std::string, FramedIsometry3<T>> X_TS_cache_;
};

}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
