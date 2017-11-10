#include "drake/multibody/parsing/frame_cache.h"

#include <algorithm>

namespace drake {
namespace multibody {
namespace parsing {

template<typename T>
FrameCache<T>::FrameCache(const std::string& root_frame)
    : root_frame_(root_frame) {}

template<typename T>
bool FrameCache<T>::IsFrameKnown(const std::string& frame) const {
  return X_TS_cache_.find(frame) != X_TS_cache_.end() || frame == root_frame_;
}

template<typename T>
void FrameCache<T>::Update(
    const std::string& target_frame,
    const std::string& source_frame,
    const Isometry3<T>& X_TS) {
  // Make sure target frame exists.
  DRAKE_THROW_UNLESS(IsFrameKnown(target_frame));
  // Make sure source frame is not root frame.
  DRAKE_THROW_UNLESS(source_frame != root_frame_);
  // Check for trivial same frame case.
  if (target_frame == source_frame) {
    DRAKE_THROW_UNLESS(X_TS.isApprox(Isometry3<T>::Identity()));
    return;
  }
  if (IsFrameKnown(source_frame)) {
    // Ensure no cycle is introduced by this update.
    std::string frame = target_frame;
    while (frame != root_frame_) {
      const FramedIsometry3<T>& entry = X_TS_cache_.at(frame);
      DRAKE_THROW_UNLESS(entry.frame != source_frame);
      frame = entry.frame;
    }
  }
  // Update map with source frame's (S) pose in the
  // the target frame (T).
  X_TS_cache_[source_frame].isometry = X_TS;
  X_TS_cache_[source_frame].frame = target_frame;
}

template<typename T>
Isometry3<T> FrameCache<T>::Transform(
    const std::string& target_frame,
    const std::string& source_frame) const {
  // Make sure both target and source frames exist.
  DRAKE_THROW_UNLESS(IsFrameKnown(target_frame));
  DRAKE_THROW_UNLESS(IsFrameKnown(source_frame));
  // Check for trivial same frame case.
  if (target_frame == source_frame) {
    return Isometry3<T>::Identity();
  }
  // Compute source frame's (S) pose in the target frame (T)
  // using both frames's poses in the root frame (R).
  // That is, X_TS = X_RT^-1 * X_RS.
  Isometry3<T> X_RT = RootTransform(target_frame);
  Isometry3<T> X_RS = RootTransform(source_frame);
  return X_RT.inverse() * X_RS;
}

template <typename T>
Isometry3<T> FrameCache<T>::RootTransform(std::string frame) const {
  // Compute root frame's (R) pose in the given frame (F)
  // as it's easier given the cache structure, and then invert
  // it. That is, X_RF = X_FR^-1
  Isometry3<T> X_FR = Isometry3<T>::Identity();
  while (frame != root_frame_) {
    const FramedIsometry3<T>& entry = X_TS_cache_.at(frame);
    X_FR = X_FR * entry.isometry.inverse();
    frame = entry.frame;
  }
  return X_FR.inverse();
}

template class FrameCache<double>;
template struct FramedIsometry3<double>;

}  // namespace parsing
}  // namespace multibody
}  // namespace drake
