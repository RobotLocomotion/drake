#include "drake/examples/double_pendulum/frame_cache.h"

#include <algorithm>

namespace drake {
namespace examples {
namespace double_pendulum {

template<typename T>
FrameCache<T>::FrameCache(const std::string& root_frame)
    : root_frame_(root_frame) {}

template<typename T>
void FrameCache<T>::Update(
    const std::string& target_frame,
    const std::string& source_frame,
    const Isometry3<T>& X_TS) {
  // Make sure target frame exists.
  DRAKE_THROW_UNLESS(
      target_frame == root_frame_ || cache_.find(target_frame) != cache_.end());
  // Update map with source frame's (S) pose in the
  // the root frame (R). That is X_RS = X_RT * X_TS.
  cache_[source_frame].isometry = X_TS;
  cache_[source_frame].frame = target_frame;
}

template<typename T>
Isometry3<T> FrameCache<T>::Transform(
    const std::string& target_frame,
    const std::string& source_frame) const {
  // Make sure both target and source frames exist.
  DRAKE_THROW_UNLESS(
      target_frame == root_frame_ || cache_.find(target_frame) != cache_.end());
  DRAKE_THROW_UNLESS(
      source_frame == root_frame_ || cache_.find(source_frame) != cache_.end());
  // Check for trivial cases.
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
    const FramedIsometry3<T>& entry = cache_.at(frame);
    X_FR = X_FR * entry.isometry.inverse();
    frame = entry.frame;
  }
  return X_FR.inverse();
}

template class FrameCache<double>;
template struct FramedIsometry3<double>;

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
