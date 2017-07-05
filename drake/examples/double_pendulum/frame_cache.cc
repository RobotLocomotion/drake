#include "drake/examples/double_pendulum/frame_cache.h"

namespace drake {
namespace examples {
namespace double_pendulum {

template<typename T>
FrameCache<T>::FrameCache(const std::string& root_frame)
    : root_frame_(root_frame) {
  // Root frame pose is the identity.
  rooted_map_[root_frame_] = Isometry3<T>::Identity();
}

template<typename T>
void FrameCache<T>::Update(
    const std::string& target_frame,
    const std::string& source_frame,
    const Isometry3<T>& X_TS) {
  // Make sure target frame exists.
  DRAKE_THROW_UNLESS(rooted_map_.find(target_frame) != rooted_map_.end());
  // Update map with source frame's (S) pose in the
  // the root frame (R). That is X_RS = X_RT * X_TS.
  rooted_map_[source_frame] = rooted_map_.at(target_frame) * X_TS;
}

template<typename T>
Isometry3<T> FrameCache<T>::Transform(
    const std::string& target_frame,
    const std::string& source_frame) const {
  // Make sure both target and source frames exist.
  DRAKE_THROW_UNLESS(rooted_map_.find(target_frame) != rooted_map_.end());
  DRAKE_THROW_UNLESS(rooted_map_.find(source_frame) != rooted_map_.end());
  // Compute source frame's (S) pose in the target frame (T)
  // using the both frames's poses in the root frame (R).
  // That is, X_TS = X_RT^-1 * X_RS.
  return rooted_map_.at(target_frame).inverse() * rooted_map_.at(source_frame);
}

template class FrameCache<double>;

}  // namespace double_pendulum
}  // namespace examples
}  // namespace drake
