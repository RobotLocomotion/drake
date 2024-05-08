#pragma once

#include <memory>
#include <optional>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace internal {

/* Helper class used by Meshcat to implement all of the state machine logic for
creating recordings. For clarity, note the distinction between MeshcatAnimation
(the container object for the animation track data) and MeshcatRecording (this
state machine surrounding the animation). The Recording has an Animation as a
member field. */
class MeshcatRecording {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatRecording);

  MeshcatRecording();

  ~MeshcatRecording();

  /* The implementation of the same-named function on Meshcat. Refer to that
  public API for details. */
  void StartRecording(double frames_per_second,
                      bool set_visualizations_while_recording);

  /* The implementation of the same-named function on Meshcat. Refer to that
  public API for details. */
  void StopRecording();

  /* The implementation of the same-named function on Meshcat. Refer to that
  public API for details. */
  void DeleteRecording();

  /* The return value is invalidated by StartRecording or DeleteRecording. */
  const MeshcatAnimation& get_animation() const { return *animation_; }

  /* The return value is invalidated by StartRecording or DeleteRecording. */
  MeshcatAnimation& get_mutable_animation() { return *animation_; }

  /* Conditionally adds this property to the current animation, and decides
  whether or not to show this property in the live Meshcat session.

  If no recording is in progress, then does NOT animate and returns true.
  If time_in_recording is missing, then does NOT animate and returns true.

  If recording is in progress and time_in_recording is present, then DOES
  animate and returns `set_visualizations_while_recording`.

  @retval show_live Whether Meshcat should set this property in the live
  session.

  @tparam T One of `bool`, `double`, or `vector<double>` */
  template <typename T>
  bool SetProperty(std::string_view path, std::string_view property,
                   const T& value, std::optional<double> time_in_recording);

  /* Like SetProperty (with all of the same animation decisions), but for a
  RigidTransformd. */
  bool SetTransform(std::string_view path,
                    const math::RigidTransformd& X_ParentPath,
                    std::optional<double> time_in_recording);

 private:
  struct AnimationDetail {
    // A frame number for use with MeshcatAnimation, or nullopt if this event
    // should not be animated.
    std::optional<int> frame;

    // Whether or not this event should be published to Meshcat. Might be false
    // when set_visualizations_while_recording is true and other conditions are
    // also met.
    bool show_live{true};
  };

  /* Given a time_in_recording and taking into account our state machine status,
  returns the animation details. */
  AnimationDetail CalcDetail(std::optional<double> time_in_recording) const;

  /* The current animation (may be empty, but is never nullptr). */
  std::unique_ptr<MeshcatAnimation> animation_;
  bool recording_{false};
  bool set_visualizations_while_recording_{false};
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
