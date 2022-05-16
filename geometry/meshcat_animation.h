#pragma once

#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// Forward declaration.
class Meshcat;

/** An interface for recording/playback animations in Meshcat. Use
Meshcat::SetAnimation to publish a MeshcatAnimation to the visualizer.

Currently, an animation consists of (only) transforms and properties that are
set at a particular integer frame number. Although we do not support calls to
SetObject/Delete in an animation, you can consider using `SetProperty(frame,
path, "visible", true/false)` in your animation to make the object appear or
disappear at a particular frame.
*/
class MeshcatAnimation {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatAnimation)

  /** Constructs the animation object.
  @param frames_per_second a positive integer specifying the timing at which the
  frames are played back. */
  explicit MeshcatAnimation(double frames_per_second = 32.0);

  ~MeshcatAnimation();

  /** Returns the frame rate at which the animation will be played back. */
  double frames_per_second() const { return frames_per_second_; }

  /** Uses the frame rate to convert from time to the frame number, using
  std::floor.
  @pre `time` ≥ start_time().
  */
  int frame(double time) const {
    DRAKE_DEMAND(time >= start_time_);
    return static_cast<int>(
        std::floor((time - start_time_) * frames_per_second_));
  }

  // The documentation is adapted from
  // https://threejs.org/docs/index.html?q=Animation#api/en/animation/AnimationAction
  // The values are from
  // https://github.com/mrdoob/three.js/blob/dev/src/constants.js
  enum LoopMode {
    /** Plays the clip once. */
    kLoopOnce = 2200,

    /** Plays the clip with the chosen number of repetitions, each time jumping
    from the end of the clip directly to its beginning. */
    kLoopRepeat = 2201,

    /** Plays the clip with the chosen number of repetitions, alternately
    playing forward and backward. */
    kLoopPingPong = 2202
  };

  // Accessors.
  double start_time() const { return start_time_; }
  bool autoplay() const { return play_; }
  LoopMode loop_mode() const { return loop_mode_; }
  int repetitions() const { return repetitions_; }
  bool clamp_when_finished() const { return clamp_when_finished_; }

  /** Set the start time of the animation.  This is only for convenience; it is
  used in the frame() method to allow callers to look up the frame number based
  on the current time, the start time, and the frame rate.  It is not passed to
  Meshcat. It does not change any frames that have previously been set.  The
  default is zero.*/
  void set_start_time(double time) { start_time_ = time; }

  /** Set the behavior when the animation is first sent to the visualizer.  The
  animation will play immediately iff `play` is true.  The default is true.*/
  void set_autoplay(bool play) { play_ = play; }

  /** Sets the loop behavior on play.  @see LoopMode for details.  The default
  is kLoopRepeat. */
  void set_loop_mode(LoopMode mode) { loop_mode_ = mode; }

  /** Sets the number of repetitions of the animation each time it is played.
  This number has no effect when the loop mode is set to kLoopOnce.
  `repetitions` must be a positive integer.  The default value is 1. */
  void set_repetitions(int repetitions) {
    DRAKE_DEMAND(repetitions >= 1);
    repetitions_ = repetitions;
  }

  /** Sets the behavior at the end of the animation.  If true, then the
  animation will automatically be paused on its last frame.  If false, the
  scene will be reset to before the animation. The default is true.

  Note: This setting has no impact if the action is interrupted (it has
  only an effect if its last loop has really finished). */
  void set_clamp_when_finished(bool clamp) { clamp_when_finished_ = clamp; }

  /** Set the RigidTransform at `frame` in the animation for a given `path` in
  the the scene tree.  @see Meshcat::SetTransform.
  @param frame a non-negative integer indicating the frame at which this
               transform is applied.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path "Meshcat paths" for the semantics.
  @param X_ParentPath the relative transform from the path to its immediate
  parent.
  @throws std::exception if the position or quaternion properties of this
                         path have already been set to an incorrect type.

  */
  void SetTransform(int frame, const std::string& path,
                    const math::RigidTransformd& X_ParentPath);

  /** Sets a single named property of the object at the given `path` at the
  specified `frame` in the animation. @see Meshcat::SetProperty.
  @param frame a non-negative integer indicating the frame at which this
               transform is applied.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.
  @throws std::exception if this path/property has already been set with a
                         different type.

  @pydrake_mkdoc_identifier{bool}
  */
  void SetProperty(int frame, const std::string& path,
                   const std::string& property, bool value);

  /** Sets a single named property of the object at the given `path` at the
  specified `frame` in the animation. @see Meshcat::SetProperty.
  @param frame a non-negative integer indicating the frame at which this
               transform is applied.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.
  @throws std::exception if this path/property has already been set with a
                         different type.

  @pydrake_mkdoc_identifier{double}
  */
  void SetProperty(int frame, const std::string& path,
                   const std::string& property, double value);

  /** Sets a single named property of the object at the given `path` at the
  specified `frame` in the animation. @see Meshcat::SetProperty.
  @param frame a non-negative integer indicating the frame at which this
               transform is applied.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.
  @throws std::exception if this path/property has already been set with a
                         different type.

  @pydrake_mkdoc_identifier{vector_double}
  */
  void SetProperty(int frame, const std::string& path,
                   const std::string& property,
                   const std::vector<double>& value);

  // TODO(russt): Consider ColorKeyframeTrack.js and/or StringKeyframeTrack.js

  // TODO(russt): Possibly support interpolation modes and ending modes from
  // https://threejs.org/docs/#api/en/constants/Animation.

  // TODO(russt): Could implement a SetObject/Delete here that would effectively
  // implement behind the scenes the work-around I've documented in the class
  // documentation.  There would be some subtleties to make it robust (e.g.
  // setting multiple objects at different frames on the same path, but still
  // supporting properties, etc), but it could work.

  /** Returns the value information for a particular path/property at a
  particular frame if a value of type T has been set, otherwise returns
  std::nullopt. This method is intended primarily for testing. */
  template <typename T>
  std::optional<T> get_key_frame(int frame, const std::string& path,
                                 const std::string& property) const {
    if (path_tracks_.find(path) == path_tracks_.end() ||
        path_tracks_.at(path).find(property) == path_tracks_.at(path).end()) {
      return std::nullopt;
    }
    const TypedTrack& tt = path_tracks_.at(path).at(property);
    if (!std::holds_alternative<Track<T>>(tt.track)) {
      return std::nullopt;
    }
    const Track<T>& t = std::get<Track<T>>(tt.track);
    if (t.find(frame) == t.end()) {
      return std::nullopt;
    }
    return t.at(frame);
  }

  /** Returns the javascript type for a particular path/property, or the empty
  string if nothing has been set. This method is intended primarily for
  testing. */
  std::string get_javascript_type(const std::string& path,
                                  const std::string& property) const;

 private:
  // Implements the SetProperty methods.
  // js_type must match three.js getTrackTypeForValueTypeName implementation.
  template <typename T>
  void SetProperty(int frame, const std::string& path,
                   const std::string& property, const std::string& js_type,
                   const T& value);

  // A map of frame => value.
  template <typename T>
  using Track = std::map<int, T>;

  // All property values in a track must be the same type.
  struct TypedTrack {
    std::variant<std::monostate, Track<bool>, Track<double>,
                 Track<std::vector<double>>>
        track;
    std::string js_type;
  };

  // A map of property name => tracks.
  using PropertyTracks = std::map<std::string, TypedTrack>;

  // A map of path => property tracks.
  using PathTracks = std::map<std::string, PropertyTracks>;

  // TODO(russt): Narrow this access to restore encapsulation.
  friend class Meshcat;

  // A map of path name => property tracks.
  PathTracks path_tracks_{};

  const double frames_per_second_;
  double start_time_{0.0};
  bool play_{true};
  LoopMode loop_mode_{kLoopRepeat};
  int repetitions_{1};
  bool clamp_when_finished_{true};
};

}  // namespace geometry
}  // namespace drake
