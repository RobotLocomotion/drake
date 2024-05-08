#include "drake/geometry/meshcat_recording_internal.h"

#include <utility>

namespace drake {
namespace geometry {
namespace internal {

MeshcatRecording::MeshcatRecording()
    : animation_{std::make_unique<MeshcatAnimation>()} {}

MeshcatRecording::~MeshcatRecording() = default;

void MeshcatRecording::StartRecording(double frames_per_second,
                                      bool set_visualizations_while_recording) {
  animation_ = std::make_unique<MeshcatAnimation>(frames_per_second);
  recording_ = true;
  set_visualizations_while_recording_ = set_visualizations_while_recording;
  last_frames_.clear();
}

void MeshcatRecording::StopRecording() {
  recording_ = false;
}

void MeshcatRecording::DeleteRecording() {
  const double frames_per_second = animation_->frames_per_second();
  animation_ = std::make_unique<MeshcatAnimation>(frames_per_second);
}

template <typename T>
bool MeshcatRecording::SetProperty(std::string_view path,
                                   std::string_view property, const T& value,
                                   std::optional<double> time_in_recording) {
  const AnimationDetail detail = CalcDetail(time_in_recording);
  if (detail.frame.has_value()) {
    animation_->SetProperty(*detail.frame, path, property, value);
  }
  return detail.show_live;
}

template bool MeshcatRecording::SetProperty(std::string_view, std::string_view,
                                            const bool&, std::optional<double>);
template bool MeshcatRecording::SetProperty(std::string_view, std::string_view,
                                            const double&,
                                            std::optional<double>);
template bool MeshcatRecording::SetProperty(std::string_view, std::string_view,
                                            const std::vector<double>&,
                                            std::optional<double>);

bool MeshcatRecording::SetTransform(std::string_view path,
                                    const math::RigidTransformd& X_ParentPath,
                                    std::optional<double> time_in_recording) {
  const AnimationDetail detail = CalcDetail(time_in_recording);
  if (detail.frame.has_value()) {
    animation_->SetTransform(*detail.frame, path, X_ParentPath);
  }
  return detail.show_live;
}

MeshcatRecording::SetObjectDetail MeshcatRecording::SetObject(
    std::string_view path, std::optional<double> time_in_recording) {
  const AnimationDetail detail = CalcDetail(time_in_recording);
  if (!detail.frame.has_value()) {
    // This SetObject is NOT being recorded.
    return SetObjectDetail{.new_path = std::string(path)};
  }

  // This SetObject IS being recorded. Check if this path was already set in a
  // prior animation frame, and then update the last_frames_ bookkeeping to this
  // frame.
  const double time = *time_in_recording;
  const int frame = *detail.frame;
  auto last_frame_iter = last_frames_.find(path);
  const std::optional<int> last_frame =
      last_frame_iter != last_frames_.end()
          ? std::optional<int>(last_frame_iter->second)
          : std::nullopt;
  if (last_frame.has_value() && *last_frame > frame) {
    // XXX clean up error message
    throw std::runtime_error(
        "SetTriangleColorMesh with time_in_recording that corresponds to an "
        "earlier frame than the last frame.");
  }
  if (last_frame.has_value()) {
    last_frame_iter->second = frame;
  } else {
    last_frames_.emplace(path, frame);
  }

  // Compute the meshcat path for a timed SetObject.
  auto get_animation_path = [&path](int some_frame) {
    return fmt::format("{}/<animation>/{}", path, some_frame);
  };
  std::string animation_path = get_animation_path(frame);

  // Sequence calls to SetProperty to mask and unveil the object appropriately.
  SetObjectDetail result;
  if (!last_frame.has_value()) {
    // This is the first frame. Make sure the unanimated object is visible
    // only from the start time to `time_in_recording`. It is possible that
    // there was no unanimated object, which is ok because we can set property
    // without the object.
    std::string object_path = fmt::format("{}/<object>", path);
    result.properties.push_back(SetObjectDetail::Visible{
        .path = object_path,
        .time = animation_->start_time(),
        .visible = true,
    });
    result.properties.push_back(SetObjectDetail::Visible{
        .path = std::move(object_path),
        .time = time,
        .visible = false,
    });
  } else {
    // This is an update to a prior frame. Hide the prior object.
    result.properties.push_back(SetObjectDetail::Visible{
        .path = get_animation_path(*last_frame),
        .time = time,
        .visible = false,
    });
  }

  // This frame starts invisible and should unveil at `time`.
  result.properties.push_back(SetObjectDetail::Visible{
      .path = animation_path,
      .time = animation_->start_time(),
      .visible = false,
  });
  result.properties.push_back(SetObjectDetail::Visible{
      .path = std::move(animation_path),
      .time = time,
      .visible = true,
  });

  return result;
}

MeshcatRecording::AnimationDetail MeshcatRecording::CalcDetail(
    std::optional<double> time_in_recording) const {
  if (!(recording_ && time_in_recording.has_value())) {
    return AnimationDetail{};
  }
  const int frame = animation_->frame(*time_in_recording);
  return {.frame = frame, .show_live = set_visualizations_while_recording_};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
