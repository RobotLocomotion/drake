#include "drake/geometry/meshcat_recording_internal.h"

#include <vector>

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
