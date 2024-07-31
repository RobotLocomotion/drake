#include "drake/geometry/meshcat_animation.h"

namespace drake {
namespace geometry {

MeshcatAnimation::MeshcatAnimation(double frames_per_second)
    : frames_per_second_(frames_per_second) {}

MeshcatAnimation::~MeshcatAnimation() = default;

void MeshcatAnimation::SetTransform(int frame, std::string_view path,
                                    const math::RigidTransformd& X_ParentPath) {
  std::vector<double> position(3);
  Eigen::Vector3d::Map(&position[0]) = X_ParentPath.translation();
  const auto q = X_ParentPath.rotation().ToQuaternion();
  std::vector<double> quaternion{q.x(), q.y(), q.z(), q.w()};
  SetProperty(frame, path, "position", "vector3", position);
  SetProperty(frame, path, "quaternion", "quaternion", quaternion);
}

void MeshcatAnimation::SetProperty(int frame, std::string_view path,
                                   std::string_view property, bool value) {
  SetProperty(frame, path, property, "boolean", value);
}

void MeshcatAnimation::SetProperty(int frame, std::string_view path,
                                   std::string_view property, double value) {
  SetProperty(frame, path, property, "number", value);
}

void MeshcatAnimation::SetProperty(int frame, std::string_view path,
                                   std::string_view property,
                                   const std::vector<double>& value) {
  SetProperty(frame, path, property, "vector", value);
}

const MeshcatAnimation::TypedTrack* MeshcatAnimation::GetTypedTrack(
    std::string_view path, std::string_view property) const {
  const auto path_iter = path_tracks_.find(path);
  if (path_iter == path_tracks_.end()) {
    return nullptr;
  }
  const PropertyTracks& property_tracks = path_iter->second;
  const auto prop_iter = property_tracks.find(property);
  if (prop_iter == property_tracks.end()) {
    return nullptr;
  }
  return &prop_iter->second;
}

MeshcatAnimation::TypedTrack& MeshcatAnimation::GetOrCreateTypedTrack(
    std::string_view path, std::string_view property) {
  PropertyTracks& property_tracks =
      path_tracks_.emplace(path, PropertyTracks{}).first->second;
  TypedTrack& typed_track =
      property_tracks.emplace(property, TypedTrack{}).first->second;
  return typed_track;
}

template <typename T>
std::optional<T> MeshcatAnimation::get_key_frame(
    int frame, std::string_view path, std::string_view property) const {
  const TypedTrack* const typed_track = GetTypedTrack(path, property);
  if (typed_track != nullptr) {
    const auto* const track = std::get_if<Track<T>>(&typed_track->track);
    if (track != nullptr) {
      const auto frame_iter = track->find(frame);
      if (frame_iter != track->end()) {
        return frame_iter->second;
      }
    }
  }
  return std::nullopt;
}

template std::optional<bool> MeshcatAnimation::get_key_frame(
    int, std::string_view, std::string_view) const;
template std::optional<double> MeshcatAnimation::get_key_frame(
    int, std::string_view, std::string_view) const;
template std::optional<std::vector<double>> MeshcatAnimation::get_key_frame(
    int, std::string_view, std::string_view) const;

std::string MeshcatAnimation::get_javascript_type(
    std::string_view path, std::string_view property) const {
  const TypedTrack* const typed_track = GetTypedTrack(path, property);
  return (typed_track == nullptr) ? std::string{} : typed_track->js_type;
}

template <typename T>
void MeshcatAnimation::SetProperty(int frame, std::string_view path,
                                   std::string_view property,
                                   std::string_view js_type, const T& value) {
  TypedTrack& typed_track = GetOrCreateTypedTrack(path, property);
  if (std::holds_alternative<std::monostate>(typed_track.track)) {
    typed_track.track = Track<T>();
    typed_track.js_type = js_type;
  } else if (typed_track.js_type != js_type) {
    throw std::runtime_error(fmt::format(
        "{} property {} already has a track with javascript type {} != {}",
        path, property, typed_track.js_type, js_type));
  }
  // get<T> will also throw bad_variant_access if the types don't match.
  std::get<Track<T>>(typed_track.track)[frame] = value;
}

}  // namespace geometry
}  // namespace drake
