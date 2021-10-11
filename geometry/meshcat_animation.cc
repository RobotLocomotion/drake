#include "drake/geometry/meshcat_animation.h"

namespace drake {
namespace geometry {

MeshcatAnimation::MeshcatAnimation(double frames_per_second)
    : frames_per_second_(frames_per_second) {}

MeshcatAnimation::~MeshcatAnimation() = default;

void MeshcatAnimation::SetTransform(int frame, const std::string& path,
                                    const math::RigidTransformd& X_ParentPath) {
  std::vector<double> position(3);
  Eigen::Vector3d::Map(&position[0]) = X_ParentPath.translation();
  const auto q = X_ParentPath.rotation().ToQuaternion();
  std::vector<double> quaternion{q.x(), q.y(), q.z(), q.w()};
  SetProperty(frame, path, "position", "vector3", position);
  SetProperty(frame, path, "quaternion", "quaternion", quaternion);
}

void MeshcatAnimation::SetProperty(int frame, const std::string& path,
                  const std::string& property, bool value) {
  SetProperty(frame, path, property, "boolean", value);
}

void MeshcatAnimation::SetProperty(int frame, const std::string& path,
                  const std::string& property, double value) {
  SetProperty(frame, path, property, "number", value);
}

void MeshcatAnimation::SetProperty(int frame, const std::string& path,
                  const std::string& property,
                  const std::vector<double>& value) {
  SetProperty(frame, path, property, "vector", value);
}

std::string MeshcatAnimation::get_javascript_type(
    const std::string& path, const std::string& property) const {
  if (path_tracks_.find(path) == path_tracks_.end() ||
      path_tracks_.at(path).find(property) == path_tracks_.at(path).end()) {
    return "";
  }
  return path_tracks_.at(path).at(property).js_type;
}

template <typename T>
void MeshcatAnimation::SetProperty(int frame, const std::string& path,
                                   const std::string& property,
                                   const std::string& js_type, const T& value) {
  TypedTrack& tt = path_tracks_[path][property];
  if (std::holds_alternative<std::monostate>(tt.track)) {
    tt.track = Track<T>();
    tt.js_type = js_type;
  } else if (tt.js_type != js_type) {
    throw std::runtime_error(fmt::format(
        "{} property {} already has a track with javascript type {} != {}",
        path, property, tt.js_type, js_type));
  }
  // get<T> will also throw bad_variant_access if the types don't match.
  std::get<Track<T>>(tt.track)[frame] = value;
}

}  // namespace geometry
}  // namespace drake
