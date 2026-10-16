#include "drake/multibody/meshcat/point_contact_visualizer.h"

#include <string>
#include <unordered_map>
#include <utility>

#include <fmt/format.h>

#include "drake/common/unused.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace multibody {
namespace meshcat {
namespace internal {

using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::Cylinder;
using geometry::Meshcat;
using geometry::MeshcatCone;
using math::RigidTransformd;
using math::RotationMatrixd;

PointContactVisualizer::PointContactVisualizer(std::shared_ptr<Meshcat> meshcat,
                                               ContactVisualizerParams params)
    : meshcat_(std::move(meshcat)), params_(std::move(params)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
}

PointContactVisualizer::~PointContactVisualizer() = default;

void PointContactVisualizer::Delete() {
  meshcat_->Delete(params_.prefix);
  path_visibility_status_.clear();
}

void PointContactVisualizer::Update(
    double time, const std::vector<PointContactVisualizerItem>& items) {
  // Set all contacts to be inactive. They will be re-activated as we loop over
  // `items`, below. Anything that is not re-activated will be set to invisible
  // in a final clean-up pass at the end.
  for (auto& [path, status] : path_visibility_status_) {
    unused(path);
    status.active = false;
  }

  // Process the new contacts to find the active ones. A body pair can have
  // more than one contact at a time (a body with several collision geometries,
  // or a multi-point contact manifold), so the repeats are numbered and each
  // contact gets its own meshcat path instead of overdrawing the first one.
  std::unordered_map<std::string, int> pair_counts;
  for (const PointContactVisualizerItem& item : items) {
    // Find our meshcat state for this contact.
    std::string path =
        fmt::format("{}/{}+{}", params_.prefix, item.body_A, item.body_B);
    const int repeat = pair_counts[path]++;
    if (repeat > 0) {
      // The separator is '#' rather than '+' so that a numbered path can never
      // coincide with the base path of some other body pair. (Body names that
      // contain '+' could already make two pairs share a base path; that is a
      // pre-existing limitation of the naming scheme.)
      path = fmt::format("{}#{}", path, repeat);
    }
    VisibilityStatus& status = FindOrAdd(path);

    // Decide whether the contact should be shown.
    const double force_norm = item.contact_force.norm();
    status.active = (force_norm >= params_.force_threshold);
    if (!status.active) {
      continue;
    }

    // Update this active contact's transforms.
    // Position the center.
    meshcat_->SetTransform(path,
                           RigidTransformd(RotationMatrixd::MakeFromOneVector(
                                               item.contact_force, 2),
                                           item.contact_point),
                           time);
    // Stretch the cylinder in z.
    const double height = force_norm / params_.newtons_per_meter;
    // Note: Meshcat does not fully support non-uniform scaling (see #18095).
    // We get away with it here since there is no rotation on this frame and no
    // children in the kinematic tree.
    meshcat_->SetProperty(path + "/cylinder", "scale", {1, 1, height}, time);
    // Translate the arrowheads.
    const double arrowhead_height = params_.radius * 2.0;
    meshcat_->SetTransform(
        path + "/head",
        RigidTransformd(Vector3d{0, 0, -height - arrowhead_height}), time);
    meshcat_->SetTransform(
        path + "/tail",
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                        Vector3d{0, 0, height + arrowhead_height}),
        time);
  }

  // Update meshcat visibility to match the active status.
  for (auto& [path, status] : path_visibility_status_) {
    if (status.visible != status.active) {
      meshcat_->SetProperty(path, "visible", status.active, time);
      status.visible = status.active;
    }
  }
}

PointContactVisualizer::VisibilityStatus& PointContactVisualizer::FindOrAdd(
    const std::string& path) {
  auto iter = path_visibility_status_.find(path);
  if (iter != path_visibility_status_.end()) {
    return iter->second;
  }

  // Start with it invisible, to prevent flickering at the origin.
  iter = path_visibility_status_.insert({path, {false, false}}).first;
  meshcat_->SetProperty(path, "visible", false, 0);

  // Add the geometry to meshcat. The height of the cylinder is 2 and gets
  // scaled to twice the contact force length because we draw both (equal
  // and opposite) forces.
  const Cylinder cylinder(params_.radius, 2.0);
  meshcat_->SetObject(path + "/cylinder", cylinder, params_.color);
  const double arrowhead_height = params_.radius * 2.0;
  const double arrowhead_width = params_.radius * 2.0;
  const MeshcatCone arrowhead(arrowhead_height, arrowhead_width,
                              arrowhead_width);
  meshcat_->SetObject(path + "/head", arrowhead, params_.color);
  meshcat_->SetObject(path + "/tail", arrowhead, params_.color);

  return iter->second;
}

}  // namespace internal
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
