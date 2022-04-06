#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"

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

HydroelasticContactVisualizer::HydroelasticContactVisualizer(
    std::shared_ptr<Meshcat> meshcat, ContactVisualizerParams params)
    : meshcat_(std::move(meshcat)), params_(std::move(params)) {
  DRAKE_DEMAND(meshcat_ != nullptr);
}

HydroelasticContactVisualizer::~HydroelasticContactVisualizer() = default;

void HydroelasticContactVisualizer::Delete() {
  meshcat_->Delete(params_.prefix);
  path_visibility_status_.clear();
}

void HydroelasticContactVisualizer::Update(
    const std::vector<HydroelasticContactVisualizerItem>& items) {
  // Set all contacts to be inactive. They will be re-activated as we loop over
  // `items`, below. Anything that is not re-activated will be set to invisible
  // in a final clean-up pass at the end.
  for (auto& [path, status] : path_visibility_status_) {
    unused(path);
    status.active = false;
  }

  // Process the new contacts to find the active ones.
  for (const HydroelasticContactVisualizerItem& item : items) {
    // Find our meshcat state for this contact pair.
    const std::string path =
        fmt::format("{}/{}+{}", params_.prefix, item.body_A, item.body_B);

    VisibilityStatus& status = FindOrAdd(path);

    // Decide whether the contact should be shown.
    const double force_norm = item.force_C_W.norm();
    const double moment_norm = item.moment_C_W.norm();
    status.active = (force_norm >= params_.force_threshold) ||
                    (moment_norm >= params_.force_threshold);
    if (!status.active) {
      continue;
    }

    // Update this active contact's transforms.
    // Position the centroid.
    meshcat_->SetTransform(path, RigidTransformd(item.centroid_W));

    // Force vector.
    {
      meshcat_->SetTransform(path + "/force_C_W",
                             RigidTransformd(RotationMatrixd::MakeFromOneVector(
                                 item.force_C_W, 2)));

      // Stretch the cylinder in z.
      const double height = force_norm / params_.newtons_per_meter;
      meshcat_->SetTransform(path + "/force_C_W/cylinder",
                             Matrix4d(Vector4d{1, 1, height, 1}.asDiagonal()));
      // Translate the arrowheads.
      const double arrowhead_height = params_.radius * 2.0;
      meshcat_->SetTransform(
          path + "/force_C_W/head",
          RigidTransformd(Vector3d{0, 0, -height - arrowhead_height}));
      meshcat_->SetTransform(
          path + "/force_C_W/tail",
          RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                          Vector3d{0, 0, height + arrowhead_height}));
    }
    // Moment vector.
    {
      meshcat_->SetTransform(path + "/moment_C_W",
                             RigidTransformd(RotationMatrixd::MakeFromOneVector(
                                 item.moment_C_W, 2)));

      // Stretch the cylinder in z.
      const double height = std::sqrt(moment_norm / params_.newtons_per_meter);
      meshcat_->SetTransform(path + "/moment_C_W/cylinder",
                             Matrix4d(Vector4d{1, 1, height, 1}.asDiagonal()));
      // Translate the arrowheads.
      const double arrowhead_height = params_.radius * 2.0;
      meshcat_->SetTransform(
          path + "/moment_C_W/head",
          RigidTransformd(Vector3d{0, 0, -height - arrowhead_height}));
      meshcat_->SetTransform(
          path + "/moment_C_W/tail",
          RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                          Vector3d{0, 0, height + arrowhead_height}));
    }
  }

  // Update meshcat visiblity to match the active status.
  for (auto& [path, status] : path_visibility_status_) {
    if (status.visible != status.active) {
      meshcat_->SetProperty(path, "visible", status.active);
      status.visible = status.active;
    }
  }
}

HydroelasticContactVisualizer::VisibilityStatus&
HydroelasticContactVisualizer::FindOrAdd(const std::string& path) {
  auto iter = path_visibility_status_.find(path);
  if (iter != path_visibility_status_.end()) {
    return iter->second;
  }

  // Start with it invisible, to prevent flickering at the origin.
  iter = path_visibility_status_.insert({path, {false, false}}).first;
  meshcat_->SetProperty(path, "visible", false);

  // Add the geometry to meshcat. The height of the cylinder is 2 and gets
  // scaled to twice the contact force length because we draw both (equal
  // and opposite) forces.
  const Cylinder cylinder(params_.radius, 2.0);
  const double arrowhead_height = params_.radius * 2.0;
  const double arrowhead_width = params_.radius * 2.0;
  const MeshcatCone arrowhead(arrowhead_height, arrowhead_width,
                              arrowhead_width);

  meshcat_->SetObject(path + "/force_C_W/cylinder", cylinder,
                      params_.hydro_force_color);
  meshcat_->SetObject(path + "/force_C_W/head", arrowhead,
                      params_.hydro_force_color);
  meshcat_->SetObject(path + "/force_C_W/tail", arrowhead,
                      params_.hydro_force_color);
  meshcat_->SetObject(path + "/moment_C_W/cylinder", cylinder,
                      params_.hydro_moment_color);
  meshcat_->SetObject(path + "/moment_C_W/head", arrowhead,
                      params_.hydro_moment_color);
  meshcat_->SetObject(path + "/moment_C_W/tail", arrowhead,
                      params_.hydro_moment_color);

  return iter->second;
}

}  // namespace internal
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
