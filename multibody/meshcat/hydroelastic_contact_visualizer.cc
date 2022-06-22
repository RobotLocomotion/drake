#include "drake/multibody/meshcat/hydroelastic_contact_visualizer.h"

#include <utility>
#include <algorithm>

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
                    (moment_norm >= params_.moment_threshold);
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
      // clang-format off
      meshcat_->SetTransform(path + "/force_C_W/cylinder",
          (Matrix4d() <<  1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, height, 0.5*height,
                          0, 0, 0, 1).finished());
      // clang-format on
      // Translate the arrowheads.
      const double arrowhead_height = params_.radius * 2.0;
      meshcat_->SetTransform(
          path + "/force_C_W/head",
          RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                          Vector3d{0, 0, height + arrowhead_height}));
    }
    // Moment vector.
    {
      meshcat_->SetTransform(path + "/moment_C_W",
                             RigidTransformd(RotationMatrixd::MakeFromOneVector(
                                 item.moment_C_W, 2)));

      // Stretch the cylinder in z.
      const double height = moment_norm / params_.newton_meters_per_meter;
      // clang-format off
      meshcat_->SetTransform(path + "/moment_C_W/cylinder",
          (Matrix4d() <<  1, 0, 0, 0,
                          0, 1, 0, 0,
                          0, 0, height, 0.5*height,
                          0, 0, 0, 1).finished());
      // clang-format on
      // Translate the arrowheads.
      const double arrowhead_height = params_.radius * 2.0;
      meshcat_->SetTransform(
          path + "/moment_C_W/head",
          RigidTransformd(RotationMatrixd::MakeXRotation(M_PI),
                          Vector3d{0, 0, height + arrowhead_height}));
    }

    // Contact surface
    {
      // Flame map
      double min_pressure = item.pressure.minCoeff();
      double max_pressure = item.pressure.maxCoeff();

      Eigen::Matrix3Xd colors(3, item.pressure.size());
      for (int i = 0; i < item.pressure.size(); ++i) {
        const double norm_pressure =
            (item.pressure[i] - min_pressure) / (max_pressure - min_pressure);

        colors(0, i) = std::clamp(((norm_pressure - 0.25) * 4.0), 0.0, 1.0);
        colors(1, i) = std::clamp(((norm_pressure - 0.5) * 4.0), 0.0, 1.0);
        if (norm_pressure < 0.25) {
          colors(2, i) = std::clamp(norm_pressure * 4.0, 0.0, 1.0);
        } else if (norm_pressure > 0.75) {
          colors(2, i) = std::clamp((norm_pressure - 0.75) * 4.0, 0.0, 1.0);
        } else {
          colors(2, i) =
              std::clamp(1.0 - (norm_pressure - 0.25) * 4.0, 0.0, 1.0);
        }
      }

      meshcat_->SetTriangleColorMesh(path + "/contact_surface", item.p_WV,
                                     item.faces, colors, false);
      meshcat_->SetTransform(path + "/contact_surface",
                             RigidTransformd(-item.centroid_W));
      meshcat_->SetTriangleMesh(path + "/contact_surface_wireframe", item.p_WV,
                                item.faces, geometry::Rgba(1.0, 1.0, 1.0, 1.0),
                                true);
      meshcat_->SetTransform(path + "/contact_surface_wireframe",
                             RigidTransformd(-item.centroid_W));
    }
  }

  // Update meshcat visibility to match the active status.
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

  // Start with it being invisible, to prevent flickering at the origin.
  iter = path_visibility_status_.insert({path, {false, false}}).first;
  meshcat_->SetProperty(path, "visible", false);

  // Add the geometry to meshcat.
  // Set radius 1.0 so that it can be scaled later by the force/moment norm in
  // the path transform.
  const Cylinder cylinder(params_.radius, 1.0);
  const double arrowhead_height = params_.radius * 2.0;
  const double arrowhead_width = params_.radius * 2.0;
  const MeshcatCone arrowhead(arrowhead_height, arrowhead_width,
                              arrowhead_width);

  meshcat_->SetObject(path + "/force_C_W/cylinder", cylinder,
                      params_.hydro_force_color);
  meshcat_->SetObject(path + "/force_C_W/head", arrowhead,
                      params_.hydro_force_color);
  meshcat_->SetObject(path + "/moment_C_W/cylinder", cylinder,
                      params_.hydro_moment_color);
  meshcat_->SetObject(path + "/moment_C_W/head", arrowhead,
                      params_.hydro_moment_color);
  return iter->second;
}

}  // namespace internal
}  // namespace meshcat
}  // namespace multibody
}  // namespace drake
