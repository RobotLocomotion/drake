#include "sim/common/camera_config.h"

#include <cmath>
#include <iostream>

#include "drake/systems/sensors/camera_info.h"

namespace anzu {
namespace sim {

using drake::geometry::render::ClippingRange;
using drake::geometry::render::ColorRenderCamera;
using drake::geometry::render::DepthRange;
using drake::geometry::render::DepthRenderCamera;
using drake::geometry::render::RenderCameraCore;
using drake::math::RigidTransform;
using drake::systems::sensors::CameraInfo;
using drake::Vector2;

void CameraConfig::FovDegrees::ValidateOrThrow() const {
  if (!x.has_value() && !y.has_value()) {
    throw std::logic_error(
        "Invalid camera configuration; you must define at least x or y "
        "for FovDegrees.");
  }
}

namespace {

// Computes the focal length along a single axis based on the the image
// dimension and field of view (in a degrees) in that direction.
// This computation must be kept in agreement with that in CameraInfo.
double CalcFocalLength(int image_dimension, double fov_degrees) {
  const double fov_rad = M_PI * fov_degrees / 180.0;
  return image_dimension * 0.5 / std::tan(0.5 * fov_rad);
}

}  // namespace

double CameraConfig::FovDegrees::focal_x(int width, int height) const {
  ValidateOrThrow();
  if (x.has_value()) {
    return CalcFocalLength(width, *x);
  } else {
    return focal_y(width, height);
  }
}

double CameraConfig::FovDegrees::focal_y(int width, int height) const {
  ValidateOrThrow();
  if (y.has_value()) {
    return CalcFocalLength(height, *y);
  } else {
    return focal_x(width, height);
  }
}

double CameraConfig::focal_x() const {
  if (std::holds_alternative<double>(focal)) {
    return std::get<double>(focal);
  } else if (std::holds_alternative<Anisotropic>(focal)) {
    return std::get<Anisotropic>(focal).x;
  } else {
    return std::get<FovDegrees>(focal).focal_x(width, height);
  }
}

double CameraConfig::focal_y() const {
  if (std::holds_alternative<double>(focal)) {
    return std::get<double>(focal);
  } else if (std::holds_alternative<Anisotropic>(focal)) {
    return std::get<Anisotropic>(focal).y;
  } else {
    return std::get<FovDegrees>(focal).focal_y(width, height);
  }
}

std::pair<ColorRenderCamera, DepthRenderCamera> CameraConfig::MakeCameras()
    const {
  const Vector2<double> center = principal_point();
  CameraInfo intrinsics(width, height, focal_x(), focal_y(), center.x(),
                        center.y());
  ClippingRange clipping_range(clipping_near, clipping_far);
  RenderCameraCore color_core(renderer_name, intrinsics, clipping_range,
                              X_BC.GetDeterministicValue());
  RenderCameraCore depth_core(renderer_name, intrinsics, clipping_range,
                              X_BD.GetDeterministicValue());
  DepthRange depth_range(z_near, z_far);
  return {ColorRenderCamera(color_core, show_rgb),
          DepthRenderCamera(depth_core, depth_range)};
}

void CameraConfig::ValidateOrThrow() const {
  // If we haven't specified color or depth, it is trivially valid.
  if (!(rgb || depth)) {
    return;
  }

  // This throws for us if we have bad bad numerical camera values (or an empty
  // renderer_name).
  MakeCameras();

  if (name.empty()) {
    throw std::logic_error(
        "Invalid camera configuration; name cannot be empty.");
  }

  if (renderer_name.empty()) {
    throw std::logic_error(
        "Invalid camera configuration; renderer_name cannot be empty.");
  }

  // We need to check the quantities unique to CameraConfig.
  if (fps <= 0 || !std::isfinite(fps)) {
    throw std::logic_error(
        fmt::format("Invalid camera configuration; FPS ({}) must be a finite, "
                    "positive value.",
                    fps));
  }

  if (X_BC.base_frame.has_value() && !X_BC.base_frame->empty()) {
    throw std::logic_error(
        fmt::format("Invalid camera configuration; X_BC must not specify a "
                    "base frame. '{}' found.",
                    *X_BC.base_frame));
  }

  if (X_BD.base_frame.has_value() && !X_BD.base_frame->empty()) {
    throw std::logic_error(
        fmt::format("Invalid camera configuration; X_BD must not specify a "
                    "base frame. '{}' found.",
                    *X_BD.base_frame));
  }
}
}  // namespace sim
}  // namespace anzu
