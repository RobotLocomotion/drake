#include "drake/systems/sensors/camera_config.h"

#include <cmath>

#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace systems {
namespace sensors {

using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using math::RigidTransform;

void CameraConfig::FocalLength::ValidateOrThrow() const {
  // N.B. The positive, finiteness of the values is tested when we validate
  // CameraConfig by construction render cameras.
  if (!x.has_value() && !y.has_value()) {
    throw std::logic_error(
        "Invalid camera configuration; you must define at least x or y "
        "for FocalLength.");
  }
}

double CameraConfig::FocalLength::focal_x() const {
  ValidateOrThrow();
  if (x.has_value()) {
    return *x;
  } else {
    return *y;
  }
}

double CameraConfig::FocalLength::focal_y() const {
  ValidateOrThrow();
  if (y.has_value()) {
    return *y;
  } else {
    return *x;
  }
}

void CameraConfig::FovDegrees::ValidateOrThrow() const {
  // N.B. The positive, finiteness of the values is tested when we validate
  // CameraConfig by construction render cameras.
  if (!x.has_value() && !y.has_value()) {
    throw std::logic_error(
        "Invalid camera configuration; you must define at least x or y "
        "for FovDegrees.");
  }
}

namespace {

// Computes the focal length along a single axis based on the image
// dimension and field of view (in a degrees) in that direction.
// This computation must be kept in agreement with that in CameraInfo.
double CalcFocalLength(int image_dimension, double fov_degrees) {
  const double fov_rad = M_PI * fov_degrees / 180.0;
  return image_dimension * 0.5 / std::tan(0.5 * fov_rad);
}

}  // namespace

double CameraConfig::FovDegrees::focal_x(int width_in, int height_in) const {
  ValidateOrThrow();
  if (x.has_value()) {
    return CalcFocalLength(width_in, *x);
  } else {
    return focal_y(width_in, height_in);
  }
}

double CameraConfig::FovDegrees::focal_y(int width_in, int height_in) const {
  ValidateOrThrow();
  if (y.has_value()) {
    return CalcFocalLength(height_in, *y);
  } else {
    return focal_x(width_in, height_in);
  }
}

double CameraConfig::focal_x() const {
  if (std::holds_alternative<FocalLength>(focal)) {
    return std::get<FocalLength>(focal).focal_x();
  } else {
    return std::get<FovDegrees>(focal).focal_x(width, height);
  }
}

double CameraConfig::focal_y() const {
  if (std::holds_alternative<FocalLength>(focal)) {
    return std::get<FocalLength>(focal).focal_y();
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
  // If we haven't specified any image types, it is trivially valid.
  if (!(rgb || depth || label)) {
    return;
  }

  // We validate the focal length substruct here, instead of during its local
  // Serialize. We don't want to validate anything if the cameras are disabled,
  // and because the default focal length is a property of the CameraConfig
  // instead of the child struct, we must not validate the child in isolation.
  std::visit(
      [](auto&& child) -> void {
        child.ValidateOrThrow();
      },
      focal);

  // We don't worry about the other variant alternatives; if we're here, we've
  // constructed a set of parameters, and we'll defer to the RenderEngine to
  // determine if the values are valid.
  if (std::holds_alternative<std::string>(renderer_class)) {
    const auto& class_name = std::get<std::string>(renderer_class);
    if (!class_name.empty() &&
        !(class_name == "RenderEngineVtk" || class_name == "RenderEngineGl" ||
          class_name == "RenderEngineGltfClient")) {
      throw std::logic_error(fmt::format(
          "Invalid camera configuration; the given renderer_class value '{}' "
          "must be empty (to use the default) or be one of 'RenderEngineVtk', "
          "'RenderEngineGl', or 'RenderEngineGltfClient'.",
          class_name));
    }
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

  if (capture_offset < 0 || !std::isfinite(capture_offset)) {
    throw std::logic_error(fmt::format(
        "Invalid camera configuration; capture_offset ({}) must be a finite, "
        "non-negative value.",
        capture_offset));
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
}  // namespace sensors
}  // namespace systems
}  // namespace drake
