#pragma once

#include <memory>
#include <string>
#include <utility>

namespace drake {
namespace geometry {
namespace render {

// TODO(SeanCurtis-TRI): Allow for configuring the intrinsic matrix directly
// with a projection matrix.
// TODO(eric.cousineau): Make this have `CameraInfo` as a member, move that
// class outside of `drake::systems`, and possibly rename this class, in order
// to decouple generic pinhole parameters from Drake-specific render engine
// information.
/** The intrinsic properties for a render camera. The render system
 uses a reduced set of intrinsic parameters by making some simplifying
 assumptions:

 - Zero skew coefficient between the x- and y-axes.
 - The camera's principal point lies in the center of the image.

 The focal length is inferred by the sensor format (width and height) and the
 field of view along the y-axis. */
struct CameraProperties {
  CameraProperties(int width_in, int height_in, double fov_y_in,
                   std::string renderer_name_in)
      : width(width_in),
        height(height_in),
        fov_y(fov_y_in),
        renderer_name(std::move(renderer_name_in)) {}

  int width{};          ///< The width of the image (in pixels) to be rendered.
  int height{};         ///< The height of the image (in pixels) to be rendered.
  double fov_y{};       ///< The camera's vertical field of view (in radians).
  std::string renderer_name;  ///< The name of the renderer to use.
};

/** The intrinsic properties for a render _depth_ camera. Consists of all of the
 intrinsic properties of the render camera but extended with additional
 depth-specific parameters.
 @see CameraProperties */
struct DepthCameraProperties : public CameraProperties {
  DepthCameraProperties(int width_in, int height_in, double fov_y_in,
                        std::string renderer_name_in, double z_near_in,
                        double z_far_in)
      : CameraProperties(width_in, height_in, fov_y_in,
                         std::move(renderer_name_in)),
        z_near(z_near_in),
        z_far(z_far_in) {}

  double z_near{};  ///< The minimum reportable depth value. All surfaces closer
                    ///< than this distance, saturate to this value.
  double z_far{};   ///< The maximum reportable depth value. All surfaces
                    ///< farther than this distance, saturate to this value.
};

// TODO(SeanCurtis-TRI): Add properties for structured-light depth sensor which
// includes offsets from camera pose for both the emitter and sensor.

}  // namespace render
}  // namespace geometry
}  // namespace drake
