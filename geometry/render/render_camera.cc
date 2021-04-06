#include "drake/geometry/render/render_camera.h"

#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;
using systems::sensors::CameraInfo;

ClippingRange::ClippingRange(double near, double far) : near_(near), far_(far) {
  if (near <= 0 || far <= 0 || far <= near) {
    throw std::runtime_error(fmt::format(
        "The clipping range values must both be positive and far must be "
        "greater than near. Instantiated with near = {} and far = {}",
        near, far));
  }
}

Eigen::Matrix4d RenderCameraCore::CalcProjectionMatrix() const {
  /* Given the camera properties we compute the projection matrix as follows:
   (See https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/)

            │ 2*fx/w     0      (w - 2*cx) / w       0    │
            │ 0        2*fy/h  -(h - 2*cy) / h       0    │
            │ 0          0        -(f+n) / d   -2*f*n / d │
            │ 0          0             -1            0    │

   The symbols in the matrix are predominantly aliases for the input parameter
   values (see below for details).
   */
  const double fx = intrinsics_.focal_x();
  const double fy = intrinsics_.focal_y();
  const double n = clipping().near();
  const double f = clipping().far();
  const int w = intrinsics_.width();
  const int h = intrinsics_.height();
  const double cx = intrinsics_.center_x();
  const double cy = intrinsics_.center_y();
  const double d = f - n;

  Eigen::Matrix4d T_DC = Eigen::Matrix4d::Zero();
  T_DC(0, 0) =  2 * fx / w;
  T_DC(0, 2) =  (w - 2 * cx) / w;
  T_DC(1, 1) =  2 * fy / h;
  T_DC(1, 2) =  -(h - 2 * cy) / h;
  T_DC(2, 2) =  -(f + n) / d;
  T_DC(2, 3) =  -2 * f * n / d;
  T_DC(3, 2) =  -1;
  return T_DC;
}

DepthRange::DepthRange(double min_in, double max_in)
    : min_depth_(min_in), max_depth_(max_in) {
  if (min_depth_ <= 0 || max_depth_ <= 0 || max_depth_ <= min_depth_) {
    throw std::runtime_error(
        fmt::format("The depth range values must both be positive and "
                    "the maximum depth must be greater than the minimum depth. "
                    "Instantiated with min = {} and max = {}",
                    min_depth_, max_depth_));
  }
}

DepthRenderCamera::DepthRenderCamera(RenderCameraCore core,
                                     DepthRange depth_range)
    : core_(std::move(core)), depth_range_(std::move(depth_range)) {
  if (depth_range_.min_depth() < core_.clipping().near() ||
      depth_range_.max_depth() > core_.clipping().far()) {
    throw std::runtime_error(fmt::format(
        "Depth camera's depth range extends beyond the clipping planes; near = "
        "{}, far = {}, min. depth = {}, max. depth = {}",
        core_.clipping().near(), core_.clipping().far(),
        depth_range_.min_depth(), depth_range_.max_depth()));
  }
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
