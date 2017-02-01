#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple data format for camera information that includes the size of image
/// and camera intrinsic parameters.
///
/// @ingroup sensor_systems
// TODO(kunimatsu-tri) Add camera distortion parameters and other things when
// needed.
class CameraInfo {
 public:
  /// Constructor with image size, focal lengths and center of image.
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  /// @param fx Focal length for x direction in pixel
  /// @param fy Focal length for y direction in pixel
  /// @param cx X value for center of image at image coordinate system
  /// @param cy Y value for center of image at image coordinate system
  CameraInfo(uint32_t width, uint32_t height, double fx, double fy,
             double cx, double cy) : width_(width), height_(height) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(fx > 0);
    DRAKE_ASSERT(fy > 0);
    DRAKE_ASSERT(cx > 0 && cx < static_cast<double>(width));
    DRAKE_ASSERT(cy > 0 && cy < static_cast<double>(height));
    intrinsic_matrix_ << fx, 0., cx,
                         0., fy, cy,
                         0., 0., 1.;
  }

  /// Constructor with image size and vertical field of view (fov).  We assume
  /// there is no image offset, and so the center of the image (cx, cy) is equal
  /// to (width / 2, height / 2).  The horizontal field of view is calculated by
  /// the aspect ratio of the image width and height together with the vertical
  /// field of view, then the focal lengths fx and fy are calculated by both of
  /// the field of views.
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  /// @param vertical_fov_rad Vertical field of view in radians
  CameraInfo(uint32_t width, uint32_t height, double vertical_fov_rad);

  /// Default copy constructor
  CameraInfo(const CameraInfo&) = default;

  /// Default copy assignment operator
  CameraInfo& operator=(const CameraInfo&) = default;

  /// Default move constructor
  CameraInfo(CameraInfo&&) = default;

  /// Default move assignment operator
  CameraInfo& operator=(CameraInfo&&) = default;

  CameraInfo() = delete;

  /// Return the size of width for image
  uint32_t width() const { return width_; }

  /// Return the size of height for image
  uint32_t height() const { return height_; }

  /// Return the focal length for x direction
  double fx() const { return intrinsic_matrix_(0, 0); }

  /// Return the focal length for y direction
  double fy() const { return intrinsic_matrix_(1, 1); }

  /// Return the center of image for x direction
  double cx() const { return intrinsic_matrix_(0, 2); }

  /// Return the center of image for y direction
  double cy() const { return intrinsic_matrix_(1, 2); }

  /// Return the camera intrinsic matrix
  const Eigen::Matrix3d& intrinsic_matrix() const {
    return intrinsic_matrix_;
  }

 private:
  uint32_t width_;
  uint32_t height_;
  Eigen::Matrix3d intrinsic_matrix_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
