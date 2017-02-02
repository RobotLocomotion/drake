#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple data structure for camera information that includes the image size
/// and camera intrinsic parameters.
///
/// @ingroup sensor_systems
// TODO(kunimatsu-tri) Add camera distortion parameters and other things when
// needed.
class CameraInfo {
 public:
  /// Constructor with image size, focal lengths and center of image.
  ///
  /// @param width Size of width for image in pixels which should be greater
  /// than zero
  /// @param height Size of height for image in pixels which should be greater
  /// than zero
  /// @param fx Focal length for x direction in pixels
  /// @param fy Focal length for y direction in pixels
  /// @param cx X value for center of image at image coordinate system in pixels
  /// @param cy Y value for center of image at image coordinate system in pixels
  CameraInfo(int width, int height, double fx, double fy, double cx, double cy)
      : width_(width), height_(height), intrinsic_matrix_(
            (Eigen::Matrix3d() <<
             fx, 0., cx, 0., fy, cy, 0., 0., 1.).finished()) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(fx > 0);
    DRAKE_ASSERT(fy > 0);
    DRAKE_ASSERT(cx > 0 && cx < static_cast<double>(width));
    DRAKE_ASSERT(cy > 0 && cy < static_cast<double>(height));
  }

  /// Constructor with image size and vertical field of view (fov).  We assume
  /// there is no image offset, so the center of the image `(cx, cy)` is equal
  /// to `(width / 2, height / 2)`.  The horizontal field of view is calculated
  /// by the aspect ratio of the image width and height together with the
  /// vertical field of view. The focal lengths "fx" and "fy" are calculated by
  /// both of the field of views:
  /// <pre>
  ///   fx = width / 2 / tan(horizontal_fov / 2)
  ///   fy = height / 2 / tan(vertical_fov / 2)
  /// </pre>
  /// where horizontal_fov = width / height * vertical_fov.
  ///
  /// @param width Size of width for image which should be greater than zero
  /// @param height Size of height for image which should be greater than zero
  /// @param vertical_fov Vertical field of view angle
  CameraInfo(int width, int height, double vertical_fov);

  /// Default copy constructor.
  CameraInfo(const CameraInfo&) = default;

  /// Default move constructor.
  CameraInfo(CameraInfo&&) = default;

  CameraInfo() = delete;
  CameraInfo& operator=(const CameraInfo&) = delete;
  CameraInfo& operator=(CameraInfo&&) = delete;

  /// Returns the width of the image in pixels.
  int width() const { return width_; }

  /// Returns the height of the image in pixels.
  int height() const { return height_; }

  /// Returns the focal length for x direction in pixels.
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /// Returns the focal length for y direction in pixels.
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  /// Returns the center of image for x direction in pixels.
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /// Returns the center of image for y direction in pixels.
  double center_y() const { return intrinsic_matrix_(1, 2); }

  /// Returns the camera intrinsic matrix.
  const Eigen::Matrix3d& intrinsic_matrix() const {
    return intrinsic_matrix_;
  }

 private:
  const int width_;
  const int height_;
  // Camera intrinsic parameter matrix. For the detail, see
  // http://docs.opencv.org/2.4/modules/calib3d/doc/
  // camera_calibration_and_3d_reconstruction.html
  const Eigen::Matrix3d intrinsic_matrix_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
