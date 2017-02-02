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
// TODO(kunimatsu-tri) Add camera distortion parameters and other parameters as
// needed.
class CameraInfo {
 public:
  /// Constructor that directly sets the image size, center, and focal lengths.
  ///
  /// @param width The image width in pixels, must be greater than zero.
  /// @param height The image height in pixels, must be greater than zero.
  /// @param focal_x The focal length for the x direction in pixels.
  /// @param focal_y The focal length for the y direction in pixels.
  /// @param center_x X value at the image center in the image coordinate system
  /// in pixels.
  /// @param center_y Y value at the image center in the image coordinate system
  /// in pixels.
  CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y)
      : width_(width), height_(height), intrinsic_matrix_((
            Eigen::Matrix3d() << focal_x, 0., center_x,
                                 0., focal_y, center_y,
                                 0., 0., 1.).finished()) {
    DRAKE_ASSERT(width > 0);
    DRAKE_ASSERT(height > 0);
    DRAKE_ASSERT(focal_x > 0);
    DRAKE_ASSERT(focal_y > 0);
    DRAKE_ASSERT(center_x > 0 && center_x < static_cast<double>(width));
    DRAKE_ASSERT(center_y > 0 && center_y < static_cast<double>(height));
  }

  /// Constructor that sets the image size and vertical field of view (fov).  We
  /// assume there is no image offset, so the center of the image `(center_x,`
  /// `center_y)` is equal to `(width / 2, height / 2)`.  The horizontal field
  /// of view is calculated by the aspect ratio of the image width and height
  /// together with the vertical field of view. The focal lengths `focal_x` and
  /// `focal_y` are calculated by both of the field of views:
  /// <pre>
  ///   focal_x = width / 2 / tan(horizontal_fov / 2)
  ///   focal_y = height / 2 / tan(vertical_fov / 2)
  /// </pre>
  /// where `horizontal_fov = width / height * vertical_fov`.
  ///
  /// @param width The image width in pixels, must be greater than zero.
  /// @param height The image height in pixels, must be greater than zero.
  /// @param vertical_fov The vertical field of view.
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

  /// Returns the focal length for the x direction in pixels.
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /// Returns the focal length for the y direction in pixels.
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  /// Returns the center of image for the x direction in pixels.
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /// Returns the center of image for the y direction in pixels.
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
