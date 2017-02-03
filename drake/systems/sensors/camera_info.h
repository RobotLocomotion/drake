#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {
namespace sensors {

/// Simple data structure for camera information that includes the image size
/// and camera intrinsic parameters.
///
/// To clarify the terminology used to describe 2D and/or 3D spaces throughout
/// the class, we use "coordinate system" rather than "frame".  This is because
/// the term "frame" in the computer vision field has two major meaning: a
/// synonym of coordinate system, and a snapshot out of consecutively captured
/// images.  To ensure the reader will not be confused, we clearly differentiate
/// the use of the terms "coordinate system" and "frame" by using "coordinate
/// system" to denote 2D/3D spaces and "frame" to denote a captured image.
///
/// There are three types of the coordinate systems that are relevant to this
/// class: 1) "camera coordinate system", 2) "image coordinate system", 3)
/// "pixel coordinate system".
/// The "camera coordinate system" expresses a camera's 6D pose with regard to
/// the world coordinate system.  We have chosen the axes in the "camera
/// coordinate system" to be `X-right`, `Y-down` and `Z-forward`.  The `Z` axis
/// is also called the "optical axis".  Note that each axis in the "camera
/// coordinate system" is expressed in the upper case, like `(X, Y, Z)` to
/// distinguish from those of the "image coordinate system" which we explain
/// next.
///
/// The "image coordinate system" is the 2D coordinate system made by projecting
/// the "camera coordinate system" onto the 2D image plane which is
/// perpendicular to the "optical axis".  Therefore, the direction of the each
/// axis is `x-right` and `y-down`, and the origin of the "image coordinate
/// system" is located at the crossing point between the 2D image plane and the
/// "optical axis".  This origin is called the "image center" or "principal
/// point".  Note that each axis in the "image coordinate system" is expressed
/// in the lower case, like `(x, y)`.
///
/// The "pixel coordinate system" is also the 2D coordinate system.  The main
/// differences between the "pixel coordinate system" and the "image coordinate
/// system" are the location of the origin and the direction of the axes.  We
/// have chosen the origin of the "pixel coordinate system" to be the left-upper
/// corner of the image and the direction of the each axis to be the same as
/// those of the "image coordinate system".  The axes of the "pixel coordinate
/// system" are expressed by using `u` and `v`, therefore the directions of
/// the axes are `u-right` and `v-down`.
///
/// For more detail including an explanation of the focal lengths, refer to:
/// http://docs.opencv.org/2.4/modules/calib3d/doc/
/// camera_calibration_and_3d_reconstruction.html and https://en.wikipedia.org/
/// wiki/Pinhole_camera_model.
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
  /// @param focal_x The focal length x in pixels.
  /// @param focal_y The focal length y in pixels.
  /// @param center_x The x coordinate of the image center in the pixel coordinate
  /// system in pixels.
  /// @param center_y The y coordinate of the image center in the pixel coordinate
  /// system in pixels.
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

  /// Constructor that sets the image size and vertical field of view `fov_y`.
  /// We assume there is no image offset, so the image center `(center_x,`
  /// `center_y)` is equal to `(width / 2, height / 2)`.  The horizontal field
  /// of view `fov_x` is calculated using the aspect ratio of the image width and
  /// height together with the vertical field of view. The focal lengths
  /// `focal_x` and `focal_y` are calculated as follows:
  /// <pre>
  ///   focal_x = width / 2 / tan(fov_x / 2)
  ///   focal_y = height / 2 / tan(fov_y / 2)
  /// </pre>
  /// where `fov_x = width / height * fov_x`.
  ///
  /// @param width The image width in pixels, must be greater than zero.
  /// @param height The image height in pixels, must be greater than zero.
  /// @param fov_y The vertical field of view.
  CameraInfo(int width, int height, double fov_y);

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

  /// Returns the focal length x in pixels.
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /// Returns the focal length y in pixels.
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  /// Returns the image center x value in pixels.
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /// Returns the image center y value in pixels.
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
