#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(kunimatsu-tri) Add camera distortion parameters and other parameters as
// needed.
/**
 Simple struct for characterizing the Drake camera model. The camera model is
 based on the
 [pinhole _model_](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html)
 , which is related to but distinct from an actual
 [pinhole _camera_](https://en.wikipedia.org/wiki/Pinhole_camera). The former
 is a mathematical model for producing images, the latter is a physical object.

 The camera info members are directly tied to the underlying model's
 mathematical parameters. In order to understand the model parameters, we will
 provide a discussion of how cameras are treated in Drake with some common
 terminology (liberally borrowed from computer vision).

 <h3>Pinhole camera model</h3>

 (To get an exact definition of the terms used here, please refer to the
 glossary below.)

 Intuitively, a camera produces images of an observed environment. The pinhole
 model serves as a camera for a virtually modeled environment. Its parameters
 are those required to determine where in the resultant image a virtual object
 appears. In essence, the parameters of the camera model define a mapping from
 points in 3D space to a point on the image (2D space).

 The full discussion of this mapping can be found in the
 [OpenCV documentation](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html).
 Here, we'll highlight one or two points as it relates to this struct.

 The mapping from 3D to 2D is decomposed into two sets of properties: extrinsic
 and intrinsic. The extrinsic properties define the pose of the camera in the
 environment -- specifically, given the camera frame `C`, it defines `X_WC`.
 Once a point `Q` is measured and expressed in the camera's frame (i.e.,
 `p_CQ_C`), the intrinsic matrix projects it into the 2D image. %CameraInfo does
 _not_ concern itself with the extrinsic properties (other classes are
 responsible for that -- see RgbdSensor).

 %CameraInfo defines the parameters of the intrinsic projection. The projection
 can be captured by the camera or intrinsic matrix which essentially maps points
 in the camera frame C to the image plane (the matrix is called `A` in the
 OpenCV documentation, but typically called `K` in computer vision literature):

         │ f_x  0    c_x │
     K = │ 0    f_y  c_y │, i.e.,
         │ 0    0    1   │

 - This matrix maps a point in the camera frame C to the projective space of the
   image (via homogeneous coordinates). The resulting image coordinate `(u, v)`
   is extracted by dividing out the homogeneous scale factor.
 - (c_x, c_y) defines the principal point.
 - (f_x, f_y) defines the model focal length.

 In other words, for point Q in the world frame, its texture coordinates
 `(u_Q, v_Q)` are calculated as:

      │ u_Q │
     s│ v_Q │ = K ⋅ X_CW ⋅ p_WQ
      │  1  │

 Note: The expression on the right will generally produce a homogeneous
 coordinate vector of the form `(s * u_Q, s * v_Q, s)`. The texture coordinate
 is defined as the first two measures when the *third* measure is 1. The magic
 of homogeneous coordinates allows us to simply factor out `s`.

 <h3>Glossary</h3>

 These terms are important to the discussion. Some refer to real world concepts
 and some to the model. The application domain of the term will be indicated
 and, where necessary, ambiguities will be resolved. The terms are ordered
 alphabetically for ease of reference.

 - __aperture__: the opening in a camera through which light passes. The origin
   of the camera frame `Co` is located at the aperture's center.
   - in a physical camera it may contain a lens and the size of the camera
     affects optical artifacts such as depth of field.
   - in the pinhole model, there is no lens and the aperture is a single point.
 - __focal length__: a camera property that determines the field of view angle
   and the scale of objects in the image (large focal length --> small field of
   view angle).
   - In a physical camera, it is the distance (in meters) between the center of
     the lens and the plane at which all incoming, parallel light rays converge
     (assuming a radially symmetric lens).
   - in the pinhole model, `(f_x, f_y)` is described as "focal length", but the
     units are in pixels and the interpretation is different. The relationship
     between `(f_x, f_y)` and the physical focal length `F` is `f_i = F * s_i`,
     where `(s_x, s_y)` are the number of pixels per meter in the x- and
     y-directions (of the image). Both values described as "focal length" have
     analogous effects on field of view and scale of objects in the image.
 - __frame__: (Also "pose") an origin point and a space-spanning basis in 2D/3D,
   as in @ref multibody_frames_and_bodies.
   - Incidentally, the word "frame" is also used in conjunction with a single
   image in a video (such as a "video frame"). Drake doesn't use this sense in
   discussing cameras (unless explicitly noted).
 - __image__: an array of measurements such that the measured values have
   spatial relationships based on where they are in the array.
 - __image frame__: the 2D frame embedded in 3D space spanning the image plane.
   The image frame's x- and y-axes are parallel with `Cx` and `Cy`,
   respectively. Coordinates are expressed as the pair `(u, v)` and the camera's
   image lies on the plane spanned by the frame's basis. The _center_ of the
   pixel in the first row and first column is at `(u=0, v=0)`.
 - __image plane__: a plane in 3D which is perpendicular to the camera's viewing
   direction. Conceptually, the image lies on this plane.
     - In a physical pinhole camera, the aperture is between the image plane and
     the environment being imaged.
     - In the pinhole model, the image plane lies between the environment and
     aperture (i.e., in the positive Cz direction from Co).
 - __imager__: a sensor whose measurements are reported in images.
 - __principal point__: The projection of the camera origin, `Co`, on the image
   plane. Its value is measured from the image's origin in pixels.
 - __sensor__: a measurement device.
 - __viewing direction__: the direction the camera is facing. Defined as being
   parallel with Cz.

 When looking at the resulting image and reasoning about the camera that
 produced it, one can say that Cz points into the image, Cx is parallel with the
 image rows, pointing to the right, and Cy is parallel with the image columns,
 pointing down leading to language such as: "X-right", "Y-down", and "Z-forward".
*/
class CameraInfo final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CameraInfo)

  /**
   Constructor that directly sets the image size, principal point, and focal
   lengths.

   @param width    The image width in pixels, must be greater than zero.
   @param height   The image height in pixels, must be greater than zero.
   @param focal_x  The _model_ "focal length" x in pixels (as documented above).
   @param focal_y  The _model_ "focal length" y in pixels (as documented above).
   @param center_x The x coordinate of the principal point in pixels (as
                   documented above).
   @param center_y The y coordinate of the principal point in pixels (as
                   documented above).
  */
  CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y);

  /**
   Constructs this instance from image size and vertical field of view. We
   assume the principal point is in the center of the image;
   `(center x, center_y)` is equal to `(width / 2.0 - 0.5, height / 2.0 - 0.5)`.
   We also assume the focal lengths `focal_x` and `focal_y` are identical
   (modeling a radially symmetric lens). The value is derived from field of
   view and image size as:

        focal_x = focal_y = height * 0.5 / tan(0.5 * fov_y)

   @param width The image width in pixels, must be greater than zero.
   @param height The image height in pixels, must be greater than zero.
   @param fov_y The vertical field of view in radians.
  */
  CameraInfo(int width, int height, double fov_y);

  /** Returns the width of the image in pixels. */
  int width() const { return width_; }

  /** Returns the height of the image in pixels. */
  int height() const { return height_; }

  /** Returns the focal length x in pixels. */
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /** Returns the focal length y in pixels. */
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  // TODO(eric.cousineau): Deprecate "center_{x,y}" and use
  // "principal_point_{x,y}" or "pp{x,y}".

  /** Returns the principal point's x coordinate in pixels. */
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /** Returns the principal point's y coordinate in pixels. */
  double center_y() const { return intrinsic_matrix_(1, 2); }

  /** Returns the camera intrinsic matrix, K. */
  const Eigen::Matrix3d& intrinsic_matrix() const {
    return intrinsic_matrix_;
  }

 private:
  int width_{};
  int height_{};
  // Camera intrinsic parameter matrix. For the detail, see
  // http://docs.opencv.org/2.4/modules/calib3d/doc/
  // camera_calibration_and_3d_reconstruction.html
  Eigen::Matrix3d intrinsic_matrix_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
