#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/render/camera_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace geometry {
namespace render {

/** Defines the near and far clipping planes for frustum-based (e.g. OpenGL)
 RenderEngine cameras.  */
class ClippingRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ClippingRange);

  /** Constructs the %ClippingRange.
   @throws std::runtime_error if either value isn't positive, or if
                              `near >= far`.  */
  ClippingRange(double near, double far);

  double near() const { return near_; }
  double far() const { return far_; }

 private:
  double near_{};
  double far_{};
};

/** Collection of core parameters for modeling a pinhole-model camera in a
 RenderEngine. These parameters are applicable to both depth and color/label
 renderings. Parameters specific to those output image types can be found
 below.

 While these parameters are generally applicable to all RenderEngine
 implementations, this is not guaranteed to be true. For example, the clipping
 range property only applies to frustum-based RenderEngine implementations.
 I.e., it wouldn't apply to a ray-tracing based implementation.  */
class RenderCameraCore {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderCameraCore);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /** (Advanced) Constructs a %RenderCameraCore from the old, symmetric camera
   representation. This constructor should only be used internally; it serves
   as a stop gap measure until CameraProperties is fully deprecated.  */
  RenderCameraCore(const CameraProperties& camera, double clipping_far,
                   math::RigidTransformd X_BS = {});
#pragma GCC diagnostic pop

  /** Fully-specified constructor. See the documentation on the member getter
   methods for documentation of parameters.  */
  RenderCameraCore(std::string renderer_name,
                   systems::sensors::CameraInfo intrinsics,
                   ClippingRange clipping, math::RigidTransformd X_BS)
      : renderer_name_(std::move(renderer_name)),
        intrinsics_(std::move(intrinsics)),
        clipping_(std::move(clipping)),
        X_BS_(std::move(X_BS)) {}

  /** The name of the render engine this camera should be used with.  */
  const std::string& renderer_name() const { return renderer_name_; }

  /** The camera's intrinsic properties (e.g., focal length, sensor size, etc.)
   See systems::sensors::CameraInfo for details.  */
  const systems::sensors::CameraInfo& intrinsics() const { return intrinsics_; }

  /** The near and far clipping planes for this camera. This property is ignored
   by RenderEngine implementations that don't use a clipping frustum.  */
  const ClippingRange& clipping() const { return clipping_; }

  /** The pose of the sensor frame (S) in the camera's body frame (B). This is
   the "imager" referred to in systems::sensors::CameraInfo's documentation.  */
  const math::RigidTransformd& sensor_pose_in_camera_body() const {
    return X_BS_;
  }

  /** Expresses `this` camera's pinhole camera properties as the projective
   transform T_DC which transforms points in a camera's frame C to a 2D,
   normalized device frame D. The transform is represented by a 4x4 matrix
   (i.e., a
   <a href="https://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/">
   classic OpenGl projection matrix</a>).  */
  Eigen::Matrix4d CalcProjectionMatrix() const;

 private:
  // Used to convert CameraProperties to RenderCameraCore; this is the legacy
  // value used. Remove this when we remove the conversion constructor.
  static constexpr double kClippingNear = 0.01;

  // See getter methods for documentation on these members.
  std::string renderer_name_;
  systems::sensors::CameraInfo intrinsics_;
  ClippingRange clipping_;
  math::RigidTransformd X_BS_;

  // TODO(SeanCurtis-TRI): in the future, add a sub-class of GeometryProperties
  //  so that arbitrary properties can be added to support arbitrary
  //  RenderEngine implementations.
};

/** Collection of camera properties for cameras to be used with color/label
 images.  */
class ColorRenderCamera {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ColorRenderCamera);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /** Constructs a %ColorRenderCamera from the old, symmetric camera
   representation. This constructor should only be used internally; it serves
   as a stop gap measure until CameraProperties is fully deprecated.  */
  explicit ColorRenderCamera(const CameraProperties& camera,
                             bool show_window = false,
                             math::RigidTransformd X_BC = {})
      : ColorRenderCamera(
            RenderCameraCore(camera, kClippingFar, std::move(X_BC)),
            show_window) {}
#pragma GCC diagnostic pop

  /** Fully-specified constructor. See the documentation on the member getter
   methods for documentation of parameters.  */
  explicit ColorRenderCamera(RenderCameraCore core, bool show_window = false)
      : core_(std::move(core)), show_window_(show_window) {}

  /** This camera's core render properties.  */
  const RenderCameraCore& core() const { return core_; }

  /** If true, requests that the RenderEngine display the rendered image.  */
  bool show_window() const { return show_window_; }

 private:
  // Used to convert CameraProperties to ColorRenderCamera; this is the legacy
  // value used. Remove this when we remove the conversion constructor.
  static constexpr double kClippingFar = 100;

  // See getter methods for documentation on these members.
  RenderCameraCore core_;
  bool show_window_{false};
};

/** Defines a depth sensor's functional range. Only points that lie within the
 range `[min_depth, max_depth]` will register meaningful values.

 @note It's important to carefully coordinate depth range and clipping planes.
 It might seem reasonable to use the depth range as clipping planes, but that
 would be a mistake. Objects closer than the depth range's minimum value have
 an occluding effect in reality. If the near clipping plane is set to the
 minimum depth range value, those objects will be clipped away and won't
 occlude as they should. In essence, the camera will see through them and return
 incorrect values from beyond the missing geometry. The near clipping plane
 should _always_ be closer than the minimum depth range. How much closer depends
 on the scenario. Given the scenario, evaluate the closest possible distance
 to the camera that geometry in the scene could possibly achieve; the clipping
 plane should be slightly closer than that. When in doubt, some very small
 value (e.g., 1 mm) is typically safe.  */
class DepthRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DepthRange);

  /** Constructs the %DepthRange.
   @throws std::runtime_error if either value isn't positive, or if
                              `min_in >= max_in`.  */
  DepthRange(double min_in, double max_in);

  double min_depth() const { return min_depth_; }
  double max_depth() const { return max_depth_; }

 private:
  double min_depth_{};
  double max_depth_{};
};

/** Collection of camera properties for cameras to be used with depth images.
 */
class DepthRenderCamera {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DepthRenderCamera);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /** Constructs a %DepthRenderCamera from the old, symmetric camera
   representation. This constructor should only be used internally; it serves
   as a stop gap measure until CameraProperties is fully deprecated.  */
  explicit DepthRenderCamera(const DepthCameraProperties& camera,
                             math::RigidTransformd X_BD = {})
      : DepthRenderCamera(
            RenderCameraCore(camera, camera.z_far * 1.1, std::move(X_BD)),
            DepthRange(camera.z_near, camera.z_far)) {}
#pragma GCC diagnostic pop

  /** Fully-specified constructor. See the documentation on the member getter
   methods for documentation of parameters.

   @throws std::runtime_error if the depth_range is not fully contained within
                              the clipping range.  */
  DepthRenderCamera(RenderCameraCore core, DepthRange depth_range);

  /** This camera's core render properties.  */
  const RenderCameraCore& core() const { return core_; }

  /** The range of valid values for the depth camera.  */
  const DepthRange& depth_range() const { return depth_range_; }

 private:
  // See getter methods for documentation on these members.
  RenderCameraCore core_;
  DepthRange depth_range_;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
