#pragma once

#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/camera_info.h"

namespace drake {
namespace geometry {
namespace render {

/** Defines the near and far clipping planes for frustum-based (OpenGL)
 RenderEngine cameras.

 <h3>Guidance on selecting clipping plane values</h3>

 This documentation is targeted toward those who are unfamiliar with
 the OpenGL rasterization pipeline. For in-depth explanations about how the
 clipping range defines the viewing volume, see
 <a href="https://www.glprogramming.com/red/chapter03.html#name3">
 the discussion on projective transforms</a>. For more detail on its effect on
 determining occlusion (which geometries are in front), try
 <a href="https://www.glprogramming.com/red/chapter05.html#name1">
 "A Hidden-Surface Removal Survival Kit"</a>.

 <h4>The short summary</h4>
  - The clipping range defines the distance of the *closest* and *farthest*
    things that *can* appear in the rendering.
  - Objects that cross the planes placed at those distances get clipped.
  - Make the range as small as reasonably possible to get the best occlusion
    (a.k.a. z-buffer) results.
  - For depth cameras, make sure your clipping range always includes your valid
    depth range.

 <h4>The longer discussion</h4>

 Given that the clipping range defines what you can/can't see in the camera, it
 _might_ be tempting to just put an arbitrarily large range in (e.g., starting
 1 micrometer away and going up to 1 million kilometers away). By doing so, you
 know everything you put into your scene will appear. If making sure things are
 visible were the only factor, this would be fine.

 Rasterization pipelines render objects in arbitrary order but have to be able
 to determine which objects are in front of other objects as they go. They
 achieve this by creating a "z-buffer". It is a measure of the depth of the
 triangle that rendered to a particular "pixel". When two triangles both want to
 color the same pixel, the triangle with the smallest z-value is typically
 selected.

 The z-buffer has fixed precision. That fixed precision is spread over the
 entire available depth range (as defined by the clipping range). The greater
 the range, the less precision the z-buffer has per meter of depth. A small
 range may have the ability to distinguish objects separated by 1 mm. But a
 large range may only be able to distinuish objects separated by 1 m. Two
 objects with measurably different depths relative to the camera, can become
 indistinguishable in the z-buffer due to these precision issues; the object
 that ends up in front is due to random chance. This will lead to artifacts
 where two objects near the same depth will flicker back and forth in front of
 each other (sometimes called "z fighting").

 So, it is best to define the smallest clipping range that will include the
 objects of the scene that you care about most. */
class ClippingRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ClippingRange);

  /** Constructs the %ClippingRange.
   @throws std::exception if either value isn't positive, or if
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

  /** Fully-specified constructor. See the documentation on the member getter
   methods for documentation of parameters.  */
  explicit ColorRenderCamera(RenderCameraCore core, bool show_window = false)
      : core_(std::move(core)), show_window_(show_window) {}

  /** This camera's core render properties.  */
  const RenderCameraCore& core() const { return core_; }

  /** If true, requests that the RenderEngine display the rendered image.  */
  bool show_window() const { return show_window_; }

 private:
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
   @throws std::exception if either value isn't positive, or if
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

  /** Fully-specified constructor. See the documentation on the member getter
   methods for documentation of parameters.

   @throws std::exception if the depth_range is not fully contained within
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
