#pragma once

#include "drake/geometry/query_object.h"
#include "drake/math/rigid_transform.h"
#include "drake/systems/sensors/image.h"

namespace drake {
namespace geometry {

/** An extension of QueryObject which renders the SceneGraph world -- color,
 depth, etc. -- in addition to the queries made available by QueryObject.
 SceneGraph has an abstract-valued port that contains a %PerceptionQueryObject
 (i.e., a %PerceptionQueryObject-valued output port).

 Other than the additional queries that this class provides above and beyond
 those of QueryObject, acquiring a reference to a %PerceptionQueryObject,
 working with it, the semantics of copying it, etc., are all the same as the
 parent class. Please refer to QueryObject's documentation for details.

 @tparam_nonsymbolic_scalar
*/
template <typename T>
class PerceptionQueryObject : public QueryObject<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PerceptionQueryObject)

  /** Constructs a default %PerceptionQueryObject (all pointers are null). */
  PerceptionQueryObject() = default;

  /**
   @anchor render_queries
   @name                Render Queries

   The methods support queries along the lines of "What do I see?" They support
   simulation of sensors. External entities define a sensor camera -- its
   extrinsic and intrinsic properties and %PerceptionQueryObject renders into
   the provided image.

   <!-- TODO(SeanCurtis-TRI): Currently, pose is requested as a transform of
   double. This puts the burden on the caller to be compatible. Provide
   specializations for AutoDiff and symbolic (the former extracts a
   double-valued transform and the latter throws). -->
   */
  //@{

  /** Renders an RGB image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param show_window           If true, the render window will be displayed.
   @param[out] color_image_out  The rendered color image. */
  void RenderColorImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageRgba8U* color_image_out) const;

  /** Renders a depth image for the given `camera` posed with respect to the
   indicated parent frame P.

   In contrast to the other rendering methods, rendering depth images doesn't
   provide the option to display the window; generally, basic depth images are
   not readily communicative to humans.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param[out] depth_image_out  The rendered depth image. */
  void RenderDepthImage(const render::DepthCameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        systems::sensors::ImageDepth32F* depth_image_out) const;

  /** Renders a label image for the given `camera` posed with respect to the
   indicated parent frame P.

   @param camera                The intrinsic properties of the camera.
   @param parent_frame          The id for the camera's parent frame.
   @param X_PC                  The pose of the camera body in the world frame.
   @param show_window           If true, the render window will be displayed.
   @param[out] label_image_out  The rendered label image. */
  void RenderLabelImage(const render::CameraProperties& camera,
                        FrameId parent_frame,
                        const math::RigidTransformd& X_PC,
                        bool show_window,
                        systems::sensors::ImageLabel16I* label_image_out) const;

  //@}
};

}  // namespace geometry
}  // namespace drake
