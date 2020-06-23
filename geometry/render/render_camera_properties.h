#pragma once

#include <string>

#include "drake/geometry/geometry_properties.h"

namespace drake {
namespace geometry {
namespace render {

/** Characterization of a depth sensor's functional range. Only points that lie
 within the range `[min_depth, max_depth]` will register meaningful values.  */
class DepthRange {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DepthRange);

  DepthRange(double min_in, double max_in);

  double min_depth() const { return min_depth_;  }
  double max_depth() const { return max_depth_;  }

 private:
  double min_depth_{};
  double max_depth_{};
};

/** The set of properties for specifying camera models for rendering. Some of
 the properties relate to an underlying mathematical model and others relate
 to how a particular RenderEngine implementation implements those mathematical
 models. Each consumer of %RenderCameraProperties must document what
 properties it consumes, which are required, and its behavior in the absence
 of requested properties.

 Examples of such documentation:
   - @ref rgbd_sensor_camera_properties RgbdSensor's constructor
 */
class RenderCameraProperties final : public GeometryProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderCameraProperties);

  RenderCameraProperties() = default;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
