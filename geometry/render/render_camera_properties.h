#pragma once

#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {
namespace render {

// TODO(SeanCurtis-TRI): Would this be better as a GeometryProperties instance?
/** Collection of parameters that helps bridge the camera model (as defined by a
 systems::sensors::ColorCameraModel or systems::sensors::DepthCameraModel) and a
 RenderEngine. This includes those per-camera properties appropriate to the
 RenderEngine's implementation of the camera model. This represents the union
 of known render engine camera properties and it is not the case that all
 properties are used by all render engines. */
class RenderCameraProperties {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RenderCameraProperties);

  /** Constructs properties for the given render engine. The clipping planes
   can be omitted to use the indicated default values.

   @pre `near_clipping > 0` and `far_clipping > 0`.
   @pre `far_clipping > near_clipping`.  */
  RenderCameraProperties(std::string render_engine_name,
                         double near_clipping = 0.01, double far_clipping = 10);

  const std::string& render_engine_name() const { return render_engine_name_; }
  double near_clipping_plane() const { return near_clipping_; }
  double far_clipping_plane() const { return far_clipping_; }

 private:
  std::string render_engine_name_;
  double near_clipping_{};
  double far_clipping_{};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
