#pragma once

#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace rendering {

/// A direct-feedthrough system that converts the C++ type of poses from
/// rendering::PoseVector<T> on the input to geometry::FramePoseVector<T>
/// on the output.
template <typename T>
class DRAKE_DEPRECATED("2021-12-01",
                       "RenderPoseToGeometryPose is no longer in use. "
                       "Instead of passing through PoseVector, please output "
                       "geometry::FramePoseVector directly.")
RenderPoseToGeometryPose final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RenderPoseToGeometryPose)

  RenderPoseToGeometryPose(geometry::SourceId, geometry::FrameId);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit RenderPoseToGeometryPose(const RenderPoseToGeometryPose<U>&);
#pragma GCC diagnostic pop

  ~RenderPoseToGeometryPose() override;

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class RenderPoseToGeometryPose;

  const geometry::SourceId source_id_;
  const geometry::FrameId frame_id_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
