#pragma once

#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace systems {
namespace rendering {

/// XXX
template <typename T>
class PoseVectorToGeometry final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseVectorToGeometry)

  PoseVectorToGeometry(geometry::SourceId, geometry::FrameId);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PoseVectorToGeometry(const PoseVectorToGeometry<U>&);

  ~PoseVectorToGeometry() override;

  /// Returns the rendering::PoseVector input port.
  const InputPort<T>& get_input_port() const {
    return LeafSystem<T>::get_input_port(0);
  }

  /// Returns the geometry::FramePoseVector output port.
  const OutputPort<T>& get_output_port() const {
    return LeafSystem<T>::get_output_port(0);
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class PoseVectorToGeometry;

  optional<bool> DoHasDirectFeedthrough(int, int) const final {
    return true;
  }

  const geometry::SourceId source_id_;
  const geometry::FrameId frame_id_;
};

}  // namespace rendering
}  // namespace systems
}  // namespace drake
