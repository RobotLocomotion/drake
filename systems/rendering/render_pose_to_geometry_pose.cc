#include "drake/systems/rendering/render_pose_to_geometry_pose.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

using drake::geometry::FrameId;
using drake::geometry::SourceId;

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
RenderPoseToGeometryPose<T>::RenderPoseToGeometryPose(
    SourceId source_id, FrameId frame_id)
    : LeafSystem<T>(SystemTypeTag<rendering::RenderPoseToGeometryPose>{}),
      source_id_(source_id),
      frame_id_(frame_id) {
  using Input = PoseVector<T>;
  using Output = geometry::FramePoseVector<T>;
  this->DeclareVectorInputPort(Input{});
  this->DeclareAbstractOutputPort(
      [source_id, frame_id]() {
        const std::vector<FrameId> frame_ids{frame_id};
        const Output result(source_id, frame_ids);
        return Value<Output>::Make(result);
      },
      [this, frame_id](const Context<T>& context, AbstractValue* calculated) {
        const Input& input = get_input_port().template Eval<Input>(context);
        Output& output = calculated->get_mutable_value<Output>();
        output.clear();
        output.set_value(frame_id, input.get_isometry());
      });
}

template <typename T>
template <typename U>
RenderPoseToGeometryPose<T>::RenderPoseToGeometryPose(
    const RenderPoseToGeometryPose<U>& other)
    : RenderPoseToGeometryPose<T>(other.source_id_, other.frame_id_) {}

template <typename T>
RenderPoseToGeometryPose<T>::~RenderPoseToGeometryPose() = default;

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::RenderPoseToGeometryPose)
