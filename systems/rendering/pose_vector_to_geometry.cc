#include "drake/systems/rendering/pose_vector_to_geometry.h"

#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

using drake::geometry::FrameId;
using drake::geometry::SourceId;

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseVectorToGeometry<T>::PoseVectorToGeometry(
    SourceId source_id, FrameId frame_id)
    : LeafSystem<T>(SystemTypeTag<rendering::PoseVectorToGeometry>{}),
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
        const Input* input =
            this->template EvalVectorInput<PoseVector>(context, 0);
        DRAKE_DEMAND(input != nullptr);
        Output& output = calculated->GetMutableValueOrThrow<Output>();
        output.clear();
        output.set_value(frame_id, input->get_isometry());
      });
}

template <typename T>
template <typename U>
PoseVectorToGeometry<T>::PoseVectorToGeometry(
    const PoseVectorToGeometry<U>& other)
    : PoseVectorToGeometry<T>(other.source_id_, other.frame_id_) {}

template <typename T>
PoseVectorToGeometry<T>::~PoseVectorToGeometry() = default;

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::PoseVectorToGeometry)
