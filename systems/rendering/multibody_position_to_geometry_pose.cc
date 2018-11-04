#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/pointer_cast.h"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
MultibodyPositionToGeometryPose<T>::MultibodyPositionToGeometryPose(
    const multibody::multibody_plant::MultibodyPlant<T>& plant)
    : plant_(plant),
      plant_context_(
          dynamic_pointer_cast_or_throw<multibody::MultibodyTreeContext<T>>(
              plant.CreateDefaultContext())) {
  DRAKE_DEMAND(plant.is_finalized());
  DRAKE_DEMAND(plant.geometry_source_is_registered());

  this->DeclareInputPort("position", kVectorValued, plant.num_positions());
  this->DeclareAbstractOutputPort(
      "geometry_pose",
      [this]() {
        return this->plant_.get_geometry_poses_output_port().Allocate();
      },
      [this](const Context<T>& context, AbstractValue* output) {
        return this->CalcGeometryPose(context, output);
      });
}

template <typename T>
void MultibodyPositionToGeometryPose<T>::CalcGeometryPose(
    const Context<T>& context, AbstractValue* output) const {
  // Set the positions in the owned (mutable) context so that we can ask the
  // MultibodyPlant to compute the outputs.
  plant_context_->get_mutable_positions() =
      this->EvalEigenVectorInput(context, 0);

  // Evaluate the plant's output port.  This output port on MultibodyPlant
  // should only depend on the positions in the Context (not any inputs, etc).
  // This call will fail if it ever tried to evaluate a plant input port,
  // since we have not wired any of them up in the `plant_context_`.
  plant_.get_geometry_poses_output_port().Calc(*plant_context_, output);
}

template class MultibodyPositionToGeometryPose<double>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
