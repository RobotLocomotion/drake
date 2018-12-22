#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/pointer_cast.h"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
MultibodyPositionToGeometryPose<T>::MultibodyPositionToGeometryPose(
    const multibody::MultibodyPlant<T>& plant)
    : plant_(plant),
      plant_context_(plant.CreateDefaultContext()) {
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

  // Fix all input ports in the Context to avoid leaving them unassigned.
  // They should not impact the output values.
  plant_.AllocateFixedInputs(plant_context_.get());
}

template <typename T>
void MultibodyPositionToGeometryPose<T>::CalcGeometryPose(
    const Context<T>& context, AbstractValue* output) const {
  // Set the positions in the owned (mutable) context so that we can ask the
  // MultibodyPlant to compute the outputs.
  // TODO(eric.cousineau): Place `plant_context_` in the cache of `context`,
  // and remove mutable member.
  plant_.GetMutablePositions(plant_context_.get()) =
      this->EvalEigenVectorInput(context, 0);

  // Evaluate the plant's output port.
  plant_.get_geometry_poses_output_port().Calc(*plant_context_, output);
}

template class MultibodyPositionToGeometryPose<double>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
