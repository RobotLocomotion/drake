#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/pointer_cast.h"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
MultibodyPositionToGeometryPose<T>::MultibodyPositionToGeometryPose(
    const multibody::MultibodyPlant<T>& plant, bool input_multibody_state)
    : plant_(plant) {
  Configure(input_multibody_state);
}

// Note: This constructor is not *obviously* correct. Compare it with this code:
//   unique_ptr<Foo> f;
//   bar(*f, move(f));
// The invocation to bar may not be valid because the compiler can chose to
// perform the move of f before the dereference (which would make the first
// parameter a reference to null). However, this constructor works because:
// "... non-static data members are initialized in order of declaration in the
// class definition."
// https://en.cppreference.com/w/cpp/language/initializer_list#Initialization_order
template <typename T>
MultibodyPositionToGeometryPose<T>::MultibodyPositionToGeometryPose(
    std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
    bool input_multibody_state)
    : plant_(*owned_plant), owned_plant_(std::move(owned_plant)) {
  DRAKE_DEMAND(owned_plant_ != nullptr);
  Configure(input_multibody_state);
}

template <typename T>
void MultibodyPositionToGeometryPose<T>::Configure(
    bool input_multibody_state) {
  // Either we don't own the plant, or we own the plant we're storing the
  // reference for.
  DRAKE_DEMAND(owned_plant_ == nullptr || owned_plant_.get() == &plant_);
  if (!plant_.is_finalized()) {
    throw std::logic_error(
        "MultibodyPositionToGeometryPose requires a MultibodyPlant that has "
        "been finalized");
  }
  if (!plant_.geometry_source_is_registered()) {
    throw std::logic_error(
        "MultibodyPositionToGeometryPose requires a MultibodyPlant that has "
        "been registered with a SceneGraph");
  }
  plant_context_ = plant_.CreateDefaultContext();

  this->DeclareInputPort("position", kVectorValued,
                         input_multibody_state ? plant_.num_multibody_states()
                                          : plant_.num_positions());
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
      this->get_input_port().Eval(context).head(plant_.num_positions());

  // Evaluate the plant's output port.
  plant_.get_geometry_poses_output_port().Calc(*plant_context_, output);
}

template class MultibodyPositionToGeometryPose<double>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
