#include "drake/systems/controllers/state_feedback_controller_interface.h"

// Define the first virtual funciton of the class to emit the vtable
// only in this translation unit.
namespace drake {
namespace systems {
namespace controllers {

template <typename T>
StateFeedbackControllerInterface<T>::~StateFeedbackControllerInterface() =
    default;

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::StateFeedbackControllerInterface);
