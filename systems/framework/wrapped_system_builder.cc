/* clang-format off to disable clang-format-includes */
#include "drake/systems/framework/wrapped_system.h"
/* clang-format on */

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace systems {
namespace internal {

// This constructor for wrapped_system.h must be defined in a separate file,
// because it uses DiagramBuilder.
template <typename T>
template <typename>
WrappedSystem<T>::WrappedSystem(std::shared_ptr<System<T>> system)
    : Diagram<T>(SystemTypeTag<WrappedSystem>{}) {
  DRAKE_THROW_UNLESS(system != nullptr);
  this->set_name(system->get_name());
  system->set_name("wrapped");
  DiagramBuilder<T> builder;
  const System<T>* alias = builder.AddSystem(std::move(system));
  for (InputPortIndex i{0}; i < alias->num_input_ports(); ++i) {
    const InputPort<T>& port =
        alias->get_input_port(i, /* warn_deprecated = */ false);
    builder.ExportInput(port);
  }
  for (OutputPortIndex i{0}; i < alias->num_output_ports(); ++i) {
    const OutputPort<T>& port =
        alias->get_output_port(i, /* warn_deprecated = */ false);
    builder.ExportOutput(port);
  }
  builder.BuildInto(this);
}

template WrappedSystem<double>::WrappedSystem(std::shared_ptr<System<double>>);
template WrappedSystem<AutoDiffXd>::WrappedSystem(
    std::shared_ptr<System<AutoDiffXd>>);
template WrappedSystem<symbolic::Expression>::WrappedSystem(
    std::shared_ptr<System<symbolic::Expression>>);

}  // namespace internal
}  // namespace systems
}  // namespace drake
