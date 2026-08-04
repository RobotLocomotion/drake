#include "drake/planning/robot_diagram_builder.h"

#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace planning {

using geometry::SceneGraph;
using multibody::AddMultibodyPlantSceneGraph;
using systems::DiagramBuilder;
using systems::InputPort;
using systems::InputPortIndex;
using systems::OutputPort;
using systems::OutputPortIndex;
using systems::System;

template <typename T>
RobotDiagramBuilder<T>::RobotDiagramBuilder(double time_step)
    : builder_(std::make_unique<DiagramBuilder<T>>()),
      pair_(AddMultibodyPlantSceneGraph<T>(builder_.get(), time_step)),
      plant_(pair_.plant),
      scene_graph_(pair_.scene_graph),
      parser_(builder_.get()) {}

template <typename T>
RobotDiagramBuilder<T>::~RobotDiagramBuilder() = default;

template <typename T>
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::Build() {
  internal::BuildResultForPython<T> result = BuildForPython();
  // Return the diagram only, allowing the result struct holding the builder to
  // delete the builder as it goes out of scope.
  return std::move(result.diagram);
}

template <typename T>
internal::BuildResultForPython<T> RobotDiagramBuilder<T>::BuildForPython() {
  ThrowIfAlreadyBuiltOrCorrupted();
  if (!plant().is_finalized()) {
    plant().Finalize();
  }
  // Unless the user has customized anything, by default it's convenient to
  // export everything.
  if (ShouldExportDefaultPorts()) {
    ExportDefaultPorts();
  }
  internal::BuildResultForPython<T> result;
  result.diagram =
      std::unique_ptr<RobotDiagram<T>>(new RobotDiagram<T>(builder_.get()));
  result.builder = std::move(builder_);
  return result;
}

template <typename T>
bool RobotDiagramBuilder<T>::IsDiagramBuilt() const {
  if (builder_ == nullptr) {
    return true;
  }
  if (builder_->already_built()) {
    throw std::logic_error(
        "RobotDiagramBuilder: Do not call mutable_builder().Build() to create"
        " a Diagram; instead, call Build() to create a RobotDiagram.");
  }
  return false;
}

template <typename T>
void RobotDiagramBuilder<T>::ThrowIfAlreadyBuiltOrCorrupted() const {
  if (IsDiagramBuilt()) {
    throw std::logic_error(
        "RobotDiagramBuilder: Build() has already been called to create a"
        " RobotDiagram; this RobotDiagramBuilder may no longer be used.");
  }
  // Check that the user didn't remove (delete) the plant or scene graph.
  std::vector<const System<T>*> systems = builder_->GetSystems();
  const bool is_good = systems.size() >= 2 && systems[0] == &plant_ &&
                       systems[1] == &scene_graph_;
  if (!is_good) {
    throw std::logic_error(
        "RobotDiagramBuilder: The underlying DiagramBuilder has become "
        "corrupted. You must not remove the MultibodyPlant or SceneGraph.");
  }
}

/* Checks for whether or not the user has customized anything that would impact
the default counts or names of exported ports. */
template <typename T>
bool RobotDiagramBuilder<T>::ShouldExportDefaultPorts() const {
  return plant().get_name() == "plant" &&
         scene_graph().get_name() == "scene_graph" &&
         builder_->GetSystems().size() == 2 &&
         builder_->num_input_ports() == 0 && builder_->num_output_ports() == 0;
}

template <typename T>
void RobotDiagramBuilder<T>::ExportDefaultPorts() const {
  // Export ports per the contract in the RobotDiagram class overview.
  for (const System<T>* system : builder_->GetSystems()) {
    for (InputPortIndex i{0}; i < system->num_input_ports(); ++i) {
      const InputPort<T>& port =
          system->get_input_port(i, /* warn_deprecated = */ false);
      if (port.get_deprecation().has_value()) {
        continue;  // Don't export deprecated ports.
      }
      if (builder_->IsConnectedOrExported(port)) {
        // We can't export this input because it's internally-sourced already.
        continue;  // Skip just this one input port.
      }
      builder_->ExportInput(port);
    }
    for (OutputPortIndex i{0}; i < system->num_output_ports(); ++i) {
      const OutputPort<T>& port =
          system->get_output_port(i, /* warn_deprecated = */ false);
      if (port.get_deprecation().has_value()) {
        continue;  // Don't export deprecated ports.
      }
      builder_->ExportOutput(port);
    }
  }
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::planning::RobotDiagramBuilder);
