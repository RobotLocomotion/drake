#include "drake/planning/robot_diagram_builder.h"

#include <stdexcept>

#include "drake/common/drake_throw.h"

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
      parser_(&plant_) {}

template <typename T>
RobotDiagramBuilder<T>::~RobotDiagramBuilder() = default;

template <typename T>
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::Build() {
  ThrowIfAlreadyBuilt();
  if (!plant().is_finalized()) {
    plant().Finalize();
  }
  // Unless the user has already customized the input or output ports,
  // by default it's convenient to export everything.
  if (builder_->num_input_ports() == 0 && builder_->num_output_ports() == 0) {
    ExportAllPorts();
  }
  return std::unique_ptr<RobotDiagram<T>>(
      new RobotDiagram<T>(std::move(builder_)));
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
void RobotDiagramBuilder<T>::ThrowIfAlreadyBuilt() const {
  if (IsDiagramBuilt()) {
    throw std::logic_error(
        "RobotDiagramBuilder: Build() has already been called to create a"
        " RobotDiagram; this RobotDiagramBuilder may no longer be used.");
  }
}

template <typename T>
void RobotDiagramBuilder<T>::ExportAllPorts() const {
  for (const System<T>* system : builder_->GetSystems()) {
    for (InputPortIndex i{0}; i < system->num_input_ports(); ++i) {
      const InputPort<T>& port = system->get_input_port(i);
      if (!builder_->IsConnectedOrExported(port)) {
        builder_->ExportInput(port);
      }
    }
    for (OutputPortIndex i{0}; i < system->num_output_ports(); ++i) {
      const OutputPort<T>& port = system->get_output_port(i);
      builder_->ExportOutput(port);
    }
  }
}

}  // namespace planning
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::planning::RobotDiagramBuilder)
