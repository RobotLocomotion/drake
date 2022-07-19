#include "planning/robot_diagram_builder.h"

namespace anzu {
namespace planning {

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::systems::DiagramBuilder;
using drake::systems::System;

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
std::unique_ptr<RobotDiagram<T>> RobotDiagramBuilder<T>::BuildDiagram() {
  DRAKE_THROW_UNLESS(!IsDiagramBuilt());
  if (!IsPlantFinalized()) {
    FinalizePlant();
  }
  return std::unique_ptr<RobotDiagram<T>>(
      new RobotDiagram<T>(std::move(builder_)));
}

}  // namespace planning
}  // namespace anzu

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::anzu::planning::RobotDiagramBuilder)
