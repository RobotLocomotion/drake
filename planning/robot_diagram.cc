#include "planning/robot_diagram.h"

#include <vector>

namespace anzu {
namespace planning {

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::System;
using drake::systems::SystemTypeTag;

// These are a consequence of AddMultibodyPlantSceneGraph.
constexpr size_t kPlantIndex = 0;
constexpr size_t kSceneGraphIndex = 1;

// Returns the index'th subsystem from the given diagram, downcasting it to
// the requested subtype.
//
// @pre The index'th system actually exists and is of the correct subtype.
//
// @tparam_default_scalar
// @tparam ChildSystem the subsystem class to extract, e.g., MultibodyPlant.
// @tparam DiagramOrDiagramBuilder duck type for either a diagram or a builder.
template <
  typename T,
  template <typename> class ChildSystem,
  template <typename> class DiagramOrDiagramBuilder>
ChildSystem<T>& DowncastSubsystem(
    DiagramOrDiagramBuilder<T>* diagram, size_t index) {
  DRAKE_DEMAND(diagram != nullptr);
  const std::vector<const System<T>*>& items = diagram->GetSystems();
  const auto* child = dynamic_cast<const ChildSystem<T>*>(items.at(index));
  DRAKE_DEMAND(child != nullptr);
  return const_cast<ChildSystem<T>&>(*child);
}

template <typename T>
RobotDiagram<T>::RobotDiagram(
    std::unique_ptr<DiagramBuilder<T>> diagram_builder)
    : Diagram<T>(SystemTypeTag<RobotDiagram>{}),
      plant_(DowncastSubsystem<T, MultibodyPlant>(
          diagram_builder.get(), kPlantIndex)),
      scene_graph_(DowncastSubsystem<T, SceneGraph>(
          diagram_builder.get(), kSceneGraphIndex)) {
  diagram_builder->BuildInto(this);
  // TODO(jeremy.nimmer) For convenience, we should probably re-export most (or
  // all) of the the subsytems' input and output ports here.
}

template <typename T>
RobotDiagram<T>::~RobotDiagram() = default;

template <typename T>
template <typename U>
RobotDiagram<T>::RobotDiagram(const RobotDiagram<U>& other)
    : Diagram<T>(other),
      plant_(DowncastSubsystem<T, MultibodyPlant>(this, kPlantIndex)),
      scene_graph_(DowncastSubsystem<T, SceneGraph>(this, kSceneGraphIndex)) {}

}  // namespace planning
}  // namespace anzu

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::anzu::planning::RobotDiagram)
