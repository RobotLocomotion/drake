#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "planning/robot_diagram.h"

namespace anzu {
namespace planning {

/** Storage for a combined diagram builder, plant, and scene graph.
When T == double, a parser (and package map) is also available.

This class is a convenient syntactic sugar to help build a robot diagram,
especially in C++ code where it simplifies object lifetime tracking and
downcasting of the plant and scene graph references.

@tparam_default_scalar */
template <typename T>
class RobotDiagramBuilder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotDiagramBuilder)

  /** Constructs with the specified time step for the contained plant. */
  explicit RobotDiagramBuilder(double time_step = 0.0);

  ~RobotDiagramBuilder();

  /** Gets the contained DiagramBuilder (mutable).
  @throws exception when IsDiagramBuilt() already. */
  drake::systems::DiagramBuilder<T>& mutable_builder() {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return *builder_;
  }

  /** Gets the contained DiagramBuilder (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const drake::systems::DiagramBuilder<T>& builder() const {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return *builder_;
  }

  /** Gets the contained Parser (mutable).
  @throws exception when IsDiagramBuilt() already. */
  template <typename T1 = T, typename std::enable_if_t<
    std::is_same_v<T1, double>>* = nullptr>
  drake::multibody::Parser& mutable_parser() {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return parser_;
  }

  /** Gets the contained Parser (readonly).
  @throws exception when IsDiagramBuilt() already. */
  template <typename T1 = T, typename std::enable_if_t<
    std::is_same_v<T1, double>>* = nullptr>
  const drake::multibody::Parser& parser() const {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return parser_;
  }

  /** Gets the contained plant (mutable).
  @throws exception when IsDiagramBuilt() already. */
  drake::multibody::MultibodyPlant<T>& mutable_plant() {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return plant_;
  }

  /** Gets the contained plant (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const drake::multibody::MultibodyPlant<T>& plant() const {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return plant_;
  }

  /** Gets the contained scene graph (mutable).
  @throws exception when IsDiagramBuilt() already. */
  drake::geometry::SceneGraph<T>& mutable_scene_graph() {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return scene_graph_;
  }

  /** Gets the contained scene graph (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const drake::geometry::SceneGraph<T>& scene_graph() const {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return scene_graph_;
  }

  /** Checks if the contained plant is finalized.
  @throws exception when IsDiagramBuilt() already. */
  bool IsPlantFinalized() const {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    return plant_.is_finalized();
  }

  /** Finalizes the contained plant.
  @throws exception when IsDiagramBuilt() already. */
  void FinalizePlant() {
    DRAKE_THROW_UNLESS(!IsDiagramBuilt());
    plant_.Finalize();
  }

  /** Checks if the diagram has already been built. */
  bool IsDiagramBuilt() const {
    return builder_ == nullptr;
  }

  /** Builds the diagram and returns the diagram plus plant and scene graph in a
  RobotDiagram. The plant will be finalized during this function, unless it's
  already been finalized.
  @throws exception when IsDiagramBuilt() already. */
  std::unique_ptr<RobotDiagram<T>> BuildDiagram();

 private:
  // Storage for the diagram and its plant and scene graph.
  // After building, the `builder_` is set to nullptr.
  std::unique_ptr<drake::systems::DiagramBuilder<T>> builder_;
  drake::multibody::AddMultibodyPlantSceneGraphResult<T> pair_;
  drake::multibody::MultibodyPlant<T>& plant_;
  drake::geometry::SceneGraph<T>& scene_graph_;

  // The Parser object only exists when T == double.
  using MaybeParser = std::conditional_t<
      std::is_same_v<T, double>, drake::multibody::Parser, void*>;
  MaybeParser parser_;
};

}  // namespace planning
}  // namespace anzu

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::anzu::planning::RobotDiagramBuilder)
