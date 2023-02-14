#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_deprecated.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
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
  Do not call Build() on the return value; instead, call Build() on this.
  @throws exception when IsDiagramBuilt() already. */
  systems::DiagramBuilder<T>& builder() {
    ThrowIfAlreadyBuilt();
    return *builder_;
  }

  /** Gets the contained DiagramBuilder (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const systems::DiagramBuilder<T>& builder() const {
    ThrowIfAlreadyBuilt();
    return *builder_;
  }

  /** Gets the contained Parser (mutable).
  @throws exception when IsDiagramBuilt() already. */
  template <typename T1 = T,
            typename std::enable_if_t<std::is_same_v<T1, double>>* = nullptr>
  multibody::Parser& parser() {
    ThrowIfAlreadyBuilt();
    return parser_;
  }

  /** Gets the contained Parser (readonly).
  @throws exception when IsDiagramBuilt() already. */
  template <typename T1 = T,
            typename std::enable_if_t<std::is_same_v<T1, double>>* = nullptr>
  const multibody::Parser& parser() const {
    ThrowIfAlreadyBuilt();
    return parser_;
  }

  /** Gets the contained plant (mutable).
  @throws exception when IsDiagramBuilt() already. */
  multibody::MultibodyPlant<T>& plant() {
    ThrowIfAlreadyBuilt();
    return plant_;
  }

  /** Gets the contained plant (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const multibody::MultibodyPlant<T>& plant() const {
    ThrowIfAlreadyBuilt();
    return plant_;
  }

  /** Gets the contained scene graph (mutable).
  @throws exception when IsDiagramBuilt() already. */
  geometry::SceneGraph<T>& scene_graph() {
    ThrowIfAlreadyBuilt();
    return scene_graph_;
  }

  /** Gets the contained scene graph (readonly).
  @throws exception when IsDiagramBuilt() already. */
  const geometry::SceneGraph<T>& scene_graph() const {
    ThrowIfAlreadyBuilt();
    return scene_graph_;
  }

  /** Checks if the diagram has already been built. */
  bool IsDiagramBuilt() const;

  /** Builds the diagram and returns the diagram plus plant and scene graph in a
  RobotDiagram. The plant will be finalized during this function, unless it's
  already been finalized.
  @throws exception when IsDiagramBuilt() already. */
  std::unique_ptr<RobotDiagram<T>> Build();

  DRAKE_DEPRECATED("2023-06-01", "Use Build() instead of BuildDiagram().")
  std::unique_ptr<RobotDiagram<T>> BuildDiagram() { return Build(); }

  DRAKE_DEPRECATED("2023-06-01", "Use builder() instead of mutable_builder().")
  systems::DiagramBuilder<T>& mutable_builder() { return builder(); }

  template <typename T1 = T,
            typename std::enable_if_t<std::is_same_v<T1, double>>* = nullptr>
  DRAKE_DEPRECATED("2023-06-01", "Use parser() instead of mutable_parser().")
  multibody::Parser& mutable_parser() {
    return parser();
  }

  DRAKE_DEPRECATED("2023-06-01", "Use plant() instead of mutable_plant().")
  multibody::MultibodyPlant<T>& mutable_plant() { return plant(); }

  DRAKE_DEPRECATED("2023-06-01",
                   "Use scene_graph() instead of mutable_scene_graph().")
  geometry::SceneGraph<T>& mutable_scene_graph() { return scene_graph(); }

  DRAKE_DEPRECATED("2023-06-01",
                   "Use plant().is_finalized() instead of IsPlantFinalized().")
  bool IsPlantFinalized() const {
    ThrowIfAlreadyBuilt();
    return plant_.is_finalized();
  }

  DRAKE_DEPRECATED("2023-06-01",
                   "Use plant().Finalize() instead of FinalizePlant().")
  void FinalizePlant() {
    ThrowIfAlreadyBuilt();
    plant_.Finalize();
  }

 private:
  void ThrowIfAlreadyBuilt() const;

  // Storage for the diagram and its plant and scene graph.
  // After Build(), the `builder_` is set to nullptr.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_;
  multibody::AddMultibodyPlantSceneGraphResult<T> pair_;
  multibody::MultibodyPlant<T>& plant_;
  geometry::SceneGraph<T>& scene_graph_;

  // The Parser object only exists when T == double.
  using MaybeParser =
      std::conditional_t<std::is_same_v<T, double>, multibody::Parser, void*>;
  MaybeParser parser_;
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::planning::RobotDiagramBuilder)
