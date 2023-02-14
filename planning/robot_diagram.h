#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace planning {

/** Storage for a combined diagram, plant, and scene graph.

This class is a convenient syntactic sugar, especially in C++ code where it
simplifies object lifetime tracking and downcasting of the plant and scene graph
references. It's purpose is to serve as planner-specific syntactic sugar for
operating on a MultibodyPlant. For other purposes (e.g., simulation), users
should generally prefer to just use a Diagram, instead.

@tparam_default_scalar */
template <typename T>
class RobotDiagram final : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RobotDiagram)

  // There are no user-serviceable constructors for this class. To make an
  // instance of this class, use RobotDiagramBuilder.

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit RobotDiagram(const RobotDiagram<U>&);

  ~RobotDiagram() final;

  /** Creates a default context for this diagram.
  This is one way to create a valid `root_context` argument to pass to the
  context-related helper functions below. */
  using systems::Diagram<T>::CreateDefaultContext;

  /** Gets the contained plant (readonly). */
  const multibody::MultibodyPlant<T>& plant() const { return plant_; }

  /** Gets the contained scene graph (mutable). */
  geometry::SceneGraph<T>& mutable_scene_graph() { return scene_graph_; }

  /** Gets the contained scene graph (readonly). */
  const geometry::SceneGraph<T>& scene_graph() const { return scene_graph_; }

  /** Gets the contained plant's context (mutable) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  systems::Context<T>& mutable_plant_context(
      systems::Context<T>* root_context) const {
    return plant_.GetMyMutableContextFromRoot(root_context);
  }

  /** Gets the contained plant's context (readonly) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  const systems::Context<T>& plant_context(
      const systems::Context<T>& root_context) const {
    return plant_.GetMyContextFromRoot(root_context);
  }

  /** Gets the contained scene graph's context (mutable) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  systems::Context<T>& mutable_scene_graph_context(
      systems::Context<T>* root_context) const {
    return scene_graph_.GetMyMutableContextFromRoot(root_context);
  }

  /** Gets the contained scene graph's context (readonly) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  const systems::Context<T>& scene_graph_context(
      const systems::Context<T>& root_context) const {
    return scene_graph_.GetMyContextFromRoot(root_context);
  }

 private:
  // To access our private constructor.
  template <typename>
  friend class RobotDiagramBuilder;

  // For use by RobotDiagramBuilder.
  explicit RobotDiagram(std::unique_ptr<systems::DiagramBuilder<T>>);

  // Aliases for the plant and scene graph (which are owned by our base class).
  multibody::MultibodyPlant<T>& plant_;
  geometry::SceneGraph<T>& scene_graph_;
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::planning::RobotDiagram)
