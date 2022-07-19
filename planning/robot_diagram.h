#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace anzu {
namespace planning {

/** Storage for a combined diagram, plant, and scene graph.

This class is a convenient syntactic sugar, especially in C++ code where it
simplifies object lifetime tracking and downcasting of the plant and scene graph
references.

@tparam_default_scalar */
template <typename T>
class RobotDiagram final : public drake::systems::Diagram<T> {
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
  using drake::systems::Diagram<T>::CreateDefaultContext;

  /** Gets the contained plant (readonly). */
  const drake::multibody::MultibodyPlant<T>& plant() const {
    return plant_;
  }

  /** Gets the contained scene graph (mutable). */
  drake::geometry::SceneGraph<T>& mutable_scene_graph() {
    return scene_graph_;
  }

  /** Gets the contained scene graph (readonly). */
  const drake::geometry::SceneGraph<T>& scene_graph() const {
    return scene_graph_;
  }

  /** Gets the contained plant's context (mutable) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  drake::systems::Context<T>& mutable_plant_context(
      drake::systems::Context<T>* root_context) const {
    return plant_.GetMyMutableContextFromRoot(root_context);
  }

  /** Gets the contained plant's context (readonly) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  const drake::systems::Context<T>& plant_context(
      const drake::systems::Context<T>& root_context) const {
    return plant_.GetMyContextFromRoot(root_context);
  }

  /** Gets the contained scene graph's context (mutable) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  drake::systems::Context<T>& mutable_scene_graph_context(
      drake::systems::Context<T>* root_context) const {
    return scene_graph_.GetMyMutableContextFromRoot(root_context);
  }

  /** Gets the contained scene graph's context (readonly) out of the given root
  context. Refer to drake::systems::System::GetMyContextFromRoot() to
  understand `root_context`.
  @throws std::exception if the `root_context` is not a root context. */
  const drake::systems::Context<T>& scene_graph_context(
      const drake::systems::Context<T>& root_context) const {
    return scene_graph_.GetMyContextFromRoot(root_context);
  }

 private:
  // To access our private constructor.
  template <typename> friend class RobotDiagramBuilder;

  // For use by RobotDiagramBuilder.
  explicit RobotDiagram(
      std::unique_ptr<drake::systems::DiagramBuilder<T>>);

  // Aliases for the plant and scene graph (which are owned by our base class).
  drake::multibody::MultibodyPlant<T>& plant_;
  drake::geometry::SceneGraph<T>& scene_graph_;
};

}  // namespace planning
}  // namespace anzu

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::anzu::planning::RobotDiagram)
