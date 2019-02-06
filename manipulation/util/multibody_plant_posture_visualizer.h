#pragma once

#include <memory>
#include <string>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace manipulation {
class MultibodyPlantPostureVisualizer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlantPostureVisualizer)

  explicit MultibodyPlantPostureVisualizer(const std::string& file_path);

  /**
   * @param plant The plant to be visualized. This plant must have been
   * registered as the source for @p scene_graph, by calling
   * plant.RegisterAsSouceForSceneGraph(scne_graph.get()).
   * @param scene_graph The SceneGraph containing the geometric information for
   * @p plant.
   */
  MultibodyPlantPostureVisualizer(
      const multibody::MultibodyPlant<double>& plant,
      std::unique_ptr<geometry::SceneGraph<double>> scene_graph);

  void VisualizePosture(const Eigen::Ref<const Eigen::VectorXd>& q);

 private:
  void BuildVisualizer(
      const multibody::MultibodyPlant<double>& plant,
      std::unique_ptr<geometry::SceneGraph<double>> scene_graph,
      systems::DiagramBuilder<double>* builder);

  std::unique_ptr<multibody::MultibodyPlant<double>> owned_plant_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  systems::ConstantVectorSource<double>* posture_source_;
  std::unique_ptr<systems::Simulator<double>> simulator_;
};
}  // namespace manipulation
}  // namespace drake
