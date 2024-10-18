#include "drake/planning/configuration_space_obstacle_collision_checker.h"

#include <algorithm>
#include <functional>
#include <set>
#include <utility>
#include <iterator>

#include "drake/common/fmt_eigen.h"
#include "drake/common/copyable_unique_ptr.h"
#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram.h"
#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace planning {

using Eigen::RowVectorXd;
using Eigen::Vector3d;
using geometry::CollisionFilterDeclaration;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::GeometryInstance;
using geometry::GeometrySet;
using geometry::QueryObject;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::Shape;
using geometry::SignedDistancePair;
using math::RigidTransform;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::JacobianWrtVariable;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using systems::Context;

using geometry::optimization::ConvexSets;

ConfigurationSpaceObstacleCollisionChecker::ConfigurationSpaceObstacleCollisionChecker(
    copyable_unique_ptr<CollisionChecker> checker, 
    ConvexSets configuration_obstacles) : CollisionChecker(*checker), 
    checker_(std::move(checker)), 
    configuration_obstacles_(configuration_obstacles) {
}

ConfigurationSpaceObstacleCollisionChecker::ConfigurationSpaceObstacleCollisionChecker(
    const ConfigurationSpaceObstacleCollisionChecker&) = default;

void ConfigurationSpaceObstacleCollisionChecker::AddConfigurationSpaceObstacles(
    ConvexSets new_obstacles) {
  std::move(new_obstacles.begin(), new_obstacles.end(), 
      std::back_inserter(configuration_obstacles_));
}

void ConfigurationSpaceObstacleCollisionChecker::SetConfigurationSpaceObstacles(
    ConvexSets new_obstacles) {
  configuration_obstacles_.assign(
      std::make_move_iterator(new_obstacles.begin()), 
      std::make_move_iterator(new_obstacles.end()));
}

std::unique_ptr<CollisionChecker> ConfigurationSpaceObstacleCollisionChecker::
    DoClone() const {
  // N.B. We cannot use make_unique due to private-only access.
  return std::unique_ptr<ConfigurationSpaceObstacleCollisionChecker>(
      new ConfigurationSpaceObstacleCollisionChecker(*this));
}

void ConfigurationSpaceObstacleCollisionChecker::DoUpdateContextPositions(
    CollisionCheckerContext*) const {
  // No additional actions are required to update positions.
}

bool ConfigurationSpaceObstacleCollisionChecker::DoCheckContextConfigCollisionFree(
    const CollisionCheckerContext& model_context) const {
  if (not checker_->DoCheckContextConfigCollisionFree(model_context)) {
    return false;
  };

  for (const auto& obstacle_ptr : configuration_obstacles_) {
    if (obstacle_ptr->PointInSet(plant().GetPositions(
        model_context.plant_context()))) {
      return false;
    }
  }

  return true;
}

std::optional<GeometryId> ConfigurationSpaceObstacleCollisionChecker::
    DoAddCollisionShapeToBody(const std::string& group_name, 
    const RigidBody<double>& bodyA, const Shape& shape, 
    const RigidTransform<double>& X_AG) {
  return checker_->DoAddCollisionShapeToBody(group_name, bodyA, shape, X_AG);
}

void ConfigurationSpaceObstacleCollisionChecker::RemoveAddedGeometries(
    const std::vector<CollisionChecker::AddedShape>& shapes) {
  checker_->RemoveAddedGeometries(shapes);
}

void ConfigurationSpaceObstacleCollisionChecker::UpdateCollisionFilters() {
  checker_->UpdateCollisionFilters();
}

RobotClearance ConfigurationSpaceObstacleCollisionChecker::
    DoCalcContextRobotClearance(const CollisionCheckerContext& model_context,
    const double influence_distance) const {
  return checker_->DoCalcContextRobotClearance(model_context, 
       influence_distance);
}

std::vector<RobotCollisionType>
ConfigurationSpaceObstacleCollisionChecker::DoClassifyContextBodyCollisions(
    const CollisionCheckerContext& model_context) const {
  return checker_->DoClassifyContextBodyCollisions(model_context);
}

int ConfigurationSpaceObstacleCollisionChecker::DoMaxContextNumDistances(
    const CollisionCheckerContext& model_context) const {
  return checker_->DoMaxContextNumDistances(model_context);
}

}  // namespace planning
}  // namespace drake
