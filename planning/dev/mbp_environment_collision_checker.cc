#include "planning/mbp_environment_collision_checker.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/print.hpp>

#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::FrameId;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::GeometrySet;
using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::planning::CollisionChecker;
using drake::planning::CollisionCheckerContext;
using std::optional;

namespace anzu {
namespace planning {

MbpEnvironmentCollisionChecker::MbpEnvironmentCollisionChecker(
    drake::planning::CollisionCheckerParams params)
    : SphereRobotModelCollisionChecker(std::move(params)) {
  AllocateContexts();
}

MbpEnvironmentCollisionChecker::MbpEnvironmentCollisionChecker(
    const MbpEnvironmentCollisionChecker&) = default;

std::unique_ptr<CollisionChecker>
MbpEnvironmentCollisionChecker::DoClone() const {
  // N.B. We cannot use make_unique due to protected-only access.
  return std::unique_ptr<MbpEnvironmentCollisionChecker>(
      new MbpEnvironmentCollisionChecker(*this));
}

PointSignedDistanceAndGradientResult
MbpEnvironmentCollisionChecker::
    ComputePointToEnvironmentSignedDistanceAndGradient(
        const drake::systems::Context<double>& context,
        const drake::geometry::QueryObject<double>& query_object,
        const Eigen::Vector4d& p_WQ, const double query_radius,
        const std::vector<Eigen::Isometry3d>&,
        const std::vector<Eigen::Isometry3d>&) const {
  const drake::geometry::SceneGraphInspector<double>& inspector =
      query_object.inspector();
  // Query signed distances.
  const std::vector<drake::geometry::SignedDistanceToPoint<double>> distances =
      query_object.ComputeSignedDistanceToPoint(p_WQ.head<3>(), query_radius);
  // Compile distances and gradients.
  PointSignedDistanceAndGradientResult result;
  // Check each of the returned distance+gradients.
  for (const auto& distance_query : distances) {
    // TODO(calderpg) Remove once better filtering for query is available.
    if (RobotGeometries().count(distance_query.id_G) == 0) {
      const drake::multibody::BodyIndex colliding_body_index =
          plant().GetBodyFromFrameId(
              inspector.GetFrameId(distance_query.id_G))->index();
      const double distance = distance_query.distance;
      const Eigen::Vector4d gradient(distance_query.grad_W.x(),
                                     distance_query.grad_W.y(),
                                     distance_query.grad_W.z(),
                                     0.0);
      result.AddDistanceAndGradient(distance, gradient, colliding_body_index);
    }
  }
  return result;
}

optional<GeometryId>
MbpEnvironmentCollisionChecker::AddEnvironmentCollisionShapeToBody(
    const std::string& group_name, const drake::multibody::Body<double>& bodyA,
    const drake::geometry::Shape& shape,
    const drake::math::RigidTransform<double>& X_AG) {
  const std::optional<FrameId> body_frame_id =
      plant().GetBodyFrameIdIfExists(bodyA.index());
  if (!body_frame_id.has_value()) {
    drake::log()->warn(
        "Cannot add collision geometry because the body to which it is attached"
        " ({}) does not have any previously registered collision geometry.",
        bodyA.scoped_name());
    return std::nullopt;
  }

  const GeometrySet bodyA_geometries =
      plant().CollectRegisteredGeometries(plant().GetBodiesWeldedTo(bodyA));
  drake::log()->debug(
      "Adding shape (group: [{}]) to {} (FrameID {}) at X_AG =\n{}",
      group_name, bodyA.scoped_name(), body_frame_id.value(),
      X_AG.GetAsMatrix4());

  // The geometry instance template which will be copied into each per-thread
  // SceneGraph Context; the GeometryId will match across each thread this way.
  GeometryInstance geometry_template(X_AG, shape.Clone(), "temp");
  geometry_template.set_name(fmt::format("Added collision geometry on {} ({})",
                                         bodyA.scoped_name(),
                                         geometry_template.id()));
  geometry_template.set_proximity_properties({});

  using Operation = std::function<void(
      const RobotDiagram<double>&, CollisionCheckerContext*)>;
  const Operation operation = [&](const RobotDiagram<double>& model,
                                  CollisionCheckerContext* model_context) {
    auto& sg_context = model_context->mutable_scene_graph_context();
    const GeometryId added_geometry_id = model.scene_graph().RegisterGeometry(
        &sg_context, model.plant().get_source_id().value(),
        body_frame_id.value(),
        std::make_unique<GeometryInstance>(geometry_template));
    model.scene_graph()
        .collision_filter_manager(&sg_context)
        .Apply(drake::geometry::CollisionFilterDeclaration().ExcludeBetween(
            bodyA_geometries, GeometrySet{added_geometry_id}));
  };
  PerformOperationAgainstAllModelContexts(operation);

  return geometry_template.id();
}

void MbpEnvironmentCollisionChecker::RemoveAllAddedEnvironment(
    const std::vector<CollisionChecker::AddedShape>& shapes) {
  const std::function<void(const RobotDiagram<double>&,
                           CollisionCheckerContext*)>
      operation = [&](const RobotDiagram<double>& model,
                      CollisionCheckerContext* model_context) {
        for (const auto& shape : shapes) {
          if (IsPartOfRobot(shape.body_index)) continue;
          drake::log()->debug("Removing geometry {}.", shape.geometry_id);
          model.scene_graph().RemoveGeometry(
              &model_context->mutable_scene_graph_context(),
              model.plant().get_source_id().value(), shape.geometry_id);
        }
      };
  PerformOperationAgainstAllModelContexts(operation);
}

}  // namespace planning
}  // namespace anzu
