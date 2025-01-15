#include "drake/planning/scene_graph_collision_checker.h"

#include <functional>
#include <set>
#include <utility>

#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram.h"

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

SceneGraphCollisionChecker::SceneGraphCollisionChecker(
    CollisionCheckerParams params)
    : CollisionChecker(std::move(params), true /* supports parallel */) {
  AllocateContexts();
  // Enforce that all filters known to the collision checker are active in
  // SceneGraph, as CollisionChecker introduces additional filters that may not
  // already be present in SceneGraph (e.g. environment-environment body pairs).
  ApplyCollisionFiltersToSceneGraph();
}

SceneGraphCollisionChecker::SceneGraphCollisionChecker(
    const SceneGraphCollisionChecker&) = default;

std::unique_ptr<CollisionChecker> SceneGraphCollisionChecker::DoClone() const {
  // N.B. We cannot use make_unique due to private-only access.
  return std::unique_ptr<SceneGraphCollisionChecker>(
      new SceneGraphCollisionChecker(*this));
}

void SceneGraphCollisionChecker::DoUpdateContextPositions(
    CollisionCheckerContext*) const {
  // No additional actions are required to update positions.
}

std::optional<GeometryId> SceneGraphCollisionChecker::DoAddCollisionShapeToBody(
    const std::string& group_name, const RigidBody<double>& bodyA,
    const Shape& shape, const RigidTransform<double>& X_AG) {
  const FrameId body_frame_id = plant().GetBodyFrameIdOrThrow(bodyA.index());

  const GeometrySet bodyA_geometries =
      plant().CollectRegisteredGeometries(plant().GetBodiesWeldedTo(bodyA));
  log()->debug("Adding shape (group: [{}]) to {} (FrameID {}) at X_AG =\n{}",
               group_name, bodyA.scoped_name(), body_frame_id,
               fmt_eigen(X_AG.GetAsMatrix4()));

  // The geometry instance template which will be copied into each per-thread
  // SceneGraph Context; the GeometryId will match across each thread this way.
  GeometryInstance geometry_template(X_AG, shape.Clone(), "temp");
  geometry_template.set_name(fmt::format("Added collision geometry on {} ({})",
                                         bodyA.scoped_name(),
                                         geometry_template.id()));
  // Set proximity properties in order for this geometry to actually collide.
  geometry_template.set_proximity_properties({});

  const auto operation = [&](const RobotDiagram<double>& model,
                             CollisionCheckerContext* model_context) {
    auto& sg_context = model_context->mutable_scene_graph_context();
    model.scene_graph().RegisterGeometry(
        &sg_context, model.plant().get_source_id().value(), body_frame_id,
        std::make_unique<GeometryInstance>(geometry_template));
  };
  PerformOperationAgainstAllModelContexts(operation);

  // Ensure that filters in SceneGraph cover the new geometry, including the
  // within-body filter for the new geometry.
  ApplyCollisionFiltersToSceneGraph();

  return geometry_template.id();
}

void SceneGraphCollisionChecker::RemoveAddedGeometries(
    const std::vector<CollisionChecker::AddedShape>& shapes) {
  const auto operation = [&shapes](const RobotDiagram<double>& model,
                                   CollisionCheckerContext* model_context) {
    for (const auto& checker_shape : shapes) {
      const GeometryId geometry_id = checker_shape.geometry_id;
      // Our public NVI wrapper logs "Removing geometries from group..." with
      // no indentation; we'll whitespace-indent our detail logging under it.
      log()->debug("  Removing geometry {}.", geometry_id);
      model.scene_graph().RemoveGeometry(
          &model_context->mutable_scene_graph_context(),
          model.plant().get_source_id().value(), geometry_id);
    }
  };

  PerformOperationAgainstAllModelContexts(operation);
}

void SceneGraphCollisionChecker::UpdateCollisionFilters() {
  // Apply changes to the collision filters to SceneGraph.
  ApplyCollisionFiltersToSceneGraph();
}

bool SceneGraphCollisionChecker::DoCheckContextConfigCollisionFree(
    const CollisionCheckerContext& model_context) const {
  const QueryObject<double>& query_object = model_context.GetQueryObject();
  const SceneGraphInspector<double>& inspector = query_object.inspector();

  const std::vector<SignedDistancePair<double>>& distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          GetLargestPadding());

  for (const auto& distance_pair : distance_pairs) {
    // Get the bodies corresponding to the distance pair.
    const FrameId frame_id_A = inspector.GetFrameId(distance_pair.id_A);
    const FrameId frame_id_B = inspector.GetFrameId(distance_pair.id_B);
    const RigidBody<double>* body_A = plant().GetBodyFromFrameId(frame_id_A);
    const RigidBody<double>* body_B = plant().GetBodyFromFrameId(frame_id_B);
    DRAKE_THROW_UNLESS(body_A != nullptr);
    DRAKE_THROW_UNLESS(body_B != nullptr);
    // Enforce that our collision filters are consistent with query results.
    if (IsCollisionFilteredBetween(*body_A, *body_B)) {
      throw std::runtime_error(fmt::format(
          "Drake internal error at {}:{} in {}(): Collision between bodies [{}]"
          " and [{}] should already be filtered",
          __FILE__, __LINE__, __func__, body_A->scoped_name(),
          body_B->scoped_name()));
    }
    const bool body_A_part_of_robot = IsPartOfRobot(*body_A);
    const bool body_B_part_of_robot = IsPartOfRobot(*body_B);
    DRAKE_ASSERT(body_A_part_of_robot || body_B_part_of_robot);
    // What padding do we use?
    const double padding = GetPaddingBetween(*body_A, *body_B);
    if (distance_pair.distance <= padding) {
      if (body_A_part_of_robot && body_B_part_of_robot) {
        log()->trace("Self collision between bodies [{}] and [{}]",
                     body_A->scoped_name(), body_B->scoped_name());
      } else {
        log()->trace("Environment collision between bodies [{}] and [{}]",
                     body_A->scoped_name(), body_B->scoped_name());
      }
      return false;
    }
  }
  // No relevant collisions found.
  return true;
}

RobotClearance SceneGraphCollisionChecker::DoCalcContextRobotClearance(
    const CollisionCheckerContext& model_context,
    const double influence_distance) const {
  const Frame<double>& frame_W = plant().world_frame();
  const Context<double>& plant_context = model_context.plant_context();
  const QueryObject<double>& query_object = model_context.GetQueryObject();

  // Compute the (sorted) list of SceneGraph distances.
  const double max_influence_distance =
      influence_distance + GetLargestPadding();
  const std::vector<SignedDistancePair<double>> signed_distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          max_influence_distance);

  // For each signed distance pair we're computing ϕ and
  // ∂ϕ/∂qᵣ = ∂ϕ/∂p_BAᵀ⋅∂p_BA/∂qᵣ (as documented in RobotClearance). In this
  // code, they are dist, and ddist_dq = ddist_dp_BA * dp_BA_dq.

  // Temporary storage, reused each time though the loop.
  Matrix3X<double> dp_BA_dq(3, plant().num_positions());
  Matrix3X<double> partial_temp(3, plant().num_positions());
  RowVectorXd ddist_dq(plant().num_positions());

  // Loop over the distances, converting to the robot clearance result.
  RobotClearance result(plant().num_positions());
  result.Reserve(signed_distance_pairs.size());
  for (const auto& signed_distance_pair : signed_distance_pairs) {
    const double unpadded_distance = signed_distance_pair.distance;
    DRAKE_ASSERT(unpadded_distance <= max_influence_distance);

    // Grab some nice, short names for the relevant data.
    const GeometryId geometry_id_A = signed_distance_pair.id_A;
    const GeometryId geometry_id_B = signed_distance_pair.id_B;
    const SceneGraphInspector<double>& inspector = query_object.inspector();
    const FrameId frame_id_A = inspector.GetFrameId(geometry_id_A);
    const FrameId frame_id_B = inspector.GetFrameId(geometry_id_B);
    const RigidBody<double>* body_A_maybe =
        plant().GetBodyFromFrameId(frame_id_A);
    const RigidBody<double>* body_B_maybe =
        plant().GetBodyFromFrameId(frame_id_B);
    DRAKE_THROW_UNLESS(body_A_maybe != nullptr);
    DRAKE_THROW_UNLESS(body_B_maybe != nullptr);
    const RigidBody<double>& body_A = *body_A_maybe;
    const RigidBody<double>& body_B = *body_B_maybe;
    const Frame<double>& frame_A = body_A.body_frame();
    const Frame<double>& frame_B = body_B.body_frame();

    // Enforce that our collision filters are consistent with query results.
    if (IsCollisionFilteredBetween(body_A, body_B)) {
      throw std::runtime_error(fmt::format(
          "Drake internal error at {}:{} in {}(): Collision between bodies [{}]"
          " and [{}] should already be filtered",
          __FILE__, __LINE__, __func__, body_A.scoped_name(),
          body_B.scoped_name()));
    }

    // Adjust pair-specific padding, and skip now-unnecessary distances.
    // Two geometries must be >= padding apart to be treated as non-colliding.
    const double padding = GetPaddingBetween(body_A, body_B);
    const double distance = unpadded_distance - padding;
    if (distance > influence_distance) {
      continue;
    }

    // Classify the two bodies.
    const bool body_A_part_of_robot = IsPartOfRobot(body_A);
    const bool body_B_part_of_robot = IsPartOfRobot(body_B);
    DRAKE_ASSERT(body_A_part_of_robot || body_B_part_of_robot);
    const BodyIndex robot_index =
        body_A_part_of_robot ? body_A.index() : body_B.index();
    const BodyIndex other_index =
        body_A_part_of_robot ? body_B.index() : body_A.index();
    const RobotCollisionType collision_type =
        (body_A_part_of_robot && body_B_part_of_robot)
            ? RobotCollisionType::kSelfCollision
            : RobotCollisionType::kEnvironmentCollision;

    // Convert the witness points from geometry frame to body frame. (Note
    // that the monogram frame names are somewhat unhelpful here because
    // "A" and "B" are overloaded for frame points vs body points.)
    const Vector3d p_ACa =
        inspector.GetPoseInFrame(geometry_id_A) * signed_distance_pair.p_ACa;
    const Vector3d p_BCb =
        inspector.GetPoseInFrame(geometry_id_B) * signed_distance_pair.p_BCb;

    // Convert the witness points from body frame to world frame.
    // TODO(jwnimmer-tri) Instead of CalcPointsPositions, try either
    // RigidBody::EvalPoseInWorld or QueryObject::GetPoseInWorld.
    Vector3d p_WCa;
    plant().CalcPointsPositions(plant_context, frame_A, p_ACa, frame_W, &p_WCa);
    Vector3d p_WCb;
    plant().CalcPointsPositions(plant_context, frame_B, p_BCb, frame_W, &p_WCb);

    // Compute the spatial gradient of distance (expressed in the world).
    const Vector3d ddist_dp_BA =
        (distance > 0 ? 1 : -1) * (p_WCa - p_WCb).stableNormalized();

    // We're computing ∂p_BA_W/∂qᵣ = ∂p_WA/∂qᵣ - ∂p_WB/∂qᵣ. Because we're
    // differentiating w.r.t. qᵣ, if either A or B is not a robot body, that
    // body's contribution goes to zero (and we skip computing that term as an
    // optimization).
    //
    // However, even with these zero terms, we are not necessarily returning
    // ∂ϕ/∂qᵣ. If either A or B has non-robot, movable bodies inboard of it,
    // our ∂p_BA_W/∂qᵣ may include additional non-zero columns; we rely on the
    // parent class to zero those columns out and make the result truly ∂ϕ/∂qᵣ.
    if (body_A_part_of_robot) {
      plant().CalcJacobianPositionVector(  // BR
          plant_context, frame_A, p_ACa, frame_W, frame_W,
          // ∂p_WA/∂q after this invocation
          &dp_BA_dq);
    } else {
      dp_BA_dq.setZero();
    }
    if (body_B_part_of_robot) {
      plant().CalcJacobianPositionVector(  // BR
          plant_context, frame_B, p_BCb, frame_W, frame_W,
          // ∂p_WB/∂q
          &partial_temp);
      dp_BA_dq -= partial_temp;  // ∂p_WA/∂qᵣ - ∂p_WB/∂qᵣ.
    }
    ddist_dq.noalias() = ddist_dp_BA.transpose() * dp_BA_dq;

    result.Append(robot_index, other_index, collision_type, distance, ddist_dq);
  }
  return result;
}

std::vector<RobotCollisionType>
SceneGraphCollisionChecker::DoClassifyContextBodyCollisions(
    const CollisionCheckerContext& model_context) const {
  // Collision check to get colliding geometry.
  const QueryObject<double>& query_object = model_context.GetQueryObject();
  const SceneGraphInspector<double>& inspector = query_object.inspector();

  const std::vector<SignedDistancePair<double>>& distance_pairs =
      query_object.ComputeSignedDistancePairwiseClosestPoints(
          GetLargestPadding());

  std::vector<RobotCollisionType> robot_collision_types(
      plant().num_bodies(), RobotCollisionType::kNoCollision);

  for (const auto& distance_pair : distance_pairs) {
    // Get the bodies corresponding to the distance pair.
    const FrameId frame_id_A = inspector.GetFrameId(distance_pair.id_A);
    const FrameId frame_id_B = inspector.GetFrameId(distance_pair.id_B);
    const RigidBody<double>* body_A = plant().GetBodyFromFrameId(frame_id_A);
    const RigidBody<double>* body_B = plant().GetBodyFromFrameId(frame_id_B);
    DRAKE_THROW_UNLESS(body_A != nullptr);
    DRAKE_THROW_UNLESS(body_B != nullptr);
    // Ignore distance pair involving allowed collisions.
    if (IsCollisionFilteredBetween(*body_A, *body_B)) {
      continue;
    }
    const bool body_A_part_of_robot = IsPartOfRobot(*body_A);
    const bool body_B_part_of_robot = IsPartOfRobot(*body_B);
    // What padding do we use?
    const double padding = GetPaddingBetween(*body_A, *body_B);
    if (distance_pair.distance <= padding) {
      if (body_A_part_of_robot && body_B_part_of_robot) {
        robot_collision_types.at(body_A->index()) =
            SetInSelfCollision(robot_collision_types.at(body_A->index()), true);
        robot_collision_types.at(body_B->index()) =
            SetInSelfCollision(robot_collision_types.at(body_B->index()), true);
      } else if (body_A_part_of_robot) {
        robot_collision_types.at(body_A->index()) = SetInEnvironmentCollision(
            robot_collision_types.at(body_A->index()), true);
      } else if (body_B_part_of_robot) {
        robot_collision_types.at(body_B->index()) = SetInEnvironmentCollision(
            robot_collision_types.at(body_B->index()), true);
      }
    }
  }

  return robot_collision_types;
}

int SceneGraphCollisionChecker::DoMaxContextNumDistances(
    const CollisionCheckerContext& model_context) const {
  const QueryObject<double>& query_object = model_context.GetQueryObject();
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  // Maximum number of SignedDistancePairs returned by calls to
  // ComputeSignedDistancePairwiseClosestPoints().
  const std::set<std::pair<GeometryId, GeometryId>>& collision_candidates =
      inspector.GetCollisionCandidates();
  int valid_candidate_count = 0;
  for (const auto& candidate : collision_candidates) {
    const FrameId frame_id_A = inspector.GetFrameId(candidate.first);
    const FrameId frame_id_B = inspector.GetFrameId(candidate.second);
    const RigidBody<double>* body_A = plant().GetBodyFromFrameId(frame_id_A);
    const RigidBody<double>* body_B = plant().GetBodyFromFrameId(frame_id_B);
    DRAKE_THROW_UNLESS(body_A != nullptr);
    DRAKE_THROW_UNLESS(body_B != nullptr);

    if (IsPartOfRobot(*body_A) || IsPartOfRobot(*body_B)) {
      ++valid_candidate_count;
    }
  }
  return valid_candidate_count;
}

void SceneGraphCollisionChecker::ApplyCollisionFiltersToSceneGraph() {
  // Assemble the new collision filters.
  CollisionFilterDeclaration new_collision_filters;

  const int num_bodies = plant().num_bodies();
  for (BodyIndex i(0); i < num_bodies; ++i) {
    const FrameId frame_i = plant().GetBodyFrameIdOrThrow(i);
    const GeometrySet geometries_i(frame_i);

    // Add within-body filter.
    new_collision_filters.ExcludeWithin(geometries_i);

    // Collect body-body filters.
    for (BodyIndex j(i + 1); j < num_bodies; ++j) {
      const FrameId frame_j = plant().GetBodyFrameIdOrThrow(j);
      const GeometrySet geometries_j(frame_j);

      if (IsCollisionFilteredBetween(i, j)) {
        new_collision_filters.ExcludeBetween(geometries_i, geometries_j);
      } else {
        new_collision_filters.AllowBetween(geometries_i, geometries_j);
      }
    }
  }

  // Apply the new collision filters to every context.
  const auto operation = [&new_collision_filters](
                             const RobotDiagram<double>& model,
                             CollisionCheckerContext* model_context) {
    auto& sg_context = model_context->mutable_scene_graph_context();
    model.scene_graph()
        .collision_filter_manager(&sg_context)
        .Apply(new_collision_filters);
  };

  PerformOperationAgainstAllModelContexts(operation);
}

}  // namespace planning
}  // namespace drake
