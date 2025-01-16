#include "drake/planning/dev/sphere_robot_model_collision_checker.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <common_robotics_utilities/math.hpp>
#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/print.hpp>

#include "drake/common/text_logging.h"
#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace planning {

using geometry::Box;
using geometry::Capsule;
using geometry::CollisionFilterDeclaration;
using geometry::Convex;
using geometry::Cylinder;
using geometry::Ellipsoid;
using geometry::GeometryId;
using geometry::GeometrySet;
using geometry::HalfSpace;
using geometry::Mesh;
using geometry::QueryObject;
using geometry::Shape;
using geometry::ShapeReifier;
using geometry::Sphere;
using math::RigidTransform;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Frame;
using multibody::JacobianWrtVariable;
using systems::Context;

namespace {
std::vector<Eigen::Isometry3d> InvertPoses(
    const std::vector<Eigen::Isometry3d>& poses) {
  std::vector<Eigen::Isometry3d> inverted(poses.size());
  for (size_t idx = 0; idx < poses.size(); idx++) {
    inverted.at(idx) = poses.at(idx).inverse();
  }
  return inverted;
}

class SphereModelShapeReifier : public geometry::ShapeReifier {
 public:
  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = sphere.radius();
  }

  void ImplementGeometry(const Cylinder&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const HalfSpace&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const Box&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const Mesh&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const Convex&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const Capsule&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(const Ellipsoid&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }
};

}  // namespace

SphereSpecification BodySpheres::ComputeApproximateBoundingSphere(
    const std::unordered_map<GeometryId, SphereSpecification>& spheres) {
  // This computes an approximate bounding sphere of the provided spheres via a
  // modified version of Ritter's algorithm for approximate bounding sphere of
  // points:
  //
  // 1. Approximate the provided spheres with a set of points on the surface of
  // the sphere. Each sphere is captured by 14 geodesically-well-distributed
  // points: the 6 axial points, plus one in each octant at the surface on a
  // (1,1,1) ray (i.e. the corners of the inscribed cube of +/- radius/sqrt(3)).
  //
  // 2. Initialize the approximate bounding sphere at the midpoint of a pair of
  // distant points.
  //
  // 3. Iteratively refine the approximate bounding sphere. For each iteration,
  // identify the farthest point on any sphere from the current bounding sphere
  // origin. If this point is not contained, update the origin and radius of the
  // bounding sphere so that both previous bounding sphere and farthest point
  // are contained. Continue iterating until the farthest point is contained by
  // the bounding sphere.
  //
  // This produces an approximate bounding sphere which may be significantly
  // larger than the true minimum bounding sphere, but since this bounding
  // sphere is only used for broadphase culling of potential collision
  // candidates this is not a problem.
  // Note: in Ritter's algorithm, the points generated in step 1 would be used
  // for the refinement in step 3. Since we can easily compute the true farthest
  // point on any sphere, we no longer need the intial set of points.
  // See Graphics Gems, pages 301-303, for the original algorithm.

  // 1. Generate an initial set of points on spheres (e.g. the 6 +/-x, y, z
  // axial points, plus 8 points on an inscribed cube).
  std::vector<Eigen::Vector4d> sphere_points;
  sphere_points.reserve(spheres.size() * (6 + 8));

  for (const auto& [sphere_id, sphere] : spheres) {
    unused(sphere_id);
    const Eigen::Vector4d& p_So = sphere.Origin();
    const double radius = sphere.Radius();

    // Add the 6 axial points.
    sphere_points.push_back(p_So + Eigen::Vector4d(-radius, 0.0, 0.0, 0.0));
    sphere_points.push_back(p_So + Eigen::Vector4d(radius, 0.0, 0.0, 0.0));
    sphere_points.push_back(p_So + Eigen::Vector4d(0.0, -radius, 0.0, 0.0));
    sphere_points.push_back(p_So + Eigen::Vector4d(0.0, radius, 0.0, 0.0));
    sphere_points.push_back(p_So + Eigen::Vector4d(0.0, 0.0, -radius, 0.0));
    sphere_points.push_back(p_So + Eigen::Vector4d(0.0, 0.0, radius, 0.0));

    // Add the 8 points on an inscribed cube of (+/- radius/sqrt(3)).
    const double offset = radius / std::sqrt(3.0);
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(-offset, -offset, -offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(-offset, -offset, offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(-offset, offset, -offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(-offset, offset, offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(offset, -offset, -offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(offset, -offset, offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(offset, offset, -offset, 0.0));
    sphere_points.push_back(p_So +
                            Eigen::Vector4d(offset, offset, offset, 0.0));
  }

  // 2. Initialize the approximate bounding sphere at the midpoint of a pair of
  // distant points.

  // Helper to get the farthest point.
  const auto get_farthest_point =
      [](const std::vector<Eigen::Vector4d>& points,
         const Eigen::Vector4d& start) -> const Eigen::Vector4d& {
    double farthest_squared_distance = 0.0;
    int farthest_index = -1;

    for (size_t index = 0; index < points.size(); ++index) {
      const double squared_distance = (points[index] - start).squaredNorm();
      if (squared_distance > farthest_squared_distance) {
        farthest_index = static_cast<int>(index);
      }
    }

    DRAKE_THROW_UNLESS(farthest_index >= 0);
    return points.at(farthest_index);
  };

  const Eigen::Vector4d& first = sphere_points.front();
  const Eigen::Vector4d& point_a = get_farthest_point(sphere_points, first);
  const Eigen::Vector4d& point_b = get_farthest_point(sphere_points, point_a);

  Eigen::Vector4d bounding_origin = (point_a + point_b) * 0.5;
  double bounding_radius = (point_b - bounding_origin).norm();

  // 3. Iteratively refine the approximate bounding sphere.
  while (true) {
    // Get the farthest point away on any sphere.
    double farthest_distance = 0.0;
    Eigen::Vector4d farthest_point = bounding_origin;

    for (const auto& [sphere_id, sphere] : spheres) {
      unused(sphere_id);
      const Eigen::Vector4d& p_So = sphere.Origin();
      const double radius = sphere.Radius();

      const Eigen::Vector4d v_OriginP = (p_So - bounding_origin);
      const double origin_distance = v_OriginP.norm();
      const double sphere_farthest_distance = origin_distance + radius;
      if (sphere_farthest_distance > farthest_distance) {
        farthest_distance = sphere_farthest_distance;
        const double distance_ratio =
            sphere_farthest_distance / origin_distance;
        farthest_point = bounding_origin + (v_OriginP * distance_ratio);
      }
    }

    DRAKE_THROW_UNLESS(farthest_distance > 0.0);

    if (bounding_radius >= farthest_distance) {
      // Return the current bounding sphere.
      return SphereSpecification(bounding_origin, bounding_radius);
    } else {
      // Refine the bounding sphere to contain the farthest point. We shift
      // the origin by half the distance needed, and expand the bounding
      // sphere by the same amount. This produces a new bounding sphere which
      // contains both the previous bounding sphere and the farthest point.
      const double origin_delta = (farthest_distance - bounding_radius) * 0.5;
      // To avoid numerical issues, grow the radius a tiny bit more than
      // necessary.
      const double radius_delta =
          (farthest_distance - bounding_radius) * (0.5 + 1e-6);
      const Eigen::Vector4d v_OriginP = (farthest_point - bounding_origin);
      const Eigen::Vector4d v_offset =
          v_OriginP.stableNormalized() * origin_delta;

      bounding_origin += v_offset;
      bounding_radius += radius_delta;
    }
  }
}

SphereRobotModelCollisionChecker::SphereRobotModelCollisionChecker(
    CollisionCheckerParams params)
    : CollisionChecker(std::move(params), true /* supports parallel */) {
  InitializeRobotModel();
}

SphereRobotModelCollisionChecker::SphereRobotModelCollisionChecker(
    const SphereRobotModelCollisionChecker&) = default;

void SphereRobotModelCollisionChecker::UpdateBodyCollisionModel(
    const BodyIndex body_index, const std::vector<SphereSpecification>& spheres,
    const bool append) {
  DRAKE_THROW_UNLESS(body_index > 0);
  DRAKE_THROW_UNLESS(body_index <
                     static_cast<int32_t>(robot_sphere_model_.size()));

  BodySpheres& link_collision_model = robot_sphere_model_.at(body_index);

  // If not appending, remove all existing geometry for the link.
  if (!append) {
    link_collision_model.Clear();
  }

  for (const SphereSpecification& sphere : spheres) {
    link_collision_model.Emplace(GeometryId::get_new_id(), sphere);
  }

  // Update the bounding sphere.
  link_collision_model.MaybeUpdateBoundingSphere();
}

bool SphereRobotModelCollisionChecker::DoCheckContextConfigCollisionFree(
    const CollisionCheckerContext& model_context) const {
  const Context<double>& plant_context = model_context.plant_context();
  const QueryObject<double>& query_object = model_context.GetQueryObject();
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const std::vector<BodySpheres> spheres_in_world_frame =
      ComputeSphereLocationsInWorldFrame(X_WB_set);

  if (CheckEnvironmentCollisionFree(plant_context, query_object, X_WB_set,
                                    X_WB_inverse_set, spheres_in_world_frame)) {
    if (CheckSelfCollisionFree(spheres_in_world_frame)) {
      drake::log()->trace(
          "[Thread {}] Configuration collision-free",
          common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum());
      return true;
    } else {
      drake::log()->trace(
          "[Thread {}] Configuration in self collision",
          common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum());
    }
  } else {
    drake::log()->trace(
        "[Thread {}] Configuration in environment collision",
        common_robotics_utilities::openmp_helpers::GetContextOmpThreadNum());
  }
  return false;
}

bool SphereRobotModelCollisionChecker::CheckEnvironmentCollisionFree(
    const Context<double>& context, const QueryObject<double>& query_object,
    const std::vector<Eigen::Isometry3d>& X_WB_set,
    const std::vector<Eigen::Isometry3d>& X_WB_inverse_set,
    const std::vector<BodySpheres>& spheres_in_world_frame) const {
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1); body_index < spheres_in_world_frame.size();
       body_index++) {
    const BodySpheres& spheres = spheres_in_world_frame.at(body_index);

    if (spheres.Empty()) {
      continue;
    }

    // Initial broadphase check using bounding sphere.
    const double bounding_radius = spheres.bounding_sphere().Radius();
    const double bounding_check_distance =
        bounding_radius + GetLargestPadding();
    // Check bounding point distance.
    const std::optional<double> maybe_minimum_distance =
        EstimateConservativePointToEnvironmentSignedDistance(
            context, query_object, spheres.bounding_sphere().Origin(),
            bounding_check_distance, X_WB_set, X_WB_inverse_set);

    if (maybe_minimum_distance.has_value() &&
        maybe_minimum_distance.value() > bounding_check_distance) {
      // If the bounding query is far enough away, skip checking the rest of the
      // spheres on the body.
      continue;
    }

    // Complete check with all body spheres.
    for (const auto& [sphere_id, sphere] : spheres) {
      unused(sphere_id);
      const double radius = sphere.Radius();
      const double check_distance = radius + GetLargestPadding();
      // Check point distance.
      const auto environment_distance_check =
          ComputePointToEnvironmentSignedDistance(
              context, query_object, sphere.Origin(), check_distance, X_WB_set,
              X_WB_inverse_set);
      for (size_t idx = 0; idx < environment_distance_check.NumberOfGradients();
           idx++) {
        const auto& distance_and_gradient =
            environment_distance_check.GetDistanceAndGradient(idx);
        const auto& colliding_body_index =
            distance_and_gradient.CollidingBodyIndex();

        // Ignore distances from bodies with allowed collisions.
        if (IsCollisionFilteredBetween(body_index, colliding_body_index)) {
          continue;
        }

        const double collision_padding =
            GetPaddingBetween(body_index, colliding_body_index);
        if (distance_and_gradient.Distance() < (radius + collision_padding)) {
          drake::log()->trace(
              "Body {}, [{}] in collision with environment body {}, [{}] - "
              "distance {} for radius {} and padding {}",
              body_index, get_body(BodyIndex(body_index)).scoped_name(),
              colliding_body_index,
              get_body(colliding_body_index).scoped_name(),
              distance_and_gradient.Distance(), radius, collision_padding);
          return false;
        }
      }
    }
  }
  return true;
}

std::vector<BodySpheres>
SphereRobotModelCollisionChecker::ComputeSphereLocationsInWorldFrame(
    const std::vector<Eigen::Isometry3d>& X_WB_set) const {
  DRAKE_THROW_UNLESS(X_WB_set.size() == robot_sphere_model_.size());
  // Go through and store the world positions of each sphere center
  std::vector<BodySpheres> spheres_in_world_frame = robot_sphere_model_;
  for (size_t body_index = 0; body_index < spheres_in_world_frame.size();
       body_index++) {
    const Eigen::Isometry3d& X_WB = X_WB_set.at(body_index);
    spheres_in_world_frame.at(body_index).TransformInPlace(X_WB);
  }
  return spheres_in_world_frame;
}

bool SphereRobotModelCollisionChecker::CheckSelfCollisionFree(
    const std::vector<Eigen::Isometry3d>& X_WB_set) const {
  // Go through and store the world positions of each sphere center
  const std::vector<BodySpheres> spheres_in_world_frame =
      ComputeSphereLocationsInWorldFrame(X_WB_set);
  return CheckSelfCollisionFree(spheres_in_world_frame);
}

bool SphereRobotModelCollisionChecker::CheckSelfCollisionFree(
    const std::vector<BodySpheres>& spheres_in_world_frame) const {
  // Naive solution - loop over the sphere centers, and for each one, check if
  // any sphere from a colliding link is within radii
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1); body_index < (robot_sphere_model_.size() - 1);
       body_index++) {
    const BodySpheres& body_world_spheres =
        spheres_in_world_frame.at(body_index);

    if (body_world_spheres.Empty()) {
      continue;
    }

    // We only need to check against links farther along the kinematic chain
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < robot_sphere_model_.size(); other_body_index++) {
      // Only perform checks if self collision between links is not allowed
      if (IsCollisionFilteredBetween(body_index, other_body_index)) {
        continue;
      }

      const BodySpheres& other_body_world_spheres =
          spheres_in_world_frame.at(other_body_index);

      if (other_body_world_spheres.Empty()) {
        continue;
      }

      const double check_self_collision_padding =
          GetPaddingBetween(body_index, other_body_index);

      // Initial broadphase check using bounding sphere.
      const SphereSpecification& body_bounding_sphere =
          body_world_spheres.bounding_sphere();
      const SphereSpecification& other_body_bounding_sphere =
          other_body_world_spheres.bounding_sphere();

      const double bounding_squared_distance =
          (body_bounding_sphere.Origin() - other_body_bounding_sphere.Origin())
              .squaredNorm();
      const double bounding_check_squared_distance = std::pow(
          body_bounding_sphere.Radius() + other_body_bounding_sphere.Radius() +
              check_self_collision_padding,
          2.0);
      if (bounding_squared_distance > bounding_check_squared_distance) {
        // If the bounding query is far enough away, skip checking the rest of
        // the spheres on the body.
        continue;
      }

      // Complete check with all body spheres.
      for (const auto& [sphere_id, sphere] : body_world_spheres) {
        unused(sphere_id);
        for (const auto& [other_sphere_id, other_sphere] :
             other_body_world_spheres) {
          unused(other_sphere_id);
          const double sphere_squared_distance =
              (sphere.Origin() - other_sphere.Origin()).squaredNorm();
          const double squared_combined_radius =
              std::pow(sphere.Radius() + other_sphere.Radius() +
                           check_self_collision_padding,
                       2.0);
          // If the distance between the sphere centers is less than the sum
          // of their radii, the spheres are in collision.
          if (sphere_squared_distance < squared_combined_radius) {
            drake::log()->trace(
                "Body {}, [{}] self-collision with body {}, [{}]", body_index,
                get_body(body_index).scoped_name(), other_body_index,
                get_body(other_body_index).scoped_name());
            return false;
          }
        }
      }
    }
  }
  return true;
}

std::optional<GeometryId>
SphereRobotModelCollisionChecker::DoAddCollisionShapeToBody(
    const std::string& group_name, const Body<double>& bodyA,
    const Shape& shape, const RigidTransform<double>& X_AG) {
  if (IsPartOfRobot(bodyA)) {
    // Adding to robot.
    SphereModelShapeReifier reifier;
    double radius = -1.0;
    shape.Reify(&reifier, &radius);
    if (radius > 0.0) {
      const BodyIndex body_index = bodyA.index();
      const GeometryId new_sphere_id = GeometryId::get_new_id();
      robot_sphere_model_.at(body_index)
          .Emplace(new_sphere_id,
                   SphereSpecification(X_AG.translation(), radius));
      robot_sphere_model_.at(body_index).MaybeUpdateBoundingSphere();
      return new_sphere_id;
    } else {
      drake::log()->warn(
          "Adding non-sphere geometry to robot body: {} is not supported",
          bodyA.scoped_name());
      return std::nullopt;
    }
  } else {
    // Adding to environment.
    return AddEnvironmentCollisionShapeToBody(group_name, bodyA, shape, X_AG);
  }
}

void SphereRobotModelCollisionChecker::RemoveAddedGeometries(
    const std::vector<CollisionChecker::AddedShape>& shapes) {
  // Process the robot shapes first.
  for (const auto& shape : shapes) {
    if (IsPartOfRobot(shape.body_index)) {
      robot_sphere_model_.at(shape.body_index).Erase(shape.geometry_id);
    }
  }

  // Update the bounding spheres if necessary.
  for (auto& body_spheres : robot_sphere_model_) {
    body_spheres.MaybeUpdateBoundingSphere();
  }

  // Process the environment shapes second.
  RemoveAllAddedEnvironment(shapes);
}

PointSignedDistanceAndGradientResult
SphereRobotModelCollisionChecker::ComputeSelfCollisionSignedDistanceAndGradient(
    const std::vector<BodySpheres>& sphere_locations_in_world_frame,
    const BodyIndex query_body_index, const GeometryId query_sphere_id,
    const double influence_distance) const {
  std::vector<BodyIndex> potential_self_colliding_bodies;
  // Note that world and the current body cannot be potentially colliding, so we
  // initialize with total # of bodies - 2.
  potential_self_colliding_bodies.reserve(
      sphere_locations_in_world_frame.size() - 2);

  // Go through the links in the model.
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1);
       body_index < sphere_locations_in_world_frame.size(); body_index++) {
    // A link cannot collide with itself.
    if (body_index != query_body_index) {
      // Only perform checks if self collision between links is not allowed.
      if (!IsCollisionFilteredBetween(query_body_index, body_index)) {
        const BodySpheres& body_world_spheres =
            sphere_locations_in_world_frame.at(body_index);
        // Only perform checks if the body has collision geometry.
        if (!body_world_spheres.Empty()) {
          potential_self_colliding_bodies.push_back(body_index);
        }
      }
    }
  }

  return ComputeSelfCollisionSignedDistanceAndGradientInternal(
      sphere_locations_in_world_frame, query_body_index, query_sphere_id,
      potential_self_colliding_bodies, influence_distance);
}

PointSignedDistanceAndGradientResult SphereRobotModelCollisionChecker ::
    ComputeSelfCollisionSignedDistanceAndGradientInternal(
        const std::vector<BodySpheres>& sphere_locations_in_world_frame,
        const BodyIndex query_body_index, const GeometryId query_sphere_id,
        const std::vector<BodyIndex>& potential_self_colliding_bodies,
        const double influence_distance) const {
  PointSignedDistanceAndGradientResult result;
  const SphereSpecification& query_sphere =
      sphere_locations_in_world_frame.at(query_body_index).At(query_sphere_id);
  // Go through the potential colliding bodies.
  for (const BodyIndex& body_index : potential_self_colliding_bodies) {
    // A link cannot collide with itself.
    DRAKE_THROW_UNLESS(body_index != query_body_index);

    // Collision is not filtered.
    DRAKE_THROW_UNLESS(
        !IsCollisionFilteredBetween(query_body_index, body_index));

    const BodySpheres& body_world_spheres =
        sphere_locations_in_world_frame.at(body_index);

    const double query_body_padding =
        GetPaddingBetween(query_body_index, body_index);

    for (const auto& [sphere_id, sphere] : body_world_spheres) {
      unused(sphere_id);
      const Eigen::Vector4d other_sphere_to_query_sphere =
          query_sphere.Origin() - sphere.Origin();
      const double sphere_centers_distance =
          other_sphere_to_query_sphere.norm();
      const double radii_and_padding =
          query_sphere.Radius() + sphere.Radius() + query_body_padding;
      const double collision_distance =
          sphere_centers_distance - radii_and_padding;
      if (collision_distance <= influence_distance) {
        const Eigen::Vector4d gradient =
            other_sphere_to_query_sphere.stableNormalized();
        result.AddDistanceAndGradient(collision_distance, gradient,
                                      BodyIndex(body_index));
      }
    }
  }
  return result;
}

RobotClearance SphereRobotModelCollisionChecker::DoCalcContextRobotClearance(
    const CollisionCheckerContext& model_context,
    const double influence_distance) const {
  const Context<double>& plant_context = model_context.plant_context();
  const QueryObject<double>& query_object = model_context.GetQueryObject();
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const Frame<double>& frame_W = plant().world_frame();

  // Compute sphere locations in world.
  const std::vector<BodySpheres> world_sphere_model =
      ComputeSphereLocationsInWorldFrame(X_WB_set);

  // Walk through the robot model.
  // Skip body 0, which is world.
  RobotClearance result(plant().num_positions());
  for (BodyIndex body_index(1); body_index < world_sphere_model.size();
       ++body_index) {
    const Frame<double>& current_frame = get_body(body_index).body_frame();

    const BodySpheres& world_spheres = world_sphere_model.at(body_index);
    const BodySpheres& body_frame_spheres = robot_sphere_model_.at(body_index);

    if (world_spheres.Empty()) {
      continue;
    }

    // Initial environment broadphase check using bounding sphere.
    bool broadphase_skip_environment_checks = false;

    const double bounding_radius = world_spheres.bounding_sphere().Radius();
    const double env_bounding_check_distance =
        influence_distance + bounding_radius + GetLargestPadding();
    // Check bounding point distance.
    const std::optional<double> maybe_minimum_env_distance =
        EstimateConservativePointToEnvironmentSignedDistance(
            plant_context, query_object,
            world_spheres.bounding_sphere().Origin(),
            env_bounding_check_distance, X_WB_set, X_WB_inverse_set);

    if (maybe_minimum_env_distance.has_value() &&
        maybe_minimum_env_distance.value() > env_bounding_check_distance) {
      // If the bounding query is far enough away, skip checking the rest of the
      // spheres on the body for environment collision.
      broadphase_skip_environment_checks = true;
    }

    // Initial self broadphase check to cull potential self-collision bodies.
    std::vector<BodyIndex> potential_self_colliding_bodies;
    // Note that world and the current body cannot be potentially colliding, so
    // we initialize with total # of bodies - 2.
    potential_self_colliding_bodies.reserve(world_sphere_model.size() - 2);

    for (BodyIndex potential_body_index(1);
         potential_body_index < world_sphere_model.size();
         potential_body_index++) {
      // A link cannot collide with itself.
      if (potential_body_index != body_index) {
        // Only perform checks if self collision between links is not allowed.
        if (!IsCollisionFilteredBetween(potential_body_index, body_index)) {
          const BodySpheres& potential_body_world_spheres =
              world_sphere_model.at(potential_body_index);
          // Only perform checks if the body has self-collision geometry.
          if (!potential_body_world_spheres.Empty()) {
            const SphereSpecification& potential_body_bounding_sphere =
                potential_body_world_spheres.bounding_sphere();
            const double potential_body_padding =
                GetPaddingBetween(potential_body_index, body_index);

            const double self_bounding_squared_distance =
                (potential_body_bounding_sphere.Origin() -
                 world_spheres.bounding_sphere().Origin())
                    .squaredNorm();
            const double self_bounding_check_squared_distance = std::pow(
                potential_body_bounding_sphere.Radius() + bounding_radius +
                    potential_body_padding + influence_distance,
                2.0);

            if (self_bounding_squared_distance <=
                self_bounding_check_squared_distance) {
              potential_self_colliding_bodies.push_back(potential_body_index);
            }
          }
        }
      }
    }

    // Complete check for all spheres on body, using the broadphase results.
    for (const auto& [sphere_id, world_sphere] : world_spheres) {
      const auto& body_frame_sphere = body_frame_spheres.At(sphere_id);
      const double radius = world_sphere.Radius();
      const Eigen::Vector4d& p_WSo = world_sphere.Origin();

      Eigen::Vector3d sphere_environment_gradient = Eigen::Vector3d::Zero();
      double minimum_distance_from_environment_collision =
          std::numeric_limits<double>::infinity();
      BodyIndex minimum_environment_body_index;

      if (!broadphase_skip_environment_checks) {
        // Handle environment-collision gradients.
        const double environment_check_distance =
            influence_distance + radius + GetLargestPadding();
        const auto environment_distance_check =
            ComputePointToEnvironmentSignedDistanceAndGradient(
                plant_context, query_object, p_WSo, environment_check_distance,
                X_WB_set, X_WB_inverse_set);

        // Combine multiple gradients for environment avoidance.
        for (size_t idx = 0;
             idx < environment_distance_check.NumberOfGradients(); idx++) {
          const auto& distance_and_gradient =
              environment_distance_check.GetDistanceAndGradient(idx);
          const BodyIndex colliding_body_index(
              distance_and_gradient.CollidingBodyIndex());
          // Ignore gradients from bodies with allowed collisions.
          if (IsCollisionFilteredBetween(body_index, colliding_body_index)) {
            continue;
          }
          const double collision_padding =
              GetPaddingBetween(body_index, colliding_body_index);
          // The magnitude of the gradient of the true distance field is
          // always 1 or zero, however, in case of errors induced by
          // discretization, we normalize or leave with zero magnitude.
          const Eigen::Vector3d unit_gradient =
              distance_and_gradient.Gradient().stableNormalized().head<3>();
          const double estimated_distance_from_collision =
              distance_and_gradient.Distance() - (radius + collision_padding);
          if (estimated_distance_from_collision <
              minimum_distance_from_environment_collision) {
            minimum_distance_from_environment_collision =
                estimated_distance_from_collision;
            minimum_environment_body_index = colliding_body_index;
          }
          // Ignore points more than influence_distance from the environment.
          if (estimated_distance_from_collision < influence_distance) {
            const double delta_from_influence =
                influence_distance - estimated_distance_from_collision;
            const Eigen::Vector3d weighted_gradient =
                unit_gradient * delta_from_influence;
            sphere_environment_gradient += weighted_gradient;
          }
        }
      }

      // Handle self-collision gradients.
      const auto self_distance_check =
          ComputeSelfCollisionSignedDistanceAndGradientInternal(
              world_sphere_model, body_index, sphere_id,
              potential_self_colliding_bodies, influence_distance);

      Eigen::Vector3d sphere_self_gradient = Eigen::Vector3d::Zero();

      // Combine multiple gradients for self-collision avoidance.
      for (size_t idx = 0; idx < self_distance_check.NumberOfGradients();
           idx++) {
        const auto& distance_and_gradient =
            self_distance_check.GetDistanceAndGradient(idx);
        // The magnitude of the gradient should always be 1 or 0. Just in case,
        // we normalize or leave with zero magnitude.
        const Eigen::Vector3d unit_gradient =
            distance_and_gradient.Gradient().stableNormalized().head<3>();
        const double estimated_distance_from_collision =
            distance_and_gradient.Distance();
        // Ignore points more than influence_distance from ourself.
        if (estimated_distance_from_collision < influence_distance) {
          const double delta_from_influence =
              influence_distance - estimated_distance_from_collision;
          const Eigen::Vector3d weighted_gradient =
              unit_gradient * delta_from_influence;
          sphere_self_gradient += weighted_gradient;
        }
      }
      const double minimum_distance_from_self_collision =
          self_distance_check.MinimumDistance();
      const BodyIndex minimum_self_body_index =
          self_distance_check.MinimumCollidingBodyIndex();

      // Since we've combined a number of weighted gradients with varying
      // magnitudes, we need to normalize or leave with zero magnitude.
      // The minimum distances from collision are returned separately to allow
      // proper weighting elsewhere.
      sphere_environment_gradient.stableNormalize();
      sphere_self_gradient.stableNormalize();

      // Environment- and self-collision rows are only necessary if the minimum
      // distance from collision is less than influence distance.
      const bool has_environment_row =
          minimum_distance_from_environment_collision < influence_distance;
      const bool has_self_row =
          minimum_distance_from_self_collision < influence_distance;

      // Only compute the Jacobian if row(s) will be returned.
      Matrix3X<double> sphere_jacobian(3, GetZeroConfiguration().size());

      if (has_environment_row || has_self_row) {
        // Get the translation-only Jacobian for the current point.
        plant().CalcJacobianTranslationalVelocity(
            plant_context, JacobianWrtVariable::kQDot, current_frame,
            body_frame_sphere.Origin().head<3>(), frame_W, frame_W,
            &sphere_jacobian);
      }

      // Only add the environment Jacobian and gradient if necessary.
      if (has_environment_row) {
        result.Append(
            body_index, minimum_environment_body_index,
            RobotCollisionType::kEnvironmentCollision,
            minimum_distance_from_environment_collision,
            sphere_environment_gradient.transpose() * sphere_jacobian);
      }
      // Only add the self Jacobian and gradient if necessary.
      if (has_self_row) {
        result.Append(body_index, minimum_self_body_index,
                      RobotCollisionType::kSelfCollision,
                      minimum_distance_from_self_collision,
                      sphere_self_gradient.transpose() * sphere_jacobian);
      }
    }
  }

  // Return the combined avoidance terms.
  return result;
}

std::vector<Eigen::Isometry3d>
SphereRobotModelCollisionChecker::ComputeBodyPoses(
    const Eigen::VectorXd& q, const std::optional<int> context_number) const {
  const auto& plant_context = UpdatePositions(q, context_number);
  return GetBodyPoses(plant_context);
}

std::vector<Eigen::Isometry3d> SphereRobotModelCollisionChecker::GetBodyPoses(
    const Context<double>& plant_context) const {
  std::vector<Eigen::Isometry3d> X_WB_set(plant().num_bodies(),
                                          Eigen::Isometry3d::Identity());
  for (BodyIndex body_index(0); body_index < plant().num_bodies();
       ++body_index) {
    X_WB_set.at(static_cast<int32_t>(body_index)) =
        plant()
            .EvalBodyPoseInWorld(plant_context, get_body(body_index))
            .GetAsIsometry3();
  }
  return X_WB_set;
}

std::vector<RobotCollisionType>
SphereRobotModelCollisionChecker::DoClassifyContextBodyCollisions(
    const CollisionCheckerContext& model_context) const {
  const Context<double>& plant_context = model_context.plant_context();
  const QueryObject<double>& query_object = model_context.GetQueryObject();

  // Go through the robot collision model.
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const std::vector<BodySpheres> spheres_in_world_frame =
      ComputeSphereLocationsInWorldFrame(X_WB_set);

  std::vector<RobotCollisionType> robot_collision_types(
      plant().num_bodies(), RobotCollisionType::kNoCollision);

  for (BodyIndex body_index(1); body_index < spheres_in_world_frame.size();
       body_index++) {
    const BodySpheres& world_spheres = spheres_in_world_frame.at(body_index);

    bool in_environment_collision = false;
    bool in_self_collision = false;

    for (const auto& [sphere_id, sphere] : world_spheres) {
      // Figure out what kind of collisions involve the current sphere.

      // Check for environment collision.
      const double radius = sphere.Radius();
      const Eigen::Vector4d p_WSo = sphere.Origin();
      const double check_distance = radius + GetLargestPadding();
      // Check point distance.
      const auto environment_distance_check =
          ComputePointToEnvironmentSignedDistance(plant_context, query_object,
                                                  p_WSo, check_distance,
                                                  X_WB_set, X_WB_inverse_set);
      for (size_t idx = 0; idx < environment_distance_check.NumberOfGradients();
           idx++) {
        const auto& distance_and_gradient =
            environment_distance_check.GetDistanceAndGradient(idx);
        const BodyIndex colliding_body_index(
            distance_and_gradient.CollidingBodyIndex());
        // Ignore distances from bodies with allowed collisions.
        if (!IsCollisionFilteredBetween(body_index, colliding_body_index)) {
          const double collision_padding =
              GetPaddingBetween(body_index, colliding_body_index);
          if (distance_and_gradient.Distance() < (radius + collision_padding)) {
            in_environment_collision = true;
            drake::log()->trace(
                "Body {}, [{}] in collision with environment body {}, [{}] "
                "- distance {} for radius {} and padding {}",
                body_index, get_body(body_index).scoped_name(),
                colliding_body_index,
                get_body(colliding_body_index).scoped_name(),
                distance_and_gradient.Distance(), radius, collision_padding);
            break;
          }
        }
      }

      // Check for self collision.
      const auto self_distance_check =
          ComputeSelfCollisionSignedDistanceAndGradient(
              spheres_in_world_frame, body_index, sphere_id, 0.0);
      const double minimum_distance_from_self_collision =
          self_distance_check.MinimumDistance();
      if (minimum_distance_from_self_collision < 0.0) {
        in_self_collision = true;
        drake::log()->trace("Body {}, [{}] self-collision", body_index,
                            get_body(body_index).scoped_name());
      }
    }

    RobotCollisionType robot_collision_type = RobotCollisionType::kNoCollision;
    robot_collision_type = SetInEnvironmentCollision(robot_collision_type,
                                                     in_environment_collision);
    robot_collision_type =
        SetInSelfCollision(robot_collision_type, in_self_collision);

    robot_collision_types.at(body_index) = robot_collision_type;
    drake::log()->trace("Body {}, [{}] collision type set to {}", body_index,
                        get_body(body_index).scoped_name(),
                        static_cast<int32_t>(robot_collision_type));
  }

  return robot_collision_types;
}

void SphereRobotModelCollisionChecker::InitializeRobotModel() {
  // Build internal collision model.
  // This *MUST* occur before real contexts are allocated!
  DRAKE_THROW_UNLESS(IsInitialSetup());
  SphereModelShapeReifier reifier;
  const int num_bodies = plant().num_bodies();
  std::vector<BodySpheres> robot_sphere_model(static_cast<size_t>(num_bodies));
  // Note that the world (body 0) has no collision geometry.
  // Go through all bodies in the model, skipping the world body
  std::unordered_set<GeometryId> robot_geometries;
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const size_t raw_body_index = static_cast<size_t>(body_index);
    const Body<double>& body = get_body(body_index);
    // Does the body belong to the robot model instance(s)?
    if (IsPartOfRobot(body)) {
      const std::vector<GeometryId>& geometries =
          plant().GetCollisionGeometriesForBody(body);
      // Build the collision model
      drake::log()->debug("Body {} has {} collision geometries to check",
                          body.scoped_name(), geometries.size());
      for (const auto& geometry_id : geometries) {
        robot_geometries.insert(geometry_id);
        double radius = -1.0;
        model()
            .scene_graph()
            .model_inspector()
            .GetShape(geometry_id)
            .Reify(&reifier, &radius);
        // This is completely undocumented, but all visualizer code (RViz + LCM)
        // assumes that Frame has the same pose as the body, and thus
        // X_FG = X_BG. There is also no apparent way to retrieve the MbP frame
        // that a geometry was registered with, only the SG frame.
        const auto& X_FG =
            model().scene_graph().model_inspector().GetPoseInFrame(geometry_id);
        // Non-sphere shapes get negative radius
        if (radius > 0.0) {
          // Add sphere
          const Eigen::Vector3d p_BSo = X_FG.translation();
          robot_sphere_model.at(raw_body_index)
              .Emplace(GeometryId::get_new_id(),
                       SphereSpecification(p_BSo, radius));
        } else {
          drake::log()->warn(
              "Ignored non-sphere collision geometry {} on body {}",
              geometry_id, body.scoped_name());
        }
      }

      // Update the bounding sphere.
      robot_sphere_model.at(raw_body_index).MaybeUpdateBoundingSphere();
    }
  }
  // Enforce that the world (body 0) has no collision geometry.
  DRAKE_THROW_UNLESS(robot_sphere_model.at(0).Size() == 0);
  // Save the geometries that belong to the robot.
  robot_geometries_ = robot_geometries;
  // Disable SG collisions with the robot geometry
  GeometrySet robot_geometry_set;
  robot_geometry_set.Add(robot_geometries_);
  GetMutableSetupModel().mutable_scene_graph().collision_filter_manager().Apply(
      CollisionFilterDeclaration().ExcludeWithin(robot_geometry_set));
  // Removing geometries is a rather crude, but effective, means of filtering
  // them from any further collision checks.
  // TODO(calderpg) Remove this once better filtering for point-signed-distance
  // queries is available (see Drake #10990).
  for (const auto robot_geom_id : robot_geometries_) {
    GetMutableSetupModel().mutable_scene_graph().RemoveGeometry(
        plant().get_source_id().value(), robot_geom_id);
  }
  // Set sphere collision model.
  robot_sphere_model_ = robot_sphere_model;
}

int SphereRobotModelCollisionChecker::DoMaxContextNumDistances(
    const CollisionCheckerContext&) const {
  int total_robot_spheres = 0;
  for (const auto& body_sphere_model : robot_sphere_model_) {
    total_robot_spheres += body_sphere_model.Size();
  }
  // Each robot sphere can produce *two* rows in the output table: one distance
  // relative to the environment and one relative to other robot spheres.
  // See the *end* of the implementation of DoCalcContextRobotClearance().
  return 2 * total_robot_spheres;
}

std::string SphereRobotModelCollisionChecker ::
    GetURDFCollisionGeometriesForRobotCollisionModel() const {
  std::string urdf_collision_elements;
  const auto& robot_collision_model = RobotCollisionModel();
  for (BodyIndex body_index(1); body_index < robot_collision_model.size();
       body_index++) {
    const auto& body_spheres = robot_collision_model.at(body_index);
    // Check if the whole body is ignored.
    const BodyIndex current_body_index(body_index);
    const std::string body_name =
        plant().get_body(current_body_index).scoped_name().to_string();
    if (!urdf_collision_elements.empty()) {
      urdf_collision_elements += "\n\n";
    }
    urdf_collision_elements += body_name + ":\n";
    for (const auto& [body_sphere_id, body_sphere] : body_spheres) {
      unused(body_sphere_id);
      const Eigen::Vector4d& p_BSo = body_sphere.Origin();
      urdf_collision_elements += "\n<collision>";
      urdf_collision_elements +=
          fmt::format("\n  <origin rpy=\"0 0 0\" xyz=\"{} {} {}\"/>", p_BSo(0),
                      p_BSo(1), p_BSo(2));
      urdf_collision_elements += "\n  <geometry>";
      urdf_collision_elements +=
          fmt::format("\n    <sphere radius=\"{}\"/>", body_sphere.Radius());
      urdf_collision_elements += "\n  </geometry>";
      urdf_collision_elements += "\n</collision>";
    }
  }
  return urdf_collision_elements;
}
}  // namespace planning
}  // namespace drake
