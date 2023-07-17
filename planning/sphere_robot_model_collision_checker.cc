#include "planning/sphere_robot_model_collision_checker.h"

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

#include "drake/geometry/collision_filter_manager.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryId;
using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::planning::CollisionChecker;
using drake::planning::CollisionCheckerContext;
using drake::planning::RobotClearance;
using drake::planning::RobotCollisionType;
using std::optional;

namespace anzu {
namespace planning {
namespace {
std::vector<Eigen::Isometry3d> InvertPoses(
    const std::vector<Eigen::Isometry3d>& poses) {
  std::vector<Eigen::Isometry3d> inverted(poses.size());
  for (size_t idx = 0; idx < poses.size(); idx++) {
    inverted.at(idx) = poses.at(idx).inverse();
  }
  return inverted;
}

class SphereModelShapeReifier : public drake::geometry::ShapeReifier {
 public:
  using drake::geometry::ShapeReifier::ImplementGeometry;

  void ImplementGeometry(
      const drake::geometry::Sphere& sphere, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = sphere.radius();
  }

  void ImplementGeometry(
      const drake::geometry::Cylinder&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::HalfSpace&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::Box&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::Mesh&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::Convex&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::Capsule&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }

  void ImplementGeometry(
      const drake::geometry::Ellipsoid&, void* radius_ptr) override {
    *(static_cast<double*>(radius_ptr)) = -1.0;
  }
};

}  // namespace

SphereRobotModelCollisionChecker::SphereRobotModelCollisionChecker(
    drake::planning::CollisionCheckerParams params)
    : CollisionChecker(std::move(params), true /* supports parallel */) {
  InitializeRobotModel();
}

SphereRobotModelCollisionChecker::SphereRobotModelCollisionChecker(
    const SphereRobotModelCollisionChecker&) = default;

void SphereRobotModelCollisionChecker::UpdateBodyCollisionModel(
    const BodyIndex body_index, const std::vector<SphereSpecification>& spheres,
    const bool append) {
  DRAKE_THROW_UNLESS(body_index > 0);
  DRAKE_THROW_UNLESS(
      body_index < static_cast<int32_t>(robot_sphere_model_.size()));

  std::unordered_map<GeometryId, SphereSpecification>& link_collision_model =
      robot_sphere_model_.at(body_index);

  // If not appending, remove all existing geometry for the link.
  if (!append) {
    link_collision_model.clear();
  }

  for (const SphereSpecification& sphere : spheres) {
    link_collision_model.emplace(GeometryId::get_new_id(), sphere);
  }
}

bool SphereRobotModelCollisionChecker::DoCheckContextConfigCollisionFree(
    const CollisionCheckerContext& model_context) const {
  const drake::systems::Context<double>& plant_context =
      model_context.plant_context();
  const drake::geometry::QueryObject<double>& query_object =
      model_context.GetQueryObject();
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      spheres_in_world_frame = ComputeSphereLocationsInWorldFrame(X_WB_set);

  if (CheckEnvironmentCollisionFree(
          plant_context, query_object, X_WB_set, X_WB_inverse_set,
          spheres_in_world_frame)) {
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
    const drake::systems::Context<double>& context,
    const drake::geometry::QueryObject<double>& query_object,
    const std::vector<Eigen::Isometry3d>& X_WB_set,
    const std::vector<Eigen::Isometry3d>& X_WB_inverse_set,
    const std::vector<std::unordered_map<GeometryId, SphereSpecification>>&
        spheres_in_world_frame) const {
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1); body_index < spheres_in_world_frame.size();
       body_index++) {
    const std::unordered_map<GeometryId, SphereSpecification>& spheres =
        spheres_in_world_frame.at(body_index);
    for (const auto& [sphere_id, sphere] : spheres) {
      drake::unused(sphere_id);
      const double radius = sphere.Radius();
      const double check_distance = radius + GetLargestPadding();
      // Check point distance.
      const auto environment_distance_check =
          ComputePointToEnvironmentSignedDistance(
              context, query_object, sphere.Origin(), check_distance, X_WB_set,
              X_WB_inverse_set);
      for (size_t idx = 0;
            idx < environment_distance_check.NumberOfGradients(); idx++) {
        const auto& distance_and_gradient =
            environment_distance_check.GetDistanceAndGradient(idx);
        const auto& colliding_body_index =
            distance_and_gradient.CollidingBodyIndex();
        // Ignore distances from bodies with allowed collisions.
        if (!IsCollisionFilteredBetween(body_index, colliding_body_index)) {
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
  }
  return true;
}

std::vector<std::unordered_map<GeometryId, SphereSpecification>>
SphereRobotModelCollisionChecker::ComputeSphereLocationsInWorldFrame(
    const std::vector<Eigen::Isometry3d>& X_WB_set) const {
  DRAKE_THROW_UNLESS(X_WB_set.size() == robot_sphere_model_.size());
  // Go through and store the world positions of each sphere center
  std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      spheres_in_world_frame = robot_sphere_model_;
  for (size_t body_index = 0; body_index < spheres_in_world_frame.size();
       body_index++) {
    const Eigen::Isometry3d& X_WB = X_WB_set.at(body_index);
    std::unordered_map<GeometryId, SphereSpecification>& spheres =
        spheres_in_world_frame.at(body_index);
    for (auto& [sphere_id, sphere] : spheres) {
      drake::unused(sphere_id);
      const Eigen::Vector4d p_WSo = X_WB * sphere.Origin();
      sphere.SetOrigin(p_WSo);
    }
  }
  return spheres_in_world_frame;
}

bool SphereRobotModelCollisionChecker::CheckSelfCollisionFree(
    const std::vector<Eigen::Isometry3d>& X_WB_set) const {
  // Go through and store the world positions of each sphere center
  const std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      spheres_in_world_frame = ComputeSphereLocationsInWorldFrame(X_WB_set);
  return CheckSelfCollisionFree(spheres_in_world_frame);
}

bool SphereRobotModelCollisionChecker::CheckSelfCollisionFree(
    const std::vector<std::unordered_map<GeometryId, SphereSpecification>>&
        spheres_in_world_frame) const {
  // Naive solution - loop over the sphere centers, and for each one, check if
  // any sphere from a colliding link is within radii
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1); body_index < (robot_sphere_model_.size() - 1);
       body_index++) {
    const std::unordered_map<GeometryId, SphereSpecification>&
        body_world_spheres = spheres_in_world_frame.at(body_index);
    // We only need to check against links farther along the kinematic chain
    for (BodyIndex other_body_index(body_index + 1);
         other_body_index < robot_sphere_model_.size(); other_body_index++) {
      // Only perform checks if self collision between links is not allowed
      if (!IsCollisionFilteredBetween(body_index, other_body_index)) {
        const std::unordered_map<GeometryId, SphereSpecification>&
            other_body_world_spheres =
                spheres_in_world_frame.at(other_body_index);
        for (const auto& [sphere_id, sphere] : body_world_spheres) {
          drake::unused(sphere_id);
          for (const auto& [other_sphere_id, other_sphere] :
                   other_body_world_spheres) {
            drake::unused(other_sphere_id);
            const double sphere_squared_distance =
                (sphere.Origin() - other_sphere.Origin()).squaredNorm();
            const double check_self_collision_padding =
                GetPaddingBetween(body_index, other_body_index);
            const double squared_combined_radius =
                std::pow(sphere.Radius() + other_sphere.Radius()
                         + check_self_collision_padding, 2.0);
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
  }
  return true;
}

optional<GeometryId>
SphereRobotModelCollisionChecker::DoAddCollisionShapeToBody(
    const std::string& group_name, const drake::multibody::Body<double>& bodyA,
    const drake::geometry::Shape& shape,
    const drake::math::RigidTransform<double>& X_AG) {
  if (IsPartOfRobot(bodyA)) {
    // Adding to robot.
    SphereModelShapeReifier reifier;
    double radius = -1.0;
    shape.Reify(&reifier, &radius);
    if (radius > 0.0) {
      const BodyIndex body_index = bodyA.index();
      const GeometryId new_sphere_id = GeometryId::get_new_id();
      robot_sphere_model_.at(body_index).emplace(
          new_sphere_id, SphereSpecification(X_AG.translation(), radius));
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
      robot_sphere_model_.at(shape.body_index).erase(shape.geometry_id);
    }
  }

  // Process the environment shapes second.
  RemoveAllAddedEnvironment(shapes);
}

PointSignedDistanceAndGradientResult
SphereRobotModelCollisionChecker::ComputeSelfCollisionSignedDistanceAndGradient(
    const std::vector<std::unordered_map<GeometryId, SphereSpecification>>&
        sphere_locations_in_world_frame,
    const BodyIndex query_body_index, const GeometryId query_sphere_id,
    const double influence_distance) const {
  PointSignedDistanceAndGradientResult result;
  const SphereSpecification& query_sphere =
      sphere_locations_in_world_frame.at(query_body_index).at(query_sphere_id);
  // Go through the links in the model.
  // Skip body 0, which is world (and can't be part of the robot).
  for (BodyIndex body_index(1);
       body_index < sphere_locations_in_world_frame.size(); body_index++) {
    // A link cannot collide with itself.
    if (body_index != query_body_index) {
      // Only perform checks if self collision between links is not allowed.
      if (!IsCollisionFilteredBetween(query_body_index, body_index)) {
        const std::unordered_map<GeometryId, SphereSpecification>&
            body_world_spheres = sphere_locations_in_world_frame.at(body_index);
        for (const auto& [sphere_id, sphere] : body_world_spheres) {
          drake::unused(sphere_id);
          const Eigen::Vector4d other_sphere_to_query_sphere =
              query_sphere.Origin() - sphere.Origin();
          const double sphere_centers_distance =
              other_sphere_to_query_sphere.norm();
          const double radii_and_padding =
              query_sphere.Radius() + sphere.Radius() +
              GetPaddingBetween(query_body_index, body_index);
          const double collision_distance =
              sphere_centers_distance - radii_and_padding;
          if (collision_distance <= influence_distance) {
            const Eigen::Vector4d gradient =
                other_sphere_to_query_sphere.stableNormalized();
            result.AddDistanceAndGradient(
                collision_distance, gradient,
                drake::multibody::BodyIndex(body_index));
          }
        }
      }
    }
  }
  return result;
}

void AppendInto(Eigen::MatrixXd* to_grow, const Eigen::MatrixXd& to_append) {
  DRAKE_THROW_UNLESS(to_grow != nullptr);
  const ssize_t original_rows = to_grow->rows();
  const ssize_t original_cols = to_grow->cols();
  if (original_rows == 0 && original_cols == 0) {
    to_grow->conservativeResize(to_append.rows(), to_append.cols());
    *to_grow << to_append;
  } else if (original_cols == to_append.cols()) {
    to_grow->conservativeResize(
        original_rows + to_append.rows(), Eigen::NoChange);
    to_grow->block(original_rows, 0, to_append.rows(), to_append.cols()) =
        to_append;
  } else {
    throw std::runtime_error("Cannot append to matrix");
  }
}

void AppendInto(Eigen::VectorXd* to_grow, const Eigen::VectorXd& to_append) {
  DRAKE_THROW_UNLESS(to_grow != nullptr);
  const ssize_t original_size = to_grow->size();
  to_grow->conservativeResize(original_size + to_append.size());
  to_grow->tail(to_append.size()) = to_append;
}

void AppendInto(Eigen::VectorXd* to_grow, double to_append) {
  DRAKE_THROW_UNLESS(to_grow != nullptr);
  const ssize_t original_size = to_grow->size();
  to_grow->conservativeResize(original_size + 1);
  // I don't know why the compiler needs this, but it does.
  static_cast<Eigen::VectorXd&>(*to_grow)(original_size) = to_append;
}

RobotClearance SphereRobotModelCollisionChecker::DoCalcContextRobotClearance(
    const CollisionCheckerContext& model_context,
    const double influence_distance) const {
  const drake::systems::Context<double>& plant_context =
      model_context.plant_context();
  const drake::geometry::QueryObject<double>& query_object =
      model_context.GetQueryObject();
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const Frame<double>& frame_W = plant().world_frame();

  // Compute sphere locations in world.
  const std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      world_sphere_model = ComputeSphereLocationsInWorldFrame(X_WB_set);

  // Walk through the robot model.
  // Skip body 0, which is world.
  RobotClearance result(plant().num_positions());
  for (BodyIndex body_index(1); body_index < world_sphere_model.size();
       ++body_index) {
    const Frame<double>& current_frame = get_body(body_index).body_frame();

    const std::unordered_map<GeometryId, SphereSpecification>&
        world_spheres = world_sphere_model.at(body_index);
    const std::unordered_map<GeometryId, SphereSpecification>&
        body_frame_spheres = robot_sphere_model_.at(body_index);

    for (const auto& [sphere_id, world_sphere] : world_spheres) {
      const auto& body_frame_sphere = body_frame_spheres.at(sphere_id);
      const double radius = world_sphere.Radius();
      const Eigen::Vector4d& p_WSo = world_sphere.Origin();

      // Handle environment-collision gradients.
      const double environment_check_distance =
          influence_distance + radius + GetLargestPadding();
      const auto environment_distance_check =
          ComputePointToEnvironmentSignedDistanceAndGradient(
              plant_context, query_object, p_WSo, environment_check_distance,
              X_WB_set, X_WB_inverse_set);

      Eigen::Vector3d sphere_environment_gradient = Eigen::Vector3d::Zero();
      double minimum_distance_from_environment_collision =
          std::numeric_limits<double>::infinity();
      BodyIndex minimum_environment_body_index;

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
            distance_and_gradient.Distance()
            - (radius + collision_padding);
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

      // Handle self-collision gradients.
      const auto self_distance_check =
          ComputeSelfCollisionSignedDistanceAndGradient(
              world_sphere_model, body_index, sphere_id, influence_distance);

      Eigen::Vector3d sphere_self_gradient = Eigen::Vector3d::Zero();

      // Combine multiple gradients for self-collision avoidance.
      for (size_t idx = 0;
           idx < self_distance_check.NumberOfGradients(); idx++) {
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
      drake::Matrix3X<double> sphere_jacobian(3, GetZeroConfiguration().size());

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
        result.Append(
            body_index, minimum_self_body_index,
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
    const Eigen::VectorXd& q) const {
  const auto& plant_context = UpdatePositions(q);
  return GetBodyPoses(plant_context);
}

std::vector<Eigen::Isometry3d>
SphereRobotModelCollisionChecker::GetBodyPoses(
    const drake::systems::Context<double>& plant_context) const {
  std::vector<Eigen::Isometry3d> X_WB_set(plant().num_bodies(),
                                          Eigen::Isometry3d::Identity());
  for (drake::multibody::BodyIndex body_index(0);
       body_index < plant().num_bodies(); body_index++) {
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
  const drake::systems::Context<double>& plant_context =
      model_context.plant_context();
  const drake::geometry::QueryObject<double>& query_object =
      model_context.GetQueryObject();

  // Go through the robot collision model.
  const std::vector<Eigen::Isometry3d> X_WB_set = GetBodyPoses(plant_context);
  const std::vector<Eigen::Isometry3d> X_WB_inverse_set = InvertPoses(X_WB_set);
  const std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      spheres_in_world_frame = ComputeSphereLocationsInWorldFrame(X_WB_set);

  std::vector<RobotCollisionType> robot_collision_types(
      plant().num_bodies(), RobotCollisionType::kNoCollision);

  for (BodyIndex body_index(1); body_index < spheres_in_world_frame.size();
       body_index++) {
    const std::unordered_map<GeometryId, SphereSpecification>& world_spheres =
        spheres_in_world_frame.at(body_index);

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
          ComputePointToEnvironmentSignedDistance(
              plant_context, query_object, p_WSo, check_distance, X_WB_set,
              X_WB_inverse_set);
      for (size_t idx = 0;
          idx < environment_distance_check.NumberOfGradients(); idx++) {
        const auto& distance_and_gradient =
            environment_distance_check.GetDistanceAndGradient(idx);
        const BodyIndex colliding_body_index(
            distance_and_gradient.CollidingBodyIndex());
        // Ignore distances from bodies with allowed collisions.
        if (!IsCollisionFilteredBetween(body_index, colliding_body_index)) {
          const double collision_padding = GetPaddingBetween(
              body_index, colliding_body_index);
          if (distance_and_gradient.Distance() <
              (radius + collision_padding)) {
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
  std::vector<std::unordered_map<GeometryId, SphereSpecification>>
      robot_sphere_model(static_cast<size_t>(num_bodies));
  // Note that the world (body 0) has no collision geometry.
  // Go through all bodies in the model, skipping the world body
  std::unordered_set<drake::geometry::GeometryId> robot_geometries;
  for (BodyIndex body_index(1); body_index < num_bodies; ++body_index) {
    const size_t raw_body_index = static_cast<size_t>(body_index);
    const drake::multibody::Body<double>& body = get_body(body_index);
    // Does the body belong to the robot model instance(s)?
    if (IsPartOfRobot(body)) {
      const std::vector<drake::geometry::GeometryId>& geometries =
          plant().GetCollisionGeometriesForBody(body);
      // Build the collision model
      drake::log()->debug(
          "Body {} has {} collision geometries to check",
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
            model().scene_graph().model_inspector().GetPoseInFrame(
                geometry_id);
        // Non-sphere shapes get negative radius
        if (radius > 0.0) {
          // Add sphere
          const Eigen::Vector3d p_BSo = X_FG.translation();
          robot_sphere_model.at(raw_body_index).emplace(
              GeometryId::get_new_id(), SphereSpecification(p_BSo, radius));
        } else {
          drake::log()->warn(
              "Ignored non-sphere collision geometry {} on body {}",
              geometry_id, body.scoped_name());
        }
      }
    }
  }
  // Enforce that the world (body 0) has no collision geometry.
  DRAKE_THROW_UNLESS(robot_sphere_model.at(0).size() == 0);
  // Save the geometries that belong to the robot.
  robot_geometries_ = robot_geometries;
  // Disable SG collisions with the robot geometry
  drake::geometry::GeometrySet robot_geometry_set;
  robot_geometry_set.Add(robot_geometries_);
  GetMutableSetupModel().mutable_scene_graph().collision_filter_manager().Apply(
      drake::geometry::CollisionFilterDeclaration().ExcludeWithin(
          robot_geometry_set));
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
    total_robot_spheres += static_cast<int>(body_sphere_model.size());
  }
  // Each robot sphere can produce *two* rows in the output table: one distance
  // relative to the environment and one relative to other robot spheres.
  // See the *end* of the implementation of DoCalcContextRobotClearance().
  return 2 * total_robot_spheres;
}

std::string SphereRobotModelCollisionChecker
::GetURDFCollisionGeometriesForRobotCollisionModel() const {
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
      drake::unused(body_sphere_id);
      const Eigen::Vector4d& p_BSo = body_sphere.Origin();
      urdf_collision_elements += "\n<collision>";
      urdf_collision_elements += "\n  <origin rpy=\"0 0 0\" xyz=\""
          + std::to_string(p_BSo(0)) + " " + std::to_string(p_BSo(1)) + " "
          + std::to_string(p_BSo(2)) + "\"/>";
      urdf_collision_elements += "\n  <geometry>";
      urdf_collision_elements += "\n    <sphere radius=\""
          + std::to_string(body_sphere.Radius()) + "\"/>";
      urdf_collision_elements += "\n  </geometry>";
      urdf_collision_elements += "\n</collision>";
    }
  }
  return urdf_collision_elements;
}
}  // namespace planning
}  // namespace anzu
