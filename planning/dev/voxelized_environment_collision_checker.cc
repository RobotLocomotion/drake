#include "drake/planning/dev/voxelized_environment_collision_checker.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/text_logging.h"
#include "drake/planning/dev/voxel_grid_internal.h"

namespace drake {
namespace planning {

using geometry::GeometryId;
using geometry::QueryObject;
using geometry::Shape;
using math::RigidTransform;
using multibody::Body;
using multibody::BodyIndex;
using systems::Context;

VoxelizedEnvironmentCollisionChecker::VoxelizedEnvironmentCollisionChecker(
    CollisionCheckerParams params)
    : SphereRobotModelCollisionChecker(std::move(params)) {
  AllocateContexts();
}

VoxelizedEnvironmentCollisionChecker::VoxelizedEnvironmentCollisionChecker(
    const VoxelizedEnvironmentCollisionChecker&) = default;

std::unique_ptr<CollisionChecker>
VoxelizedEnvironmentCollisionChecker::DoClone() const {
  // N.B. We cannot use make_unique due to protected-only access.
  return std::unique_ptr<VoxelizedEnvironmentCollisionChecker>(
      new VoxelizedEnvironmentCollisionChecker(*this));
}

void VoxelizedEnvironmentCollisionChecker::UpdateEnvironment(
    const std::string& environment_name, const VoxelOccupancyMap& environment,
    const std::optional<BodyIndex>& override_environment_body_index) {
  if (!environment.is_empty()) {
    // Use default options for SDF generation.
    const VoxelSignedDistanceField::GenerationParameters parameters;
    const VoxelSignedDistanceField environment_sdf =
        environment.ExportSignedDistanceField(parameters);

    UpdateEnvironment(environment_name, environment_sdf,
                      override_environment_body_index);
  } else {
    RemoveEnvironment(environment_name);
  }
}

void VoxelizedEnvironmentCollisionChecker::UpdateEnvironment(
    const std::string& environment_name,
    const VoxelTaggedObjectOccupancyMap& environment,
    const std::optional<BodyIndex>& override_environment_body_index) {
  if (!environment.is_empty()) {
    // An empty vector of object indices specifies that all objects in the
    // environment should be used.
    const std::vector<uint32_t> objects_to_use;
    // Use default options for SDF generation.
    const VoxelSignedDistanceField::GenerationParameters parameters;
    const VoxelSignedDistanceField environment_sdf =
        environment.ExportSignedDistanceField(objects_to_use, parameters);

    UpdateEnvironment(environment_name, environment_sdf,
                      override_environment_body_index);
  } else {
    RemoveEnvironment(environment_name);
  }
}

void VoxelizedEnvironmentCollisionChecker::UpdateEnvironment(
    const std::string& environment_name,
    const VoxelSignedDistanceField& environment_sdf,
    const std::optional<BodyIndex>& override_environment_body_index) {
  if (!environment_sdf.is_empty()) {
    const auto& internal_sdf =
        internal::GetInternalSignedDistanceField(environment_sdf);

    environment_sdfs_[environment_name] = environment_sdf;
    if (override_environment_body_index) {
      const auto& corresponding_body =
          get_body(override_environment_body_index.value());
      environment_sdf_bodies_[environment_name] = corresponding_body.index();
      drake::log()->info(
          "Adding/updating environment model [{}] that belongs to body {} [{}] "
          "(body specified via override body index)",
          environment_name, corresponding_body.index(),
          corresponding_body.scoped_name());
    } else {
      const auto& corresponding_body =
          plant().GetBodyByName(internal_sdf.GetFrame());
      environment_sdf_bodies_[environment_name] = corresponding_body.index();
      drake::log()->info(
          "Adding/updating environment model [{}] that belongs to body {} [{}] "
          "(body identified from environment frame name [{}])",
          environment_name, corresponding_body.index(),
          corresponding_body.scoped_name(), internal_sdf.GetFrame());
    }
  } else {
    RemoveEnvironment(environment_name);
  }
}

bool VoxelizedEnvironmentCollisionChecker::RemoveEnvironment(
    const std::string& environment_name) {
  auto found_itr = environment_sdfs_.find(environment_name);
  if (found_itr != environment_sdfs_.end()) {
    environment_sdfs_.erase(found_itr);
    drake::log()->info("Removing environment model [{}]", environment_name);
    return true;
  } else {
    drake::log()->warn("No environment model to remove for [{}]",
                       environment_name);
    return false;
  }
}

PointSignedDistanceAndGradientResult VoxelizedEnvironmentCollisionChecker ::
    ComputePointToEnvironmentSignedDistanceAndGradient(
        const Context<double>&, const QueryObject<double>&,
        const Eigen::Vector4d& p_WQ, const double query_radius,
        const std::vector<Eigen::Isometry3d>& X_WB_set,
        const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const {
  PointSignedDistanceAndGradientResult result;
  // Check each of our environment models.
  for (const auto& [environment_name, environment_sdf] : environment_sdfs_) {
    const auto& internal_sdf =
        internal::GetInternalSignedDistanceField(environment_sdf);
    const auto& sdf_body_index = environment_sdf_bodies_.at(environment_name);
    const auto& X_WB = X_WB_set.at(sdf_body_index);
    const auto& X_BW = X_WB_inverse_set.at(sdf_body_index);
    const Eigen::Vector4d p_BQ = X_BW * p_WQ;
    const auto distance_query = internal_sdf.EstimateLocationDistance4d(p_BQ);
    if (distance_query) {
      if (distance_query.Value() <= query_radius) {
        const double window = internal_sdf.GetResolution() * 0.25;
        const auto gradient_query =
            internal_sdf.GetLocationFineGradient4d(p_BQ, window);
        // Make sure our check is inside the bounds of the SDF.
        if (gradient_query) {
          // Rotate the gradient from frame B into world frame.
          const Eigen::Vector4d& gradient_B = gradient_query.Value();
          const Eigen::Vector4d gradient_W = X_WB * gradient_B;
          result.AddDistanceAndGradient(distance_query.Value(), gradient_W,
                                        sdf_body_index);
        }
      }
    }
  }
  return result;
}

PointSignedDistanceAndGradientResult
VoxelizedEnvironmentCollisionChecker ::ComputePointToEnvironmentSignedDistance(
    const Context<double>&, const QueryObject<double>&,
    const Eigen::Vector4d& p_WQ, const double query_radius,
    const std::vector<Eigen::Isometry3d>&,
    const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const {
  PointSignedDistanceAndGradientResult result;
  // Check each of our environment models.
  for (const auto& [environment_name, environment_sdf] : environment_sdfs_) {
    const auto& internal_sdf =
        internal::GetInternalSignedDistanceField(environment_sdf);
    const auto& sdf_body_index = environment_sdf_bodies_.at(environment_name);
    const auto& X_BW = X_WB_inverse_set.at(sdf_body_index);
    const Eigen::Vector4d p_BQ = X_BW * p_WQ;
    const auto distance_query = internal_sdf.EstimateLocationDistance4d(p_BQ);
    if (distance_query) {
      if (distance_query.Value() <= query_radius) {
        result.AddDistance(distance_query.Value(), sdf_body_index);
      }
    }
  }
  return result;
}

std::optional<GeometryId>
VoxelizedEnvironmentCollisionChecker::AddEnvironmentCollisionShapeToBody(
    const std::string&, const Body<double>&, const Shape&,
    const RigidTransform<double>&) {
  drake::log()->warn(
      "VoxelizedEnvironmentCollisionChecker::"
      "AddEnvironmentCollisionShapeToBody() not implemented.");
  return std::nullopt;
}

void VoxelizedEnvironmentCollisionChecker::RemoveAllAddedEnvironment(
    const std::vector<CollisionChecker::AddedShape>&) {
  drake::log()->warn(
      "VoxelizedEnvironmentCollisionChecker::RemoveAllAddedEnvironment does "
      "nothing.");
}

std::optional<double> VoxelizedEnvironmentCollisionChecker ::
    EstimateConservativePointToEnvironmentSignedDistance(
        const Context<double>&, const QueryObject<double>&,
        const Eigen::Vector4d& p_WQ, const double,
        const std::vector<Eigen::Isometry3d>&,
        const std::vector<Eigen::Isometry3d>& X_WB_inverse_set) const {
  double minimum_distance = std::numeric_limits<double>::infinity();

  // Check each of our environment models. For performance, we use raw distance
  // field value, which is the coarsest approximation of the signed distance
  // available. To conservatively underestimate the minimum distance we return,
  // we simply subtract 2x voxel resolution which is more than any
  // discretization error that may be introduced.
  for (const auto& [environment_name, environment_sdf] : environment_sdfs_) {
    const auto& internal_sdf =
        internal::GetInternalSignedDistanceField(environment_sdf);
    const auto& sdf_body_index = environment_sdf_bodies_.at(environment_name);
    const auto& X_BW = X_WB_inverse_set.at(sdf_body_index);
    const Eigen::Vector4d p_BQ = X_BW * p_WQ;
    const auto coarse_distance_query =
        internal_sdf.GetLocationImmutable4d(p_BQ);
    if (coarse_distance_query) {
      const double distance_error = internal_sdf.GetResolution() * 2.0;
      const double coarse_distance_estimate =
          coarse_distance_query.Value() - distance_error;
      if (coarse_distance_estimate < minimum_distance) {
        minimum_distance = coarse_distance_estimate;
      }
    } else {
      // We can only return a distance bound if we were able to query all of the
      // environment models. If we could not query all models, we cannot
      // guarantee that the returned minimum distance is an underestimate, so we
      // fail fast and return nullopt here.
      return std::nullopt;
    }
  }

  return minimum_distance;
}

}  // namespace planning
}  // namespace drake
