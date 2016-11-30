#pragma once

#include <string>
#include <vector>

#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"

namespace DrakeCollision {

/// An unusable model, used when no collision detection backend is available.
class UnusableModel : public Model {
 public:
  UnusableModel() {}

  virtual ~UnusableModel() {}

  void updateModel() override;

  bool updateElementWorldTransform(
      const ElementId, const Eigen::Isometry3d& T_local_to_world) override;

  bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  bool ComputeMaximumDepthCollisionPoints(
      const bool use_margins, std::vector<PointPair>& points) override;

  bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             const bool use_margins,
                             std::vector<PointPair>& closest_points) override;

  void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>& closest_points) override;

  void ClearCachedResults(bool use_margins) override;

  bool collisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd& distances,
                        Eigen::Matrix3Xd& normals) override;

  std::vector<PointPair> potentialCollisionPoints(bool use_margins) override;

  bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

  std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

 private:
  // UnusableModel objects are not copyable
  UnusableModel(const UnusableModel&) = delete;
  UnusableModel& operator=(const UnusableModel&) = delete;
};

}  // namespace DrakeCollision
