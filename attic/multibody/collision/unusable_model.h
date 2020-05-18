#pragma once

#include <string>
#include <vector>

#include "drake/attic_warning.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"

namespace drake {
namespace multibody {
namespace collision {

/// An unusable model, used when no collision detection backend is available.
class UnusableModel : public Model {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnusableModel)

  UnusableModel() {}

  virtual ~UnusableModel() {}

  void UpdateModel() override;

  bool UpdateElementWorldTransform(
      ElementId, const Eigen::Isometry3d& T_local_to_world) override;

  bool ClosestPointsAllToAll(
      const std::vector<ElementId>& ids_to_check, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair<double>>* points) override;

  bool ClosestPointsPairwise(
      const std::vector<ElementIdPair>& id_pairs, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  void CollisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair<double>>* closest_points) override;

  void ClearCachedResults(bool use_margins) override;

  bool CollisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd* distances,
                        Eigen::Matrix3Xd* normals) override;

  bool CollidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;

  std::vector<size_t> CollidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
