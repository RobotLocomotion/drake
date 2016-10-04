#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/drakeCollision_export.h"

#include "drake/systems/plants/collision/Element.h"
#include "drake/systems/plants/collision/Model.h"
#include "drake/systems/plants/collision/point_pair.h"

namespace DrakeCollision {

class DRAKECOLLISION_EXPORT FCLModel : public Model {
 public:
  FCLModel() {}
  virtual ~FCLModel() {}

  bool closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             bool use_margins,
                             std::vector<PointPair>& closest_points) override;
  bool closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             bool use_margins,
                             std::vector<PointPair>& closest_points) override;
  bool collidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  bool collisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd& distances,
                        Eigen::Matrix3Xd& normals) override;
  bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair>& points) override;
  std::vector<PointPair> potentialCollisionPoints(bool use_margins) override;
  std::vector<size_t> collidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  void ClearCachedResults(bool use_margins) override;
  void collisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>& closest_points) override;
  void updateModel() override;

  FCLModel(const FCLModel&) = delete;
  FCLModel& operator=(const FCLModel&) = delete;
};

}  // namespace DrakeCollision
