#pragma once

#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/collision/model.h"
#include "drake/multibody/collision/point_pair.h"

namespace drake {
namespace multibody {
namespace collision {

class FclModel : public Model {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FclModel)

  FclModel() {}
  virtual ~FclModel() {}

  void DoAddElement(const Element& element) override;
  bool ClosestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                             bool use_margins,
                             std::vector<PointPair>* closest_points) override;
  bool ClosestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                             bool use_margins,
                             std::vector<PointPair>* closest_points) override;
  bool CollidingPointsCheckOnly(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  bool CollisionRaycast(const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints, bool use_margins,
                        Eigen::VectorXd* distances,
                        Eigen::Matrix3Xd* normals) override;
  bool ComputeMaximumDepthCollisionPoints(
      bool use_margins, std::vector<PointPair>* points) override;
  std::vector<size_t> CollidingPoints(
      const std::vector<Eigen::Vector3d>& input_points,
      double collision_threshold) override;
  void ClearCachedResults(bool use_margins) override;
  void CollisionDetectFromPoints(
      const Eigen::Matrix3Xd& points, bool use_margins,
      std::vector<PointPair>* closest_points) override;
  void UpdateModel() override;
};

}  // namespace collision
}  // namespace multibody
}  // namespace drake
