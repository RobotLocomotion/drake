#include "drake/systems/plants/collision/unusable_model.h"

#include "drake/common/drake_assert.h"

namespace DrakeCollision {

void UnusableModel::updateModel() {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::updateElementWorldTransform(
    const ElementId, const Eigen::Isometry3d& T_local_to_world) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::closestPointsAllToAll(
    const std::vector<ElementId>& ids_to_check, const bool use_margins,
    std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::ComputeMaximumDepthCollisionPoints(
    const bool use_margins, std::vector<PointPair>& points) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::closestPointsPairwise(
    const std::vector<ElementIdPair>& id_pairs, const bool use_margins,
    std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

void UnusableModel::collisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

void UnusableModel::ClearCachedResults(bool use_margins) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::collisionRaycast(const Eigen::Matrix3Xd& origins,
                                     const Eigen::Matrix3Xd& ray_endpoints,
                                     bool use_margins,
                                     Eigen::VectorXd& distances,
                                     Eigen::Matrix3Xd& normals) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

std::vector<PointPair> UnusableModel::potentialCollisionPoints(
    bool use_margins) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return std::vector<PointPair>();
}

bool UnusableModel::collidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

std::vector<size_t> UnusableModel::collidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return std::vector<size_t>();
}

}  // namespace DrakeCollision
