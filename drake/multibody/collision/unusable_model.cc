#include "drake/multibody/collision/unusable_model.h"

#include "drake/common/drake_assert.h"

namespace DrakeCollision {

void UnusableModel::updateModel() {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::updateElementWorldTransform(
    ElementId, const Eigen::Isometry3d&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::closestPointsAllToAll(
    const std::vector<ElementId>&, bool, std::vector<PointPair>&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::ComputeMaximumDepthCollisionPoints(
    bool, std::vector<PointPair>&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::closestPointsPairwise(
    const std::vector<ElementIdPair>&, bool, std::vector<PointPair>&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

void UnusableModel::collisionDetectFromPoints(
    const Eigen::Matrix3Xd&, bool, std::vector<PointPair>&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

void UnusableModel::ClearCachedResults(bool) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::collisionRaycast(const Eigen::Matrix3Xd&,
                                     const Eigen::Matrix3Xd&,
                                     bool,
                                     Eigen::VectorXd&,
                                     Eigen::Matrix3Xd&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

std::vector<PointPair> UnusableModel::potentialCollisionPoints(bool) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return std::vector<PointPair>();
}

bool UnusableModel::collidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>&, double) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

std::vector<size_t> UnusableModel::collidingPoints(
    const std::vector<Eigen::Vector3d>&, double) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return std::vector<size_t>();
}

}  // namespace DrakeCollision
