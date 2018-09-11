#include "drake/multibody/collision/unusable_model.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace collision {

void UnusableModel::UpdateModel() {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::UpdateElementWorldTransform(
    ElementId, const Eigen::Isometry3d&) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::ClosestPointsAllToAll(const std::vector<ElementId>&, bool,
                                          std::vector<PointPair<double>>*) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::ComputeMaximumDepthCollisionPoints(
    bool, std::vector<PointPair<double>>*) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::ClosestPointsPairwise(
    const std::vector<ElementIdPair>&, bool, std::vector<PointPair<double>>*) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

void UnusableModel::CollisionDetectFromPoints(
    const Eigen::Matrix3Xd&, bool, std::vector<PointPair<double>>*) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

void UnusableModel::ClearCachedResults(bool) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return;
}

bool UnusableModel::CollisionRaycast(const Eigen::Matrix3Xd&,
                                     const Eigen::Matrix3Xd&,
                                     bool,
                                     Eigen::VectorXd*,
                                     Eigen::Matrix3Xd*) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

bool UnusableModel::CollidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>&, double) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return false;
}

std::vector<size_t> UnusableModel::CollidingPoints(
    const std::vector<Eigen::Vector3d>&, double) {
  DRAKE_ABORT_MSG(
      "Compile Drake with a collision library backend for collision support!");
  return std::vector<size_t>();
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
