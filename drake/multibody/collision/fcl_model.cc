#include "drake/multibody/collision/fcl_model.h"

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"

namespace DrakeCollision {

// TODO(jamiesnape): Implement the model.

ElementId FCLModel::addElement(const Element& element) {
  DRAKE_ABORT_MSG("Not implemented.");
  return Model::addElement(element);
}

void FCLModel::updateModel() {
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FCLModel::closestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FCLModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair>& points) {
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FCLModel::closestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     bool use_margins,
                                     std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

void FCLModel::collisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>& closest_points) {
  DRAKE_ABORT_MSG("Not implemented.");
}

void FCLModel::ClearCachedResults(bool use_margins) {
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FCLModel::collisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd& distances,
                                Eigen::Matrix3Xd& normals) {
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<PointPair> FCLModel::potentialCollisionPoints(bool use_margins) {
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<PointPair>();
}

bool FCLModel::collidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<size_t> FCLModel::collidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<size_t>();
}

}  // namespace DrakeCollision
