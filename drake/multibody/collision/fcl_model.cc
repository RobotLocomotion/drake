#include "drake/multibody/collision/fcl_model.h"

#include <utility>

#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace multibody {
namespace collision {

// TODO(jamiesnape): Implement the model.

void FclModel::DoAddElement(const Element& element) {
  ElementId id = element.getId();

  // Create the fcl::CollisionObject. Store it as a shared_ptr for use with the
  // fcl::CollisionObject constructor below.
  std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;

  switch (element.getShape()) {
    case DrakeShapes::SPHERE: {
      const auto sphere =
          static_cast<const DrakeShapes::Sphere&>(element.getGeometry());
      fcl_geometry = std::make_shared<fcl::Sphered>(sphere.radius);
    } break;
    default:
      DRAKE_ABORT_MSG("Not implemented.");
      break;
  }

  auto fcl_object = std::make_unique<fcl::CollisionObjectd>(fcl_geometry);

  // Store a pointer to the Element in the fcl collision object. This will be
  // used for collision filtering inside callback functions.
  // NOTE: setUserData requires a mutable pointer as input. However, the user
  // data is only used by the user-provided callback functions, and our
  // callbacks cast the user data back to a const Element*.
  fcl_object->setUserData(const_cast<Element*>(&element));

  // NOTE: This will be the *only* time that anchored collision objects
  // will have their world transform set.  This code assumes that the world
  // transform on the corresponding input Drake element has already been
  // properly set. (See RigidBodyTree::CompileCollisionState.)
  fcl_object->setTransform(element.getWorldTransform());

  // Register the object with the broadphase collision manager.
  broadphase_manager_.registerObject(fcl_object.get());
  // Take ownership of FCL collision object, as its lifetime must extend to at
  // least that of the broadphase manager.
  fcl_collision_objects_.insert(std::make_pair(id, move(fcl_object)));
}

void FclModel::UpdateModel() {
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FclModel::ClosestPointsAllToAll(const std::vector<ElementId>& ids_to_check,
                                     bool use_margins,
                                     std::vector<PointPair>* closest_points) {
  drake::unused(ids_to_check, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FclModel::ComputeMaximumDepthCollisionPoints(
    bool use_margins, std::vector<PointPair>* points) {
  drake::unused(use_margins, points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FclModel::ClosestPointsPairwise(const std::vector<ElementIdPair>& id_pairs,
                                     bool use_margins,
                                     std::vector<PointPair>* closest_points) {
  drake::unused(id_pairs, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

void FclModel::CollisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair>* closest_points) {
  drake::unused(points, use_margins, closest_points);
  DRAKE_ABORT_MSG("Not implemented.");
}

void FclModel::ClearCachedResults(bool use_margins) {
  drake::unused(use_margins);
  DRAKE_ABORT_MSG("Not implemented.");
}

bool FclModel::CollisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd* distances,
                                Eigen::Matrix3Xd* normals) {
  drake::unused(origins, ray_endpoints, use_margins, distances, normals);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

bool FclModel::CollidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return false;
}

std::vector<size_t> FclModel::CollidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  DRAKE_ABORT_MSG("Not implemented.");
  return std::vector<size_t>();
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
