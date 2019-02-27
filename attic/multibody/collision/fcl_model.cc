#include "drake/multibody/collision/fcl_model.h"

#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <fcl/fcl.h>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace collision {
namespace {
// Struct for use in CollisionCallback(). Contains the collision request and
// accumulates results in a drake::multibody::collision::PointPair vector.
struct CollisionData {
  // Collision request
  fcl::CollisionRequestd request;

  // Vector of distance results
  std::vector<PointPair<double>>* closest_points{};
};

// Callback function for FCL's collide() function.
bool CollisionCallback(fcl::CollisionObjectd* fcl_object_A,
                       fcl::CollisionObjectd* fcl_object_B,
                       void* callback_data) {
  // Retrieve the drake::multibody::collision::Element object associated with
  // each FCL collision object.
  auto element_A = static_cast<const Element*>(fcl_object_A->getUserData());
  auto element_B = static_cast<const Element*>(fcl_object_B->getUserData());
  // Do not proceed with collision checking if this pair of elements is excluded
  // by either of our collision filtering mechanisms (filter groups and
  // cliques).
  if (element_A && element_B && element_A->CanCollideWith(element_B)) {
    // Unpack the callback data
    auto* collision_data = static_cast<CollisionData*>(callback_data);
    const fcl::CollisionRequestd& request = collision_data->request;
    fcl::CollisionResultd result;

    // Perform nearphase collision detection
    fcl::collide(fcl_object_A, fcl_object_B, request, result);

    // Process the contact points
    std::vector<fcl::Contactd> contacts;
    result.getContacts(contacts);
    if (!contacts.empty()) {
      const fcl::Contactd& contact{contacts[0]};
      //  By convention, Drake requires the contact normal to point out of B and
      //  into A. FCL uses the opposite convention.
      Vector3d drake_normal = -contact.normal;

      // Signed distance is negative when penetration depth is positive.
      double signed_distance = -contact.penetration_depth;

      // FCL returns a single contact point, but PointPair expects two, one on
      // the surface of body A (Ac) and one on the surface of body B (Bc).
      // Choose points along the line defined by the contact point and normal,
      // equi-distant to the contact point. Recall that signed_distance is
      // strictly non-positive, so signed_distance * drake_normal points out of
      // A and into B.
      const Vector3d p_WAc{contact.pos + 0.5 * signed_distance * drake_normal};
      const Vector3d p_WBc{contact.pos - 0.5 * signed_distance * drake_normal};

      collision_data->closest_points->emplace_back(
          element_A, element_B, p_WAc, p_WBc, drake_normal, signed_distance);
    }
  }

  // Returning true would tell the broadphase manager to terminate early. Since
  // we want to find all the collisions present in the model's current
  // configuration, we return false.
  return false;
}
}  // namespace

void FclModel::DoAddElement(const Element& element) {
  ElementId id = element.getId();

  // Create the fcl::CollisionObject. Store it as a shared_ptr for use with the
  // fcl::CollisionObject constructor below.
  std::shared_ptr<fcl::CollisionGeometryd> fcl_geometry;

  switch (element.getShape()) {
    case DrakeShapes::SPHERE: {
      const auto& sphere =
          static_cast<const DrakeShapes::Sphere&>(element.getGeometry());
      fcl_geometry = std::make_shared<fcl::Sphered>(sphere.radius);
    } break;
    case DrakeShapes::CYLINDER: {
      const auto& cylinder =
          static_cast<const DrakeShapes::Cylinder&>(element.getGeometry());
      fcl_geometry =
          std::make_shared<fcl::Cylinderd>(cylinder.radius, cylinder.length);
    } break;
    default:
      throw std::logic_error("Not implemented.");
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

void FclModel::DoRemoveElement(ElementId id) {
  const auto& itr = fcl_collision_objects_.find(id);
  if (itr != fcl_collision_objects_.end()) {
    broadphase_manager_.unregisterObject(itr->second.get());
  }
}

void FclModel::UpdateModel() { broadphase_manager_.update(); }

bool FclModel::UpdateElementWorldTransform(ElementId id,
                                           const Isometry3d& X_WL) {
  const bool element_exists(Model::UpdateElementWorldTransform(id, X_WL));
  if (element_exists) {
    fcl_collision_objects_[id]->setTransform(
        FindElement(id)->getWorldTransform());
  }
  return element_exists;
}

bool FclModel::ClosestPointsAllToAll(
    const std::vector<ElementId>& ids_to_check, bool use_margins,
    std::vector<PointPair<double>>* closest_points) {
  drake::unused(ids_to_check, use_margins, closest_points);
  throw std::logic_error("Not implemented.");
}

bool FclModel::ComputeMaximumDepthCollisionPoints(
    bool, std::vector<PointPair<double>>* points) {
  CollisionData collision_data;
  collision_data.closest_points = points;
  collision_data.request.enable_contact = true;
  broadphase_manager_.collide(static_cast<void*>(&collision_data),
                              CollisionCallback);
  return true;
}

bool FclModel::ClosestPointsPairwise(
    const std::vector<ElementIdPair>& id_pairs, bool use_margins,
    std::vector<PointPair<double>>* closest_points) {
  drake::unused(id_pairs, use_margins, closest_points);
  throw std::logic_error("Not implemented.");
}

void FclModel::CollisionDetectFromPoints(
    const Eigen::Matrix3Xd& points, bool use_margins,
    std::vector<PointPair<double>>* closest_points) {
  drake::unused(points, use_margins, closest_points);
  throw std::logic_error("Not implemented.");
}

void FclModel::ClearCachedResults(bool use_margins) {
  drake::unused(use_margins);
  throw std::logic_error("Not implemented.");
}

bool FclModel::CollisionRaycast(const Eigen::Matrix3Xd& origins,
                                const Eigen::Matrix3Xd& ray_endpoints,
                                bool use_margins, Eigen::VectorXd* distances,
                                Eigen::Matrix3Xd* normals) {
  drake::unused(origins, ray_endpoints, use_margins, distances, normals);
  throw std::logic_error("Not implemented.");
}

bool FclModel::CollidingPointsCheckOnly(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  throw std::logic_error("Not implemented.");
}

std::vector<size_t> FclModel::CollidingPoints(
    const std::vector<Eigen::Vector3d>& input_points,
    double collision_threshold) {
  drake::unused(input_points, collision_threshold);
  throw std::logic_error("Not implemented.");
}

}  // namespace collision
}  // namespace multibody
}  // namespace drake
