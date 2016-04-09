
#include "RigidBody.h"
#include <stdexcept>

using namespace std;
using namespace Eigen;

RigidBody::RigidBody()
    : parent(nullptr),
      collision_filter_group(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores(DrakeCollision::NONE_MASK) {
  robotnum = 0;
  position_num_start = 0;
  velocity_num_start = 0;
  body_index = 0;
  mass = 0.0;
  com = Vector3d::Zero();
  I << Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
}

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint) {
  this->joint = move(new_joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  } else {
    throw runtime_error("Joint is not initialized");
  }
}

bool RigidBody::hasParent() const { return parent != nullptr; }

void RigidBody::addVisualElement(const DrakeShapes::VisualElement& element) {
  visual_elements.push_back(element);
}

const DrakeShapes::VectorOfVisualElements& RigidBody::getVisualElements()
    const {
  return visual_elements;
}

void RigidBody::setCollisionFilter(const DrakeCollision::bitmask& group,
                                   const DrakeCollision::bitmask& ignores) {
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    const string& group_name, vector<DrakeCollision::ElementId>& ids) const {
  auto group_ids_iter = collision_element_groups.find(group_name);
  if (group_ids_iter != collision_element_groups.end()) {
    ids.reserve(ids.size() + distance(group_ids_iter->second.begin(),
                                      group_ids_iter->second.end()));
    ids.insert(ids.end(), group_ids_iter->second.begin(),
               group_ids_iter->second.end());
    return true;
  } else {
    return false;
  }
}

bool RigidBody::appendCollisionElementIdsFromThisBody(
    vector<DrakeCollision::ElementId>& ids) const {
  ids.reserve(ids.size() + collision_element_ids.size());
  ids.insert(ids.end(), collision_element_ids.begin(),
             collision_element_ids.end());
  return true;
}

void RigidBody::ApplyTransformToJointFrame(
    const Eigen::Isometry3d& transform_body_to_joint) {
  I = transformSpatialInertia(transform_body_to_joint, I);
  for (auto& v : visual_elements) {
    v.SetLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
}

RigidBody::CollisionElement::CollisionElement(const CollisionElement& other)
    : DrakeCollision::Element(other), body(other.getBody()) {}

RigidBody::CollisionElement::CollisionElement(
    const Isometry3d& T_element_to_link, std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(T_element_to_link), body(body) {}

RigidBody::CollisionElement::CollisionElement(
    const DrakeShapes::Geometry& geometry, const Isometry3d& T_element_to_link,
    std::shared_ptr<RigidBody> body)
    : DrakeCollision::Element(geometry, T_element_to_link), body(body) {}

RigidBody::CollisionElement* RigidBody::CollisionElement::clone() const {
  return new CollisionElement(*this);
}

const std::shared_ptr<RigidBody>& RigidBody::CollisionElement::getBody() const {
  return this->body;
}

bool RigidBody::CollisionElement::CollidesWith(
    const DrakeCollision::Element* other) const {
  auto other_rb = dynamic_cast<const RigidBody::CollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = this->body->CollidesWith(*other_rb->body);
  }
  return collides;
}

ostream& operator<<(ostream& out, const RigidBody& b) {
  std::string parent_joint_name =
      b.hasParent() ? b.getJoint().getName() : "no parent joint";

  std::stringstream collision_element_str;
  collision_element_str << "[";
  for (size_t ii = 0; ii < b.collision_element_ids.size(); ii++) {
    collision_element_str << b.collision_element_ids[ii];
    if (ii < b.collision_element_ids.size() - 1) collision_element_str << ", ";
  }
  collision_element_str << "]";

  out << "RigidBody\n"
      << "  - link name: " << b.linkname << "\n"
      << "  - parent joint: " << parent_joint_name << "\n"
      << "  - Collision elements IDs: " << collision_element_str.str();

  return out;
}
