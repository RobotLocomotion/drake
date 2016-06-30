#include "drake/systems/plants/RigidBody.h"

#include <stdexcept>

#include "drake/util/drakeGeometryUtil.h"

using Eigen::Isometry3d;
using Eigen::Matrix;
using Eigen::Vector3d;

using std::ostream;
using std::runtime_error;
using std::string;
using std::stringstream;
using std::vector;

RigidBody::RigidBody()
    : collision_filter_group(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores(DrakeCollision::NONE_MASK),
      parent(nullptr) {
  robotnum = 0;
  position_num_start = 0;
  velocity_num_start = 0;
  body_index = 0;
  mass = 0.0;
  com = Vector3d::Zero();
  I << drake::SquareTwistMatrix<double>::Zero();
}

const std::string& RigidBody::name() const { return name_; }

const std::string& RigidBody::model_name() const { return model_name_; }

int RigidBody::get_model_id() const { return robotnum; }

void RigidBody::set_model_id(int model_id) { robotnum = model_id; }

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint) {
  this->joint = move(new_joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  } else {
    throw runtime_error("ERROR: RigidBody::getJoint(): Rigid body \"" + name_ +
                        "\" in model " + model_name_ +
                        " does not have a joint!");
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

bool RigidBody::adjacentTo(const RigidBody& other) const {
  return ((has_as_parent(other) && !(joint && joint->isFloating())) ||
          (other.has_as_parent(*this) &&
           !(other.joint && other.joint->isFloating())));
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
    : DrakeCollision::Element(other) {}

RigidBody::CollisionElement::CollisionElement(
    const Isometry3d& T_element_to_link, const RigidBody* const body)
    : DrakeCollision::Element(T_element_to_link) {
  set_body(body);
}

RigidBody::CollisionElement::CollisionElement(
    const DrakeShapes::Geometry& geometry, const Isometry3d& T_element_to_link,
    const RigidBody* const body)
    : DrakeCollision::Element(geometry, T_element_to_link) {
  set_body(body);
  // This is a temporary hack to avoid having the user to set collision
  // elements to static when added to the world.
  // Collision elements should be set to static in a later Initialize() stage as
  // described in issue #2661.
  // TODO(amcastro-tri): remove this hack.
  if (body->name() == "world") set_static();
}

RigidBody::CollisionElement* RigidBody::CollisionElement::clone() const {
  return new CollisionElement(*this);
}

bool RigidBody::CollisionElement::CollidesWith(
    const DrakeCollision::Element* other) const {
  auto other_rb = dynamic_cast<const RigidBody::CollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = get_body()->CollidesWith(*other_rb->get_body());
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
      << "  - link name: " << b.name_ << "\n"
      << "  - parent joint: " << parent_joint_name << "\n"
      << "  - Collision elements IDs: " << collision_element_str.str();

  return out;
}
