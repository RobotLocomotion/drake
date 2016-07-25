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
      parent_(nullptr) {
  model_id_ = 0;
  position_start_index_ = 0;
  velocity_start_index_ = 0;
  body_index_ = 0;
  mass_ = 0.0;
  center_of_mass_ = Vector3d::Zero();
  I << drake::SquareTwistMatrix<double>::Zero();
}

const std::string& RigidBody::get_name() const { return name_; }

void RigidBody::set_name(const std::string& name) { name_ = name; }

const std::string& RigidBody::get_model_name() const { return model_name_; }

void RigidBody::set_model_name(const std::string& name) { model_name_ = name; }

int RigidBody::get_model_id() const { return model_id_; }

void RigidBody::set_model_id(int model_id) { model_id_ = model_id; }

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

void RigidBody::set_parent(RigidBody* parent) { parent_ = parent; }

const RigidBody* RigidBody::get_parent() const { return parent_; }

RigidBody* RigidBody::get_mutable_parent() { return parent_; }

bool RigidBody::hasParent() const { return parent_ != nullptr; }

void RigidBody::set_body_index(int body_index) { body_index_ = body_index; }

int RigidBody::get_body_index() const { return body_index_; }

void RigidBody::set_position_start_index(int position_start_index) {
  position_start_index_ = position_start_index;
}

int RigidBody::get_position_start_index() const {
  return position_start_index_;
}

void RigidBody::set_velocity_start_index(int velocity_start_index) {
  velocity_start_index_ = velocity_start_index;
}

int RigidBody::get_velocity_start_index() const {
  return velocity_start_index_;
}

void RigidBody::AddVisualElement(const DrakeShapes::VisualElement& element) {
  visual_elements_.push_back(element);
}

const DrakeShapes::VectorOfVisualElements& RigidBody::GetVisualElements()
    const {
  return visual_elements_;
}

void RigidBody::AddCollisionElement(DrakeCollision::ElementId id) {
  collision_element_ids_.push_back(id);
}

void RigidBody::AddCollisionElementToGroup(const std::string& group_name,
    DrakeCollision::ElementId id) {
  collision_element_groups_[group_name].push_back(id);
}

const std::vector<DrakeCollision::ElementId>&
    RigidBody::get_collision_element_ids() const {
  return collision_element_ids_;
}

std::vector<DrakeCollision::ElementId>&
    RigidBody::get_mutable_collision_element_ids() {
  return collision_element_ids_;
}

const std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    RigidBody::get_collision_element_groups() const {
  return collision_element_groups_;
}

std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    RigidBody::get_mutable_collision_element_groups() {
  return collision_element_groups_;
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
  auto group_ids_iter = collision_element_groups_.find(group_name);
  if (group_ids_iter != collision_element_groups_.end()) {
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
  ids.reserve(ids.size() + collision_element_ids_.size());
  ids.insert(ids.end(), collision_element_ids_.begin(),
             collision_element_ids_.end());
  return true;
}

void RigidBody::ApplyTransformToJointFrame(
    const Eigen::Isometry3d& transform_body_to_joint) {
  I = transformSpatialInertia(transform_body_to_joint, I);
  for (auto& v : visual_elements_) {
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
  if (body->get_name() == "world") set_static();
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

const Eigen::Matrix3Xd& RigidBody::get_contact_points() const {
  return contact_points_;
}

void RigidBody::set_contact_points(const Eigen::Matrix3Xd& contact_points) {
  contact_points_ = contact_points;
}

void RigidBody::set_mass(double mass) {
  mass_ = mass;
}

double RigidBody::get_mass() const {
  return mass_;
}

void RigidBody::set_center_of_mass(const Eigen::Vector3d& center_of_mass) {
  center_of_mass_ = center_of_mass;
}

const Eigen::Vector3d& RigidBody::get_center_of_mass() const {
  return center_of_mass_;
}

ostream& operator<<(ostream& out, const RigidBody& b) {
  std::string parent_joint_name =
      b.hasParent() ? b.getJoint().getName() : "no parent joint";

  std::stringstream collision_element_str;
  collision_element_str << "[";
  for (size_t ii = 0; ii < b.get_collision_element_ids().size(); ii++) {
    collision_element_str << b.get_collision_element_ids()[ii];
    if (ii < b.get_collision_element_ids().size() - 1)
      collision_element_str << ", ";
  }
  collision_element_str << "]";

  out << "RigidBody\n"
      << "  - link name: " << b.name_ << "\n"
      << "  - parent joint: " << parent_joint_name << "\n"
      << "  - Collision elements IDs: " << collision_element_str.str();

  return out;
}
