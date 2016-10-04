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
    : collision_filter_group_(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores_(DrakeCollision::NONE_MASK) {
  center_of_mass_ = Vector3d::Zero();
  spatial_inertia_ << drake::SquareTwistMatrix<double>::Zero();
}

const std::string& RigidBody::get_name() const { return name_; }

void RigidBody::set_name(const std::string& name) { name_ = name; }

const std::string& RigidBody::get_model_name() const { return model_name_; }

void RigidBody::set_model_name(const std::string& name) { model_name_ = name; }

int RigidBody::get_model_instance_id() const { return model_instance_id_; }

void RigidBody::set_model_instance_id(int model_instance_id) {
  model_instance_id_ = model_instance_id;
}

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> joint) {
  joint_ = move(joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint_) {
    return (*joint_);
  } else {
    throw runtime_error("ERROR: RigidBody::getJoint(): Rigid body \"" + name_ +
                        "\" in model " + model_name_ +
                        " does not have a joint!");
  }
}

void RigidBody::set_parent(RigidBody* parent) { parent_ = parent; }

const RigidBody* RigidBody::get_parent() const { return parent_; }

bool RigidBody::has_parent_body() const { return parent_ != nullptr; }

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
bool RigidBody::hasParent() const { return has_parent_body(); }

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

void RigidBody::AddCollisionElementsToClique(int clique_id) {
  for (const auto& element : collision_elements_) {
    element->AddToCollisionClique(clique_id);
  }
}

const DrakeShapes::VectorOfVisualElements& RigidBody::get_visual_elements()
    const {
  return visual_elements_;
}

void RigidBody::AddCollisionElement(const std::string& group_name,
                                    DrakeCollision::Element* element) {
  DrakeCollision::ElementId id = element->getId();
  collision_element_ids_.push_back(id);
  collision_element_groups_[group_name].push_back(id);
  collision_elements_.push_back(element);
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
    RigidBody::get_group_to_collision_ids_map() const {
  return collision_element_groups_;
}

std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    RigidBody::get_mutable_group_to_collision_ids_map() {
  return collision_element_groups_;
}

void RigidBody::setCollisionFilter(const DrakeCollision::bitmask& group,
                                   const DrakeCollision::bitmask& ignores) {
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

const DrakeCollision::bitmask& RigidBody::getCollisionFilterGroup() const {
  return collision_filter_group_;
}
void RigidBody::setCollisionFilterGroup(const DrakeCollision::bitmask& group) {
  collision_filter_group_ = group;
}

const DrakeCollision::bitmask& RigidBody::getCollisionFilterIgnores() const {
  return collision_filter_ignores_;
}
void RigidBody::setCollisionFilterIgnores(const DrakeCollision::bitmask&
    ignores) {
  collision_filter_ignores_ = ignores;
}

void RigidBody::addToCollisionFilterGroup(const DrakeCollision::bitmask&
    group) {
  collision_filter_group_ |= group;
}
void RigidBody::ignoreCollisionFilterGroup(const DrakeCollision::bitmask&
    group) {
  collision_filter_ignores_ |= group;
}
void RigidBody::collideWithCollisionFilterGroup(const DrakeCollision::bitmask&
    group) {
  collision_filter_ignores_ &= ~group;
}

bool RigidBody::adjacentTo(const RigidBody& other) const {
  return ((has_as_parent(other) && !(joint_ && joint_->is_floating())) ||
          (other.has_as_parent(*this) &&
           !(other.joint_ && other.joint_->is_floating())));
}

bool RigidBody::CanCollideWith(const RigidBody& other) const {
  bool ignored =
      this == &other || adjacentTo(other) ||
      (collision_filter_group_ & other.getCollisionFilterIgnores()).any() ||
      (other.getCollisionFilterGroup() & collision_filter_ignores_).any();
  return !ignored;
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
  spatial_inertia_ = transformSpatialInertia(transform_body_to_joint,
      spatial_inertia_);
  for (auto& v : visual_elements_) {
    v.SetLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
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

void RigidBody::set_spatial_inertia(const drake::SquareTwistMatrix<double>&
    spatial_inertia) {
  spatial_inertia_ = spatial_inertia;
}

const drake::SquareTwistMatrix<double>& RigidBody::get_spatial_inertia()
    const {
  return spatial_inertia_;
}

ostream& operator<<(ostream& out, const RigidBody& b) {
  std::string parent_joint_name =
      b.has_parent_body() ? b.getJoint().get_name() : "no parent joint";

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

bool RigidBody::SetSelfCollisionClique(int clique_id) {
  if (collision_elements_.size() > 1) {
    AddCollisionElementsToClique(clique_id);
    return true;
  }
  return false;
}
