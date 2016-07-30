#include "drake/systems/plants/RigidBody.h"

#include <stdexcept>

#include "drake/util/drakeGeometryUtil.h"

using drake::systems::plants::ModelElementId;
using drake::systems::plants::ModelElementType;

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
      collision_filter_ignores(DrakeCollision::NONE_MASK) {
  mass = 0.0;
  com = Vector3d::Zero();
  I << drake::SquareTwistMatrix<double>::Zero();
  model_element_id_.set_element_type(ModelElementType::kBodyElement);
}

const std::string& RigidBody::get_name() const {
  return model_element_id_.get_element_name();
}

void RigidBody::set_name(const std::string& name) {
  model_element_id_.set_element_name(name);
}

const std::string& RigidBody::get_model_name() const {
  return model_element_id_.get_model_name();
}

void RigidBody::set_model_name(const std::string& name) {
  model_element_id_.set_model_name(name);
}

int RigidBody::get_model_id() const {
  return model_element_id_.get_model_instance_id();
}

void RigidBody::set_model_id(int model_id) {
  model_element_id_.set_model_instance_id(model_id);
}

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint) {
  this->joint = move(new_joint);
}

const DrakeJoint& RigidBody::getJoint() const {
  if (joint) {
    return (*joint);
  } else {
    throw runtime_error("ERROR: RigidBody::getJoint(): Rigid body \"" +
                        get_name() +
                        "\" in model " + get_model_name() +
                        " does not have a joint!");
  }
}

void RigidBody::set_parent(RigidBody* parent) { parent_ = parent; }

const RigidBody* RigidBody::get_parent() const { return parent_; }

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

const DrakeShapes::VectorOfVisualElements& RigidBody::get_visual_elements()
    const {
  return visual_elements_;
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
  for (auto& v : visual_elements_) {
    v.SetLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
}

ostream& operator<<(ostream& out, const RigidBody& rigid_body) {
  std::string parent_joint_name =
      rigid_body.hasParent() ? rigid_body.getJoint().getName() :
          "no parent joint";

  std::stringstream collision_element_str;
  collision_element_str << "[";
  for (size_t ii = 0; ii < rigid_body.collision_element_ids.size(); ii++) {
    collision_element_str << rigid_body.collision_element_ids[ii];
    if (ii < rigid_body.collision_element_ids.size() - 1)
      collision_element_str << ", ";
  }
  collision_element_str << "]";

  out << "RigidBody\n"
      << "  - body name: " << rigid_body.get_name() << "\n"
      << "  - parent joint: " << parent_joint_name << "\n"
      << "  - Collision elements IDs: " << collision_element_str.str();

  return out;
}
