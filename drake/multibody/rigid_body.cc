#include "drake/multibody/rigid_body.h"

#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"
#include "drake/util/drakeGeometryUtil.h"

using Eigen::Isometry3d;
using Eigen::Matrix;
using Eigen::Vector3d;

using std::ostream;
using std::runtime_error;
using std::string;
using std::stringstream;
using std::vector;

template <typename T>
RigidBody<T>::RigidBody()
    : collision_filter_group_(DrakeCollision::DEFAULT_GROUP),
      collision_filter_ignores_(DrakeCollision::NONE_MASK) {
  center_of_mass_ = Vector3d::Zero();
  spatial_inertia_ << drake::SquareTwistMatrix<double>::Zero();
}

template <typename T>
const std::string& RigidBody<T>::get_name() const { return name_; }

template <typename T>
void RigidBody<T>::set_name(const std::string& name) { name_ = name; }

template <typename T>
const std::string& RigidBody<T>::get_model_name() const { return model_name_; }

template <typename T>
void RigidBody<T>::set_model_name(const std::string& name) {
    model_name_ = name;
}

template <typename T>
int RigidBody<T>::get_model_instance_id() const { return model_instance_id_; }

template <typename T>
void RigidBody<T>::set_model_instance_id(int model_instance_id) {
  model_instance_id_ = model_instance_id;
}

template <typename T>
void RigidBody<T>::setJoint(std::unique_ptr<DrakeJoint> joint) {
  joint_ = move(joint);
}

template <typename T>
const DrakeJoint& RigidBody<T>::getJoint() const {
  if (joint_) {
    return (*joint_);
  } else {
    throw runtime_error("ERROR: RigidBody<T>::getJoint(): Rigid body \"" +
                        name_ + "\" in model " + model_name_ +
                        " does not have a joint!");
  }
}

template <typename T>
void RigidBody<T>::set_parent(RigidBody* parent) { parent_ = parent; }

template <typename T>
const RigidBody<T>* RigidBody<T>::get_parent() const { return parent_; }

template <typename T>
bool RigidBody<T>::has_parent_body() const { return parent_ != nullptr; }

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
template <typename T>
bool RigidBody<T>::hasParent() const { return has_parent_body(); }

template <typename T>
void RigidBody<T>::set_body_index(int body_index) { body_index_ = body_index; }

template <typename T>
int RigidBody<T>::get_body_index() const { return body_index_; }

template <typename T>
void RigidBody<T>::set_position_start_index(int position_start_index) {
  position_start_index_ = position_start_index;
}

template <typename T>
int RigidBody<T>::get_position_start_index() const {
  return position_start_index_;
}

template <typename T>
void RigidBody<T>::set_velocity_start_index(int velocity_start_index) {
  velocity_start_index_ = velocity_start_index;
}

template <typename T>
int RigidBody<T>::get_velocity_start_index() const {
  return velocity_start_index_;
}

template <typename T>
void RigidBody<T>::AddVisualElement(const DrakeShapes::VisualElement& element) {
  visual_elements_.push_back(element);
}

template <typename T>
void RigidBody<T>::AddCollisionElementsToClique(int clique_id) {
  for (const auto& element : collision_elements_) {
    element->AddToCollisionClique(clique_id);
  }
}

template <typename T>
const DrakeShapes::VectorOfVisualElements& RigidBody<T>::get_visual_elements()
    const {
  return visual_elements_;
}

template <typename T>
void RigidBody<T>::AddCollisionElement(const std::string& group_name,
                                    DrakeCollision::Element* element) {
  DrakeCollision::ElementId id = element->getId();
  collision_element_ids_.push_back(id);
  collision_element_groups_[group_name].push_back(id);
  collision_elements_.push_back(element);
}

template <typename T>
const std::vector<DrakeCollision::ElementId>&
    RigidBody<T>::get_collision_element_ids() const {
  return collision_element_ids_;
}

template <typename T>
std::vector<DrakeCollision::ElementId>&
    RigidBody<T>::get_mutable_collision_element_ids() {
  return collision_element_ids_;
}

template <typename T>
const std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    RigidBody<T>::get_group_to_collision_ids_map() const {
  return collision_element_groups_;
}

template <typename T>
std::map<std::string, std::vector<DrakeCollision::ElementId>>&
    RigidBody<T>::get_mutable_group_to_collision_ids_map() {
  return collision_element_groups_;
}

template <typename T>
void RigidBody<T>::setCollisionFilter(const DrakeCollision::bitmask& group,
                                   const DrakeCollision::bitmask& ignores) {
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

template <typename T>
const DrakeCollision::bitmask& RigidBody<T>::getCollisionFilterGroup() const {
  return collision_filter_group_;
}

template <typename T>
void RigidBody<T>::setCollisionFilterGroup(
  const DrakeCollision::bitmask& group) {
  collision_filter_group_ = group;
}

template <typename T>
const DrakeCollision::bitmask& RigidBody<T>::getCollisionFilterIgnores() const {
  return collision_filter_ignores_;
}

template <typename T>
void RigidBody<T>::setCollisionFilterIgnores(const DrakeCollision::bitmask&
    ignores) {
  collision_filter_ignores_ = ignores;
}

template <typename T>
void RigidBody<T>::addToCollisionFilterGroup(const DrakeCollision::bitmask&
    group) {
  collision_filter_group_ |= group;
}

template <typename T>
void RigidBody<T>::ignoreCollisionFilterGroup(const DrakeCollision::bitmask&
    group) {
  collision_filter_ignores_ |= group;
}

template <typename T>
void RigidBody<T>::collideWithCollisionFilterGroup(
  const DrakeCollision::bitmask&
    group) {
  collision_filter_ignores_ &= ~group;
}

template <typename T>
bool RigidBody<T>::adjacentTo(const RigidBody& other) const {
  return ((has_as_parent(other) && !(joint_ && joint_->is_floating())) ||
          (other.has_as_parent(*this) &&
           !(other.joint_ && other.joint_->is_floating())));
}

template <typename T>
bool RigidBody<T>::CanCollideWith(const RigidBody& other) const {
  bool ignored =
      this == &other || adjacentTo(other) ||
      (collision_filter_group_ & other.getCollisionFilterIgnores()).any() ||
      (other.getCollisionFilterGroup() & collision_filter_ignores_).any();
  return !ignored;
}

template <typename T>
bool RigidBody<T>::appendCollisionElementIdsFromThisBody(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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

template <typename T>
bool RigidBody<T>::appendCollisionElementIdsFromThisBody(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<DrakeCollision::ElementId>& ids) const {
  ids.reserve(ids.size() + collision_element_ids_.size());
  ids.insert(ids.end(), collision_element_ids_.begin(),
             collision_element_ids_.end());
  return true;
}

template <typename T>
void RigidBody<T>::ApplyTransformToJointFrame(
    const Eigen::Isometry3d& transform_body_to_joint) {
  spatial_inertia_ = transformSpatialInertia(transform_body_to_joint,
      spatial_inertia_);
  for (auto& v : visual_elements_) {
    v.SetLocalTransform(transform_body_to_joint * v.getLocalTransform());
  }
}

template <typename T>
const Eigen::Matrix3Xd& RigidBody<T>::get_contact_points() const {
  return contact_points_;
}

template <typename T>
void RigidBody<T>::set_contact_points(const Eigen::Matrix3Xd& contact_points) {
  contact_points_ = contact_points;
}

template <typename T>
void RigidBody<T>::set_mass(double mass) {
  mass_ = mass;
}

template <typename T>
double RigidBody<T>::get_mass() const {
  return mass_;
}

template <typename T>
void RigidBody<T>::set_center_of_mass(const Eigen::Vector3d& center_of_mass) {
  center_of_mass_ = center_of_mass;
}

template <typename T>
const Eigen::Vector3d& RigidBody<T>::get_center_of_mass() const {
  return center_of_mass_;
}

template <typename T>
void RigidBody<T>::set_spatial_inertia(const drake::SquareTwistMatrix<double>&
    spatial_inertia) {
  spatial_inertia_ = spatial_inertia;
}

template <typename T>
const drake::SquareTwistMatrix<double>& RigidBody<T>::get_spatial_inertia()
    const {
  return spatial_inertia_;
}

ostream& operator<<(ostream& out, const RigidBody<double>& b) {
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

template <typename T>
bool RigidBody<T>::SetSelfCollisionClique(int clique_id) {
  if (collision_elements_.size() > 1) {
    AddCollisionElementsToClique(clique_id);
    return true;
  }
  return false;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBody<double>;
template class RigidBody<drake::AutoDiffUpTo73d>;
template class RigidBody<drake::AutoDiffXd>;
