#include "drake/multibody/rigid_body_tree.h"

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>

#include "drake/common/constants.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematics_cache-inl.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/drakeUtil.h"

using Eigen::AutoDiffScalar;
using Eigen::Dynamic;
using Eigen::Isometry3d;
using Eigen::Isometry;
using Eigen::Map;
using Eigen::Matrix3Xd;
using Eigen::Matrix4Xd;
using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::MatrixXd;
using Eigen::Transform;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::AutoDiffUpTo73d;
using drake::AutoDiffXd;
using drake::Isometry3;
using drake::Matrix3X;
using drake::Matrix4X;
using drake::Matrix6X;
using drake::MatrixX;
using drake::TwistMatrix;
using drake::TwistVector;
using drake::Vector3;
using drake::Vector6;
using drake::VectorX;
using drake::WrenchVector;
using drake::kQuaternionSize;
using drake::kRpySize;
using drake::kSpaceDimension;
using drake::kTwistSize;

using drake::math::autoDiffToGradientMatrix;
using drake::math::gradientMatrixToAutoDiff;
using drake::math::Gradient;
using drake::multibody::joints::FloatingBaseType;

using DrakeCollision::CollisionFilterGroup;

using std::allocator;
using std::cerr;
using std::cout;
using std::equal_to;
using std::hash;
using std::less;
using std::make_unique;
using std::map;
using std::ofstream;
using std::pair;
using std::runtime_error;
using std::set;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using std::endl;

const char* const RigidBodyTreeConstants::kWorldName = "world";
const int RigidBodyTreeConstants::kWorldBodyIndex = 0;
// TODO(liang.fok) Update the following two variables along with the resolution
// of #3088. Once #3088 is resolved, ID of the first model instance should be 1.
const int RigidBodyTreeConstants::kFirstNonWorldModelInstanceId = 0;
const set<int> RigidBodyTreeConstants::default_model_instance_id_set = {0};

template <typename T>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void getFiniteIndexes(T const& v, std::vector<int>& finite_indexes) {
  finite_indexes.clear();
  for (int i = 0; i < v.size(); ++i) {
    if (std::isfinite(static_cast<double>(v[i]))) {
      finite_indexes.push_back(i);
    }
  }
}

std::ostream& operator<<(std::ostream& os, const RigidBodyTree<double>& tree) {
  os << *tree.collision_model_.get();
  return os;
}

template <typename T>
RigidBodyTree<T>::RigidBodyTree()
    : collision_model_(DrakeCollision::newModel()) {
  // Sets the gravity vector.
  a_grav << 0, 0, 0, 0, 0, -9.81;

  // Adds the rigid body representing the world. It has model instance ID 0.
  std::unique_ptr<RigidBody<T>> world_body(new RigidBody<T>());
  world_body->set_name(RigidBodyTreeConstants::kWorldName);
  world_body->set_model_name(RigidBodyTreeConstants::kWorldName);

  // TODO(liang.fok): Assign the world body a unique model instance ID of zero.
  // See: https://github.com/RobotLocomotion/drake/issues/3088

  bodies.push_back(std::move(world_body));
}

template <typename T>
RigidBodyTree<T>::~RigidBodyTree() {}

template <>
unique_ptr<RigidBodyTree<double>> RigidBodyTree<double>::Clone() const {
  auto clone = make_unique<RigidBodyTree<double>>();
  // The following is necessary to remove the world link from the clone. The
  // world link will be re-added when the bodies are cloned below.
  clone->bodies.clear();

  clone->joint_limit_min = this->joint_limit_min;
  clone->joint_limit_max = this->joint_limit_max;
  clone->a_grav = this->a_grav;
  clone->B = this->B;
  clone->num_positions_ = this->num_positions_;
  clone->num_velocities_ = this->num_velocities_;
  clone->num_model_instances_ = this->num_model_instances_;
  clone->initialized_ = this->initialized_;

  // Clones the rigid bodies.
  for (const auto& body : bodies) {
    clone->bodies.push_back(body->Clone());
  }

  // Clones the joints and adds them to the cloned RigidBody objects.
  for (const auto& original_body : bodies) {
    const int body_index = original_body->get_body_index();
    if (body_index == RigidBodyTreeConstants::kWorldBodyIndex) {
      continue;
    }

    RigidBody<double>* cloned_body = clone->get_mutable_body(body_index);
    DRAKE_DEMAND(cloned_body != nullptr);
    DRAKE_DEMAND(cloned_body->get_body_index() == body_index);

    const RigidBody<double>* original_body_parent = original_body->get_parent();
    DRAKE_DEMAND(original_body_parent != nullptr);

    const int parent_body_index = original_body_parent->get_body_index();

    RigidBody<double>* cloned_body_parent =
        clone->get_mutable_body(parent_body_index);
    DRAKE_DEMAND(cloned_body_parent != nullptr);

    cloned_body->add_joint(cloned_body_parent,
                           original_body->getJoint().Clone());
  }

  for (const auto& original_frame : frames) {
    const RigidBody<double>& original_frame_body =
        original_frame->get_rigid_body();
    const int cloned_frame_body_index =
        clone->FindBodyIndex(original_frame_body.get_name(),
                             original_frame_body.get_model_instance_id());
    RigidBody<double>* cloned_frame_body =
        clone->get_mutable_body(cloned_frame_body_index);
    DRAKE_DEMAND(cloned_frame_body != nullptr);
    std::shared_ptr<RigidBodyFrame<double>> cloned_frame =
        original_frame->Clone(cloned_frame_body);
    clone->frames.push_back(cloned_frame);
  }

  for (const auto& actuator : actuators) {
    const RigidBody<double>& cloned_body =
        clone->get_body(actuator.body_->get_body_index());
    clone->actuators.emplace_back(
        actuator.name_, &cloned_body, actuator.reduction_,
        actuator.effort_limit_min_, actuator.effort_limit_max_);
  }

  for (const auto& loop : loops) {
    std::shared_ptr<RigidBodyFrame<double>> frame_a =
        clone->findFrame(loop.frameA_->get_name(),
            loop.frameA_->get_model_instance_id());
    std::shared_ptr<RigidBodyFrame<double>> frame_b =
        clone->findFrame(loop.frameB_->get_name(),
            loop.frameB_->get_model_instance_id());
    clone->loops.emplace_back(frame_a, frame_b, loop.axis_);
  }

  return clone;
}

template <typename T>
bool RigidBodyTree<T>::transformCollisionFrame(
    RigidBody<T>* body, const Eigen::Isometry3d& displace_transform) {
  // Collision elements in the body-collision map have *not* been registered
  // with the collision engine yet and can simply be modified in
  // place.
  auto map_itr = body_collision_map_.find(body);
  if (map_itr != body_collision_map_.end()) {
    BodyCollisions& collision_items = map_itr->second;
    for (const auto& item : collision_items) {
      element_order_[item.element]->SetLocalTransform(
          displace_transform *
          element_order_[item.element]->getLocalTransform());
    }
  }

  // TODO(SeanCurtis-TRI): These Collision elements have *already* been
  // registered with the collision model; they must be moved through the
  // collision model's interface. We need to decide if a method that is intended
  // to be called as part of *construction* should modify collision elements
  // already registered with the collision engine. In other words, do we allow
  // the following work flow:
  //   1) Add body to tree.
  //   2) Add collision element to body.
  //   3) transform the collision frame.
  //   4) *compile*
  //   5) repeate steps 2-4 on that same body.
  // I suspect this should *not* be considered a valid workflow but still needs
  // to be officially decided.
  for (auto body_itr = body->collision_elements_begin();
       body_itr != body->collision_elements_end(); ++body_itr) {
    DrakeCollision::Element* element = *body_itr;
    if (!collision_model_->transformCollisionFrame(element->getId(),
                                                   displace_transform)) {
      return false;
    }
  }
  return true;
}

// TODO(amcastro-tri): This implementation is very inefficient since member
// vector RigidBodyTree::bodies changes in size with the calls to
// bodies.erase and bodies.insert.
// A possibility would be to use std::sort or our own version of a quick sort.
template <typename T>
void RigidBodyTree<T>::SortTree() {
  if (bodies.size() == 0) return;  // no-op if there are no RigidBody's

  for (size_t i = 0; i < bodies.size() - 1;) {
    if (bodies[i]->has_parent_body()) {
      auto iter = std::find_if(bodies.begin() + i + 1, bodies.end(),
                               [&](std::unique_ptr<RigidBody<T>> const& p) {
                                 return bodies[i]->has_as_parent(*p);
                               });
      if (iter != bodies.end()) {
        std::unique_ptr<RigidBody<T>> parent = std::move(*iter);
        bodies.erase(iter);
        bodies.insert(bodies.begin() + i, std::move(parent));
        --i;
      }
    }
    ++i;
  }

  // Re-assign body_index to be the i-th entry in RBT::bodies
  for (size_t i = 0; i < bodies.size(); ++i) {
    bodies[i]->set_body_index(static_cast<int>(i));
  }
}

template <typename T>
const RigidBodyActuator& RigidBodyTree<T>::GetActuator(
    const std::string& name) const {
  for (const auto& actuator : actuators) {
    if (actuator.name_ == name) {
      return actuator;
    }
  }
  throw std::invalid_argument("ERROR: Could not find actuator named \"" + name +
      "\"");
}

template <typename T>
void RigidBodyTree<T>::DefineCollisionFilterGroup(const std::string& name) {
  collision_group_manager_.DefineCollisionFilterGroup(name);
}

template <typename T>
void RigidBodyTree<T>::AddCollisionFilterGroupMember(
    const std::string& group_name, const std::string& body_name, int model_id) {
  int body_index = FindBodyIndex(body_name, model_id);
  RigidBody<T>* body = bodies[body_index].get();
  if (body->get_num_collision_elements() > 0) {
    throw std::runtime_error("Attempting to add a body, '" + body->get_name() +
                             "', to a collision group, '" +
                             group_name +
                             "' that has already been compiled with "
                             "collision elements.");
  }
  if (!collision_group_manager_.AddCollisionFilterGroupMember(group_name,
                                                              *body)) {
    throw std::runtime_error(
        "Attempting to add a link to an undefined collision filter group: "
        "Adding " +
        body->get_name() + " to " + group_name + ".");
  }
}

template <typename T>
void RigidBodyTree<T>::AddCollisionFilterIgnoreTarget(
    const std::string& group_name, const std::string& target_group_name) {
  collision_group_manager_.AddCollisionFilterIgnoreTarget(group_name,
                                                          target_group_name);
}

template <typename T>
void RigidBodyTree<T>::SetBodyCollisionFilters(
    const RigidBody<T>& body, const DrakeCollision::bitmask& group,
    const DrakeCollision::bitmask& ignores) {
  collision_group_manager_.SetBodyCollisionFilters(body, group, ignores);
}

template <typename T>
void RigidBodyTree<T>::compile() {
  SortTree();

  // Welds joints for links that have zero inertia and no children (as seen in
  // pr2.urdf)
  // TODO(amcastro-tri): this is O(n^2). RigidBody should contain a list of
  // children
  // TODO(amcastro-tri): the order in which these loops should be performed
  // should be stated more clearly with an iterator.
  // An option would be to have:
  //   RigidBodyTree::upwards_body_iterator: travels the tree upwards towards
  //   the root.
  //   RigidBodyTree::downwards_body_iterator: travels the tree downwards
  //   from the root towards the last leaf.
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i]->has_parent_body() &&
        bodies[i]->get_spatial_inertia().isConstant(0)) {
      bool hasChild = false;
      for (size_t j = i + 1; j < bodies.size(); ++j) {
        if (bodies[j]->has_as_parent(*bodies[i])) {
          hasChild = true;
          break;
        }
      }
      if (!hasChild) {
        // now check if this body is attached by a loop joint
        for (const auto& loop : loops) {
          if ((loop.frameA_->has_as_rigid_body(bodies[i].get())) ||
              (loop.frameB_->has_as_rigid_body(bodies[i].get()))) {
            hasChild = true;
            break;
          }
        }
      }
      if (!hasChild) {
        cout << "Welding joint " << bodies[i]->getJoint().get_name() << endl;
        std::unique_ptr<DrakeJoint> joint_unique_ptr(new FixedJoint(
            bodies[i]->getJoint().get_name(),
            bodies[i]->getJoint().get_transform_to_parent_body()));
        bodies[i]->setJoint(std::move(joint_unique_ptr));
      }
    }
  }

  // Counts the number of position and velocity states in this rigid body tree.
  // Notice that the rigid bodies are accessed from the sorted vector
  // RigidBodyTree::bodies. The order that they appear in this vector
  // determines the values of RigidBody::get_position_start_index() and
  // RigidBody::get_velocity_start_index(), which the following code sets.
  num_positions_ = 0;
  num_velocities_ = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      body.set_position_start_index(num_positions_);
      num_positions_ += body.getJoint().get_num_positions();
      body.set_velocity_start_index(num_velocities_);
      num_velocities_ += body.getJoint().get_num_velocities();
    } else {
      body.set_position_start_index(0);
      body.set_velocity_start_index(0);
    }
  }

  B.resize(num_velocities_, actuators.size());
  B = MatrixXd::Zero(num_velocities_, actuators.size());
  for (size_t ia = 0; ia < actuators.size(); ++ia) {
    for (int i = 0; i < actuators[ia].body_->getJoint().get_num_velocities();
         ++i) {
      B(actuators[ia].body_->get_velocity_start_index() + i, ia) =
          actuators[ia].reduction_;
    }
  }

  // Initializes the joint limit vectors.
  joint_limit_min = VectorXd::Constant(
      num_positions_, -std::numeric_limits<double>::infinity());
  joint_limit_max = VectorXd::Constant(num_positions_,
                                       std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < bodies.size(); ++i) {
    auto& body = bodies[i];
    if (body->has_parent_body()) {
      const DrakeJoint& joint = body->getJoint();
      joint_limit_min.segment(body->get_position_start_index(),
                              joint.get_num_positions()) =
          joint.getJointLimitMin();
      joint_limit_max.segment(body->get_position_start_index(),
                              joint.get_num_positions()) =
          joint.getJointLimitMax();
    }
  }

  ConfirmCompleteTree();

  CompileCollisionState();

  initialized_ = true;
}

template <typename T>
void RigidBodyTree<T>::CompileCollisionState() {
  // Identifies and processes collision elements that should be marked
  // "anchored".
  for (const auto& pair : body_collision_map_) {
    RigidBody<T>* body = pair.first;
    if (body->IsRigidlyFixedToWorld()) {
      const BodyCollisions& elements = pair.second;
      for (const auto& collision_item : elements) {
        element_order_[collision_item.element]->set_anchored();
        element_order_[collision_item.element]->updateWorldTransform(
            body->ComputeWorldFixedPose());
      }
    }
  }

  // Process collision filter groups
  collision_group_manager_.CompileGroups();

  // Set the collision filter data on the body's elements. Note: this does
  // *not* update the collision elements that may have already been registered
  // with the collision model. But attempts to add bodies with registered
  // collision elements to a collision filter group, should have already thrown
  // an exception.
  for (auto& pair : body_collision_map_) {
    RigidBody<T>* body = pair.first;
    DrakeCollision::bitmask group =
        collision_group_manager_.get_group_mask(*body);
    DrakeCollision::bitmask ignore =
        collision_group_manager_.get_ignore_mask(*body);
    BodyCollisions& elements = pair.second;
    for (const auto& collision_item : elements) {
      element_order_[collision_item.element]->set_collision_filter(group,
                                                                   ignore);
    }
  }
  collision_group_manager_.Clear();

  // Builds cliques for collision filtering.
  CreateCollisionCliques();

  // Computes the contact points for a body.
  for (auto& pair : body_collision_map_) {
    RigidBody<T>* body = pair.first;
    Eigen::Matrix3Xd contact_points;
    BodyCollisions& elements = pair.second;
    int num_points = 0;
    // Note: contact points does *not* rely on collision element group names.
    for (const auto& collision_item : elements) {
      Matrix3Xd element_points;
      element_order_[collision_item.element]->getTerrainContactPoints(
          element_points);
      contact_points.conservativeResize(
          Eigen::NoChange, contact_points.cols() + element_points.cols());
      contact_points.block(0, num_points, contact_points.rows(),
                           element_points.cols()) = element_points;
      num_points += element_points.cols();
    }
    body->set_contact_points(contact_points);
  }

  // Assigns finished collision elements to their corresponding rigid bodies.
  for (auto& pair : body_collision_map_) {
    RigidBody<T>* body = pair.first;
    BodyCollisions& elements = pair.second;
    for (const auto& collision_item : elements) {
      body->AddCollisionElement(collision_item.group_name,
                                element_order_[collision_item.element].get());
    }
  }

  // Registers collision elements in the instantiation order to guarantee
  // deterministic results. See Model::AddElement for details.
  // NOTE: Do *not* attempt to use the elements in the body_collision_map after
  // this loop; the collision elements will have been moved into the collision
  // model.
  for (size_t i = 0; i < element_order_.size(); ++i) {
    collision_model_->AddElement(std::move(element_order_[i]));
  }
  body_collision_map_.clear();
  element_order_.clear();
}

template <typename T>
void RigidBodyTree<T>::CreateCollisionCliques() {
  int clique_id = get_next_clique_id();
  // Marks collision elements in the same body to be in the same clique.
  for (auto& pair : body_collision_map_) {
    BodyCollisions& collision_items = pair.second;
    if ( collision_items.size() > 1 ) {
      for (auto& item : collision_items) {
        element_order_[item.element]->AddToCollisionClique(clique_id);
      }
      clique_id = get_next_clique_id();
    }
  }

  // Collision elements on "adjacent" bodies belong in the same clique.
  // This allows coarse link collision geometry. This coarse geometry might
  // superficially collide, but not represent a *physical* collision.  Instead,
  // it is assumed that constraints on the relative poses of adjacent links is
  // determined by joint limits.
  // This is an O(N^2) loop -- but only happens at initialization.
  // If this proves to be too expensive, walking the tree would be O(N)
  // and still capture all of the adjacency.
  // TODO(SeanCurtis-TRI): If compile gets called multiple times this will end
  // up encoding redundant cliques.
  for (size_t i = 0; i < bodies.size(); ++i) {
    RigidBody<T>* body_i = bodies[i].get();
    for (size_t j = i + 1; j < bodies.size(); ++j) {
      RigidBody<T>* body_j = bodies[j].get();
      if (!body_i->CanCollideWith(*body_j)) {
        BodyCollisions& elements_i =  body_collision_map_[body_i];
        for (const auto& item : elements_i) {
          element_order_[item.element]->AddToCollisionClique(clique_id);
        }
        BodyCollisions& elements_j =  body_collision_map_[body_j];
        for (const auto& item : elements_j) {
          element_order_[item.element]->AddToCollisionClique(clique_id);
        }
        clique_id = get_next_clique_id();
      }
    }
  }
}

template <typename T>
Eigen::VectorXd RigidBodyTree<T>::getZeroConfiguration() const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies) {
    if (body_ptr->has_parent_body()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(
          body_ptr->get_position_start_index(), joint.get_num_positions()) =
          joint.zeroConfiguration();
    }
  }
  return q;
}

template <typename T>
Eigen::VectorXd RigidBodyTree<T>::getRandomConfiguration(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator) const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies) {
    if (body_ptr->has_parent_body()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(
          body_ptr->get_position_start_index(), joint.get_num_positions()) =
          joint.randomConfiguration(generator);
    }
  }
  return q;
}

template <typename T>
string RigidBodyTree<T>::get_position_name(int position_num) const {
  if (position_num < 0 || position_num >= num_positions_)
    throw std::runtime_error("position_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
      bodies[body_index + 1]->get_position_start_index() <= position_num)
    body_index++;

  return bodies[body_index]->getJoint().get_position_name(
      position_num - bodies[body_index]->get_position_start_index());
}

template <typename T>
string RigidBodyTree<T>::get_velocity_name(int velocity_num) const {
  if (velocity_num < 0 || velocity_num >= num_velocities_)
    throw std::runtime_error("velocity_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
      bodies[body_index + 1]->get_velocity_start_index() <= velocity_num)
    body_index++;

  return bodies[body_index]->getJoint().get_velocity_name(
      velocity_num - bodies[body_index]->get_velocity_start_index());
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
template <typename T>
std::string RigidBodyTree<T>::getPositionName(int position_num) const {
  return get_position_name(position_num);
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
template <typename T>
std::string RigidBodyTree<T>::getVelocityName(int velocity_num) const {
  return get_velocity_name(velocity_num);
}

template <typename T>
string RigidBodyTree<T>::getStateName(int state_num) const {
  if (state_num < num_positions_)
    return get_position_name(state_num);
  else
    return get_velocity_name(state_num - num_positions_);
}

template <typename T>
void RigidBodyTree<T>::drawKinematicTree(
    std::string graphviz_dotfile_filename) const {
  ofstream dotfile;
  dotfile.open(graphviz_dotfile_filename);
  dotfile << "digraph {" << endl;
  for (const auto& body : bodies) {
    dotfile << "  " << body->get_name() << " [label=\"" << body->get_name()
            << endl;
    dotfile << "mass=" << body->get_mass()
            << ", com=" << body->get_center_of_mass().transpose() << endl;
    dotfile << "spatial inertia=" << endl
            << body->get_spatial_inertia() << endl;
    dotfile << "\"];" << endl;
    if (body->has_parent_body()) {
      const auto& joint = body->getJoint();
      dotfile << "  " << body->get_name() << " -> "
              << body->get_parent()->get_name() << " [label=\""
              << joint.get_name() << endl;
      dotfile << "transform_to_parent_body=" << endl
              << joint.get_transform_to_parent_body().matrix() << endl;
      //     dotfile << "axis=" << endl << joint.get().matrix() << endl;
      dotfile << "\"];" << endl;
    }
  }
  for (const auto& frame : frames) {
    dotfile << "  " << frame->get_name() << " [label=\"" << frame->get_name()
            << " (frame)\"];" << endl;
    dotfile << "  " << frame->get_name() << " -> "
            << frame->get_rigid_body().get_name() << " [label=\"";
    dotfile << "transform_to_body=" << endl
            << frame->get_transform_to_body().matrix() << endl;
    dotfile << "\"];" << endl;
  }

  for (const auto& loop : loops) {
    dotfile << "  " << loop.frameA_->get_rigid_body().get_name() << " -> "
            << loop.frameB_->get_rigid_body().get_name() << " [label=\"loop "
            << endl;
    dotfile << "transform_to_parent_body=" << endl
            << loop.frameA_->get_transform_to_body().matrix() << endl;
    dotfile << "transform_to_child_body=" << endl
            << loop.frameB_->get_transform_to_body().matrix() << endl;
    dotfile << "\"];" << endl;
  }
  dotfile << "}" << endl;
  dotfile.close();
  cout << "Wrote kinematic tree to " << graphviz_dotfile_filename
       << "; run e.g. 'dot -Tpng -O " << graphviz_dotfile_filename
       << "' to generate an image." << endl;
}

template <typename T>
map<string, int> RigidBodyTree<T>::computePositionNameToIndexMap() const {
  map<string, int> name_to_index_map;

  for (int i = 0; i < num_positions_; ++i) {
    name_to_index_map[get_position_name(i)] = i;
  }
  return name_to_index_map;
}

template <typename T>
void RigidBodyTree<T>::addCollisionElement(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    const DrakeCollision::Element& element, RigidBody<T>& body,
    const string& group_name) {
  auto itr = body_collision_map_.find(&body);
  if (itr == body_collision_map_.end()) {
    // NOTE: we do this instead of map[key] = value because we want an iterator
    // to the newly inserted list for use in the remainder of the function.
    bool success;
    std::tie(itr, success) =
        body_collision_map_.insert(std::make_pair(&body, BodyCollisions()));

    if (!success) {
      throw std::runtime_error(
          "Unable to add the collision element to the "
          "body: " +
          body.get_name() + ".");
    }
  }
  BodyCollisions& body_collisions = itr->second;
  size_t id = element_order_.size();
  element_order_.emplace_back(
      std::unique_ptr<DrakeCollision::Element>(element.clone()));
  body_collisions.emplace_back(group_name, id);
}

template <typename T>
void RigidBodyTree<T>::updateCollisionElements(
    const RigidBody<T>& body,
    const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world) {
  for (auto id_iter = body.get_collision_element_ids().begin();
       id_iter != body.get_collision_element_ids().end(); ++id_iter) {
    collision_model_->updateElementWorldTransform(*id_iter, transform_to_world);
  }
}

template <typename T>
void RigidBodyTree<T>::updateDynamicCollisionElements(
    const KinematicsCache<double>& cache) {
  CheckCacheValidity(cache);
  // todo: this is currently getting called many times with the same cache
  // object.  and it's presumably somewhat expensive.
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      updateCollisionElements(
          body, cache.get_element(body.get_body_index()).transform_to_world);
    }
  }
  collision_model_->updateModel();
}

template <typename T>
void RigidBodyTree<T>::getTerrainContactPoints(
    const RigidBody<T>& body,
    Eigen::Matrix3Xd* terrain_points,
    const std::string& group_name) const {
  // Ensures terrain_points is a valid pointer.
  DRAKE_DEMAND(terrain_points);

  // Clears matrix before filling it again.
  size_t num_points = 0;
  terrain_points->resize(Eigen::NoChange, 0);

  vector<DrakeCollision::ElementId> element_ids;

  // If a group name was given, use it to look up the subset of collision
  // elements that belong to that group.  Otherwise, use the full set of
  // collision elements that belong to the body.
  if (!group_name.empty()) {
    auto group_map = body.get_group_to_collision_ids_map();
    auto map_iter = group_map.find(group_name);
    if (map_iter == group_map.end()) {
      throw runtime_error(
          "RigidBodyTree::getTerrainContactPoints: "
          "could not find collision group named: " +
          group_name);
    }

    element_ids = map_iter->second;
  } else {
    element_ids = body.get_collision_element_ids();
  }

  // Iterates through each collision element in the rigid body. For each
  // collision element, obtain its contact points with the terrain and add it to
  // to the matrix pointed to by parameter terrain_points.
  for (auto id_iter = element_ids.begin();
       id_iter != element_ids.end(); ++id_iter) {
    Matrix3Xd element_points;
    collision_model_->getTerrainContactPoints(*id_iter, element_points);
    terrain_points->conservativeResize(
        Eigen::NoChange, terrain_points->cols() + element_points.cols());
    terrain_points->block(0, num_points, terrain_points->rows(),
                          element_points.cols()) = element_points;
    num_points += element_points.cols();
  }
}

template <typename T>
void RigidBodyTree<T>::collisionDetectFromPoints(
    const KinematicsCache<double>& cache, const Matrix3Xd& points,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& phi, Matrix3Xd& normal, Matrix3Xd& x, Matrix3Xd& body_x,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& body_idx, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> closest_points;

  collision_model_->collisionDetectFromPoints(points, use_margins,
                                              closest_points);
  x.resize(3, closest_points.size());
  body_x.resize(3, closest_points.size());
  normal.resize(3, closest_points.size());
  phi.resize(closest_points.size());

  for (size_t i = 0; i < closest_points.size(); ++i) {
    x.col(i) = closest_points[i].ptB;
    body_x.col(i) = closest_points[i].ptA;
    normal.col(i) = closest_points[i].normal;
    phi[i] = closest_points[i].distance;
    const DrakeCollision::Element* elementB = closest_points[i].elementB;
    body_idx.push_back(elementB->get_body()->get_body_index());
  }
}

template <typename T>
bool RigidBodyTree<T>::collisionRaycast(
    const KinematicsCache<double>& cache,
    const Matrix3Xd& origins,
    const Matrix3Xd& ray_endpoints,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& distances, bool use_margins) {
  Matrix3Xd normals;
  updateDynamicCollisionElements(cache);
  return collision_model_->collisionRaycast(origins, ray_endpoints, use_margins,
                                            distances, normals);
}

template <typename T>
bool RigidBodyTree<T>::collisionRaycast(
    const KinematicsCache<double>& cache,
    const Matrix3Xd& origins,
    const Matrix3Xd& ray_endpoints,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& distances, Matrix3Xd& normals,
    bool use_margins) {
  updateDynamicCollisionElements(cache);
  return collision_model_->collisionRaycast(origins, ray_endpoints, use_margins,
                                            distances, normals);
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx,
    const vector<DrakeCollision::ElementId>& ids_to_check, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  bool points_found =
      collision_model_->closestPointsAllToAll(ids_to_check, use_margins,
                                              points);

  xA = MatrixXd::Zero(3, points.size());
  xB = MatrixXd::Zero(3, points.size());
  normal = MatrixXd::Zero(3, points.size());
  phi = VectorXd::Zero(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    xA.col(i) = points[i].ptA;
    xB.col(i) = points[i].ptB;
    normal.col(i) = points[i].normal;
    phi[i] = points[i].distance;
    const DrakeCollision::Element* elementA = points[i].elementA;
    bodyA_idx.push_back(elementA->get_body()->get_body_index());
    const DrakeCollision::Element* elementB = points[i].elementB;
    bodyB_idx.push_back(elementB->get_body()->get_body_index());
  }
  return points_found;
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx, const vector<int>& bodies_idx,
    const set<string>& active_element_groups, bool use_margins) {
  CheckCacheValidity(cache);
  vector<DrakeCollision::ElementId> ids_to_check;
  for (const int& body_idx : bodies_idx) {
    if (body_idx >= 0 && body_idx < static_cast<int>(bodies.size())) {
      for (const string& group : active_element_groups) {
        bodies[body_idx]->appendCollisionElementIdsFromThisBody(group,
                                                                ids_to_check);
      }
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx, const vector<int>& bodies_idx, bool use_margins) {
  CheckCacheValidity(cache);
  vector<DrakeCollision::ElementId> ids_to_check;
  for (const int& body_idx : bodies_idx) {
    if (body_idx >= 0 && body_idx < static_cast<int>(bodies.size())) {
      bodies[body_idx]->appendCollisionElementIdsFromThisBody(ids_to_check);
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    const KinematicsCache<double>& cache,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx,
    const set<string>& active_element_groups,
    bool use_margins) {
  CheckCacheValidity(cache);
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_iter = bodies.begin(); body_iter != bodies.end();
       ++body_iter) {
    for (auto group_iter = active_element_groups.begin();
         group_iter != active_element_groups.end(); ++group_iter) {
      (*body_iter)
          ->appendCollisionElementIdsFromThisBody(*group_iter, ids_to_check);
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    const KinematicsCache<double>& cache,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx, bool use_margins) {
  CheckCacheValidity(cache);
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_iter = bodies.begin(); body_iter != bodies.end();
       ++body_iter) {
    (*body_iter)->appendCollisionElementIdsFromThisBody(ids_to_check);
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

template <typename T>
void RigidBodyTree<T>::potentialCollisions(
    const KinematicsCache<double>& cache,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx,
    bool use_margins) {
  updateDynamicCollisionElements(cache);
  vector<DrakeCollision::PointPair> potential_collisions;
  potential_collisions =
      collision_model_->potentialCollisionPoints(use_margins);
  size_t num_potential_collisions = potential_collisions.size();

  phi = VectorXd::Zero(num_potential_collisions);
  normal = MatrixXd::Zero(3, num_potential_collisions);
  xA = Matrix3Xd(3, num_potential_collisions);
  xB = Matrix3Xd(3, num_potential_collisions);

  bodyA_idx.clear();
  bodyB_idx.clear();

  for (size_t i = 0; i < num_potential_collisions; ++i) {
    const DrakeCollision::Element* elementA = potential_collisions[i].elementA;
    const DrakeCollision::Element* elementB = potential_collisions[i].elementB;
    xA.col(i) = potential_collisions[i].ptA;
    xB.col(i) = potential_collisions[i].ptB;
    normal.col(i) = potential_collisions[i].normal;
    phi[i] = potential_collisions[i].distance;
    bodyA_idx.push_back(elementA->get_body()->get_body_index());
    bodyB_idx.push_back(elementB->get_body()->get_body_index());
  }
}

template <typename T>
std::vector<DrakeCollision::PointPair>
RigidBodyTree<T>::ComputeMaximumDepthCollisionPoints(
    const KinematicsCache<double>& cache, bool use_margins) {
  updateDynamicCollisionElements(cache);
  vector<DrakeCollision::PointPair> contact_points;
  collision_model_->ComputeMaximumDepthCollisionPoints(use_margins,
                                                       contact_points);
  // For each contact pair, map contact point from world frame to each body's
  // frame.
  for (size_t i = 0; i < contact_points.size(); ++i) {
    auto& pair = contact_points[i];
    if (pair.elementA->CanCollideWith(pair.elementB)) {
      // Get bodies' transforms.
      const int bodyA_id = pair.elementA->get_body()->get_body_index();
      const Isometry3d& TA =
          cache.get_element(bodyA_id).transform_to_world;

      const int bodyB_id = pair.elementB->get_body()->get_body_index();
      const Isometry3d& TB =
          cache.get_element(bodyB_id).transform_to_world;

      // Transform to bodies' frames.
      // Note:
      // Eigen assumes aliasing by default and therefore this operation is safe.
      pair.ptA = TA.inverse() * contact_points[i].ptA;
      pair.ptB = TB.inverse() * contact_points[i].ptB;
    }
  }
  return contact_points;
}

template <typename T>
bool RigidBodyTree<T>::collidingPointsCheckOnly(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model_->collidingPointsCheckOnly(points,
                                                    collision_threshold);
}

template <typename T>
vector<size_t> RigidBodyTree<T>::collidingPoints(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model_->collidingPoints(points, collision_threshold);
}

template <typename T>
bool RigidBodyTree<T>::allCollisions(
    const KinematicsCache<double>& cache,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA_in_world,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xB_in_world, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  bool points_found =
      collision_model_->ComputeMaximumDepthCollisionPoints(use_margins, points);

  xA_in_world = Matrix3Xd::Zero(3, points.size());
  xB_in_world = Matrix3Xd::Zero(3, points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    xA_in_world.col(i) = points[i].ptA;
    xB_in_world.col(i) = points[i].ptB;

    const DrakeCollision::Element* elementA = points[i].elementA;
    bodyA_idx.push_back(elementA->get_body()->get_body_index());
    const DrakeCollision::Element* elementB = points[i].elementB;
    bodyB_idx.push_back(elementB->get_body()->get_body_index());
  }
  return points_found;
}

template <typename T>
template <typename Scalar>
void RigidBodyTree<T>::CheckCacheValidity(
    const KinematicsCache<Scalar>& cache) const {
  if (cache.get_num_cache_elements() != get_num_bodies()) {
    throw std::runtime_error("RigidBodyTree::CheckCacheValidity: Number of "
        "cache elements (" + std::to_string(cache.get_num_cache_elements())
        + ") does not equal the number of bodies in the RigidBodyTree (" +
        std::to_string(get_num_bodies()) + ")");
  }
  for (int i = 0; i < get_num_bodies(); ++i) {
    const RigidBody<T>& body = get_body(i);
    if (body.has_joint()) {
      const DrakeJoint& joint = body.getJoint();
      const KinematicsCacheElement<Scalar>& cache_element =
          cache.get_element(i);
      if (cache_element.get_num_positions() != joint.get_num_positions() ||
          cache_element.get_num_velocities() != joint.get_num_velocities()) {
        throw std::runtime_error("RigidBodyTree::CheckCacheValidity: Cache "
            "element " + std::to_string(i) + " for joint " + joint.get_name() +
            " has incorrect number of joint positions or velocities.\n" +
            "  - num positions: cache has " +
            std::to_string(cache_element.get_num_positions()) +
            ", joint has " + std::to_string(joint.get_num_positions()) + "\n"
            "  - num velocities: cache has " +
            std::to_string(cache.get_num_velocities()) + ", joint has " +
            std::to_string(joint.get_num_velocities()));
      }
    }
  }
}

template <typename T>
KinematicsCache<T>
RigidBodyTree<T>::CreateKinematicsCacheFromBodiesVector(
    const std::vector<std::unique_ptr<RigidBody<T>>>& bodies) {
  std::vector<int> num_joint_positions, num_joint_velocities;
  for (const auto& body : bodies) {
    int np =
        body->has_parent_body() ? body->getJoint().get_num_positions() : 0;
    int nv =
        body->has_parent_body() ? body->getJoint().get_num_velocities() : 0;
    num_joint_positions.push_back(np);
    num_joint_velocities.push_back(nv);
  }
  const int num_positions = std::accumulate(
      num_joint_positions.begin(), num_joint_positions.end(), 0);
  const int num_velocities = std::accumulate(
      num_joint_positions.begin(), num_joint_positions.end(), 0);
  KinematicsCache<T> cache(num_positions, num_velocities,
                           num_joint_positions, num_joint_velocities);
  return cache;
}

template <typename T>
template <typename CacheT>
KinematicsCache<CacheT>
RigidBodyTree<T>::CreateKinematicsCacheWithType() const {
  DRAKE_DEMAND(initialized_ && "This RigidBodyTree was not initialized."
      " RigidBodyTree::compile() must be called first.");
  std::vector<int> num_joint_positions;
  std::vector<int> num_joint_velocities;
  for (const auto& body_unique_ptr : bodies) {
    const RigidBody<T>& body = *body_unique_ptr;
    int num_positions =
        body.has_parent_body() ? body.getJoint().get_num_positions() : 0;
    int num_velocities =
        body.has_parent_body() ? body.getJoint().get_num_velocities() : 0;
    num_joint_positions.push_back(num_positions);
    num_joint_velocities.push_back(num_velocities);
  }
  KinematicsCache<CacheT> cache(get_num_positions(), get_num_velocities(),
      num_joint_positions, num_joint_velocities);
  return cache;
}

template <typename T>
KinematicsCache<T> RigidBodyTree<T>::CreateKinematicsCache() const {
  DRAKE_DEMAND(initialized_ && "This RigidBodyTree was not initialized."
      " RigidBodyTree::compile() must be called first.");
  return CreateKinematicsCacheWithType<T>();
}

template <typename T>
template <typename DerivedQ>
KinematicsCache<typename DerivedQ::Scalar> RigidBodyTree<T>::doKinematics(
    const Eigen::MatrixBase<DerivedQ>& q) const {
  auto cache = CreateKinematicsCacheWithType<typename DerivedQ::Scalar>();
  cache.initialize(q);
  doKinematics(cache);
  return cache;
}

template <typename T>
template <typename DerivedQ, typename DerivedV>
KinematicsCache<typename DerivedQ::Scalar> RigidBodyTree<T>::doKinematics(
    const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedV>& v,
    bool compute_JdotV) const {
  auto cache = CreateKinematicsCacheWithType<typename DerivedQ::Scalar>();
  cache.initialize(q, v);
  doKinematics(cache, compute_JdotV);
  return cache;
}

template <typename T>
template <typename Scalar>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void RigidBodyTree<T>::doKinematics(KinematicsCache<Scalar>& cache,
                                    bool compute_JdotV) const {
  CheckCacheValidity(cache);
  const auto& q = cache.getQ();
  if (!initialized_)
    throw runtime_error("RigidBodyTree::doKinematics: call compile first.");

  // Only compute Jdot times V if V has been provided in the cache.
  compute_JdotV = compute_JdotV && cache.hasV();
  // Required in call to geometricJacobian below.
  cache.setPositionKinematicsCached();

  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    RigidBody<T>& body = *bodies[i];
    KinematicsCacheElement<Scalar>& element = *cache.get_mutable_element(i);

    if (body.has_parent_body()) {
      const KinematicsCacheElement<Scalar>& parent_element =
          *cache.get_mutable_element(body.get_parent()->get_body_index());
      const DrakeJoint& joint = body.getJoint();
      auto q_body = q.middleRows(body.get_position_start_index(),
                                 joint.get_num_positions());

      // transform
      auto T_body_to_parent =
          joint.get_transform_to_parent_body().cast<Scalar>() *
              joint.jointTransform(q_body);
      element.transform_to_world =
          parent_element.transform_to_world * T_body_to_parent;

      // motion subspace in body frame
      Matrix<Scalar, Dynamic, Dynamic>* dSdq = nullptr;
      joint.motionSubspace(q_body, element.motion_subspace_in_body, dSdq);

      // motion subspace in world frame
      element.motion_subspace_in_world = transformSpatialMotion(
          element.transform_to_world, element.motion_subspace_in_body);

      joint.qdot2v(q_body, element.qdot_to_v, nullptr);
      joint.v2qdot(q_body, element.v_to_qdot, nullptr);

      if (cache.hasV()) {
        const auto& v = cache.getV();
        if (joint.get_num_velocities() == 0) {  // for fixed joints
          element.twist_in_world = parent_element.twist_in_world;
          if (compute_JdotV) {
            element.motion_subspace_in_world_dot_times_v =
                parent_element.motion_subspace_in_world_dot_times_v;
          }
        } else {
          // twist
          auto v_body = v.middleRows(body.get_velocity_start_index(),
                                     joint.get_num_velocities());

          TwistVector<Scalar> joint_twist =
              element.motion_subspace_in_world * v_body;
          element.twist_in_world = parent_element.twist_in_world;
          element.twist_in_world.noalias() += joint_twist;

          if (compute_JdotV) {
            // Sdotv
            joint.motionSubspaceDotTimesV(
                q_body, v_body, element.motion_subspace_in_body_dot_times_v,
                nullptr, nullptr);

            // Jdotv
            auto joint_accel =
                crossSpatialMotion(element.twist_in_world, joint_twist);
            joint_accel += transformSpatialMotion(
                element.transform_to_world,
                element.motion_subspace_in_body_dot_times_v);
            element.motion_subspace_in_world_dot_times_v =
                parent_element.motion_subspace_in_world_dot_times_v +
                    joint_accel;
          }
        }
      }
    } else {
      element.transform_to_world.setIdentity();
      // motion subspace in body frame is empty
      // motion subspace in world frame is empty
      // qdot to v is empty
      // v to qdot is empty

      if (cache.hasV()) {
        element.twist_in_world.setZero();
        element.motion_subspace_in_body.setZero();
        element.motion_subspace_in_world.setZero();
        element.qdot_to_v.setZero();
        element.v_to_qdot.setZero();

        if (compute_JdotV) {
          element.motion_subspace_in_body_dot_times_v.setZero();
          element.motion_subspace_in_world_dot_times_v.setZero();
        }
      }
    }
  }

  cache.setJdotVCached(compute_JdotV && cache.hasV());
}

template <typename T>
template <typename Scalar>
void RigidBodyTree<T>::updateCompositeRigidBodyInertias(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false,
                                      "updateCompositeRigidBodyInertias");

  if (!cache.areInertiasCached()) {
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      const RigidBody<T>& body = **it;
      auto element = cache.get_mutable_element(body.get_body_index());
      element->inertia_in_world = transformSpatialInertia(
          element->transform_to_world,
          body.get_spatial_inertia().template cast<Scalar>());
      element->crb_in_world = element->inertia_in_world;
    }

    // N.B. Reverse iteration.
    for (auto it = bodies.rbegin(); it != bodies.rend(); ++it) {
      const RigidBody<T>& body = **it;
      if (body.has_parent_body()) {
        const auto element = cache.get_mutable_element(body.get_body_index());
        auto parent_element = cache.get_mutable_element(
            body.get_parent()->get_body_index());
        parent_element->crb_in_world += element->crb_in_world;
      }
    }
  }
  cache.setInertiasCached();
}

template <typename T>
void RigidBodyTree<T>::ConfirmCompleteTree() const {
  std::set<int> bodies_with_paths;
  bodies_with_paths.insert(0);  // Adds the world node by default.

  for (const auto& body : bodies) {
    TestConnectedToWorld(*body, &bodies_with_paths);
  }
}

template <typename T>
void RigidBodyTree<T>::TestConnectedToWorld(const RigidBody<T>& body,
                                            std::set<int>* connected) const {
  DRAKE_ASSERT(connected->find(0) != connected->end() &&
    "The connected set should always include the world node: 0.");
  int id = body.get_body_index();
  if (connected->find(id) == connected->end()) {
    if (!body.has_joint()) {
      // NOTE: This test is redundant if it is called during
      // RigidBodyTree::compile because two previous operations will catch
      // the missing joint error.  However, for the sake of completeness
      // and because the cost of the redundancy is negligible, the joint
      // test is also included.
      throw runtime_error(
          "ERROR: RigidBodyTree::TestConnectedToWorld(): "
              "Rigid body \"" +
              body.get_name() + "\" in model " + body.get_model_name() +
              " has no joint!");
    }
    const RigidBody<T>* parent = body.get_parent();
    if (parent == nullptr) {
      // We know this is *not* the world node because the world node is in the
      // connected set.
      throw runtime_error(
          "ERROR: RigidBodyTree::TestConnectedToWorld(): "
          "Rigid body \"" +
          body.get_name() + "\" in model " + body.get_model_name() +
          " is not connected to the world!");
    }
    TestConnectedToWorld(*parent, connected);
    // No ancestor of this body threw an exception, so it must have a path.
    // Add it to the set of known connected nodes.
    connected->insert(id);
  }
}

template <typename T>
template <typename Scalar>
TwistMatrix<Scalar> RigidBodyTree<T>::worldMomentumMatrix(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache, const std::set<int>& model_instance_id_set,
    bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "worldMomentumMatrix");
  updateCompositeRigidBodyInertias(cache);

  int nq = num_positions_;
  int nv = num_velocities_;
  int ncols = in_terms_of_qdot ? nq : nv;
  TwistMatrix<Scalar> ret(kTwistSize, ncols);
  ret.setZero();
  int gradient_row_start = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      const auto& element = cache.get_element(body.get_body_index());
      const DrakeJoint& joint = body.getJoint();
      int ncols_joint =
          in_terms_of_qdot ? joint.get_num_positions() :
          joint.get_num_velocities();
      if (is_part_of_model_instances(body, model_instance_id_set)) {
        int start = in_terms_of_qdot ? body.get_position_start_index()
                                     : body.get_velocity_start_index();

        if (in_terms_of_qdot) {
          auto crb =
              (element.crb_in_world * element.motion_subspace_in_world).eval();
          ret.middleCols(start, ncols_joint).noalias() =
              crb * element.qdot_to_v;
        } else {
          ret.middleCols(start, ncols_joint).noalias() =
              element.crb_in_world * element.motion_subspace_in_world;
        }
      }
      gradient_row_start += kTwistSize * ncols_joint;
    }
  }
  return ret;
}

template <typename T>
template <typename Scalar>
TwistVector<Scalar> RigidBodyTree<T>::worldMomentumMatrixDotTimesV(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache, const std::set<int>& model_instance_id_set)
const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "worldMomentumMatrixDotTimesV");
  updateCompositeRigidBodyInertias(cache);

  TwistVector<Scalar> ret;
  ret.setZero();
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      if (is_part_of_model_instances(body, model_instance_id_set)) {
        const auto& element = cache.get_element(body.get_body_index());
        ret.noalias() += element.inertia_in_world *
            element.motion_subspace_in_world_dot_times_v;
        auto inertia_times_twist =
            (element.inertia_in_world * element.twist_in_world).eval();
        ret.noalias() +=
            crossSpatialForce(element.twist_in_world, inertia_times_twist);
      }
    }
  }

  return ret;
}

template <typename T>
template <typename Scalar>
TwistMatrix<Scalar> RigidBodyTree<T>::centroidalMomentumMatrix(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache, const std::set<int>& model_instance_id_set,
    bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  // kinematics cache checks already being done in worldMomentumMatrix.
  auto ret = worldMomentumMatrix(cache, model_instance_id_set,
                                 in_terms_of_qdot);

  // transform from world frame to COM frame
  auto com = centerOfMass(cache, model_instance_id_set);
  auto angular_momentum_matrix = ret.template topRows<kSpaceDimension>();
  auto linear_momentum_matrix = ret.template bottomRows<kSpaceDimension>();
  angular_momentum_matrix += linear_momentum_matrix.colwise().cross(com);

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, kSpaceDimension, Eigen::Isometry>
  //  T(Translation<Scalar, kSpaceDimension>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

template <typename T>
template <typename Scalar>
TwistVector<Scalar> RigidBodyTree<T>::centroidalMomentumMatrixDotTimesV(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache, const std::set<int>& model_instance_id_set)
const {
  CheckCacheValidity(cache);
  // kinematics cache checks already being done in worldMomentumMatrixDotTimesV
  auto ret = worldMomentumMatrixDotTimesV(cache, model_instance_id_set);

  // transform from world frame to COM frame:
  auto com = centerOfMass(cache, model_instance_id_set);
  auto angular_momentum_matrix_dot_times_v =
      ret.template topRows<kSpaceDimension>();
  auto linear_momentum_matrix_dot_times_v =
      ret.template bottomRows<kSpaceDimension>();
  angular_momentum_matrix_dot_times_v +=
      linear_momentum_matrix_dot_times_v.cross(com);

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, kSpaceDimension, Eigen::Isometry>
  //  T(Translation<Scalar, kSpaceDimension>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

template <typename T>
bool RigidBodyTree<T>::is_part_of_model_instances(
    const RigidBody<T>& body,
    const std::set<int>& model_instance_id_set) const {
  for (std::set<int>::const_iterator it = model_instance_id_set.begin();
       it != model_instance_id_set.end(); ++it) {
    if (*it < -1) {
      return true;
    }
  }

  return model_instance_id_set.find(body.get_model_instance_id()) !=
         model_instance_id_set.end();
}

template <typename T>
double RigidBodyTree<T>::getMass(
    const std::set<int>& model_instance_id_set) const {
  double total_mass = 0.0;
  for (const auto& body : bodies) {
    if (is_part_of_model_instances(*body.get(), model_instance_id_set)) {
      total_mass += body->get_mass();
    }
  }
  return total_mass;
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, kSpaceDimension, 1> RigidBodyTree<T>::centerOfMass(
    const KinematicsCache<Scalar>& cache,
    const std::set<int>& model_instance_id_set) const {
  cache.checkCachedKinematicsSettings(false, false, "centerOfMass");

  Eigen::Matrix<Scalar, kSpaceDimension, 1> com;
  com.setZero();
  double m = 0.0;

  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    RigidBody<T>& body = *bodies[i];
    if (is_part_of_model_instances(body, model_instance_id_set)) {
      if (body.get_mass() > 0) {
        com.noalias() +=
            body.get_mass() *
                transformPoints(
                        cache,
                        body.get_center_of_mass().template cast<Scalar>(),
                        i, 0);
      }
      m += body.get_mass();
    }
  }
  if (m > 0.0) com /= m;

  return com;
}

template <typename T>
drake::Matrix6<T> RigidBodyTree<T>::LumpedSpatialInertiaInWorldFrame(
      const KinematicsCache<T>& cache,
      const std::set<int>& model_instance_id_set) const {
  drake::Matrix6<T> I_W = drake::Matrix6<T>::Zero();
  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    const RigidBody<T>& body = *bodies[i];
    if (is_part_of_model_instances(body, model_instance_id_set)) {
      const Isometry3<T> X_WB = CalcBodyPoseInWorldFrame(cache, body);
      I_W += transformSpatialInertia(
          X_WB, body.get_spatial_inertia().template cast<T>());
    }
  }
  return I_W;
}

template <typename T>
drake::Matrix6<T> RigidBodyTree<T>::LumpedSpatialInertiaInWorldFrame(
      const KinematicsCache<T>& cache,
      const std::vector<const RigidBody<T>*>& bodies_of_interest) const {
  drake::Matrix6<T> I_W = drake::Matrix6<T>::Zero();
  for (const RigidBody<T>* body : bodies_of_interest) {
    const Isometry3<T> X_WB = CalcBodyPoseInWorldFrame(cache, *body);
    I_W += transformSpatialInertia(
        X_WB, body->get_spatial_inertia().template cast<T>());
  }
  return I_W;
}

template <typename T>
VectorX<T> RigidBodyTree<T>::transformVelocityToQDot(
    const KinematicsCache<T>& cache,
    const VectorX<T>& v) {
  VectorX<T> qdot(cache.get_num_positions());
  int qdot_start = 0;
  int v_start = 0;
  for (int body_id = 0; body_id < cache.get_num_cache_elements(); ++body_id) {
    const auto& element = cache.get_element(body_id);
    qdot.segment(qdot_start, element.get_num_positions()).noalias() =
        element.v_to_qdot * v.segment(v_start, element.get_num_velocities());
    qdot_start += element.get_num_positions();
    v_start += element.get_num_velocities();
  }
  return qdot;
}

template <typename T>
VectorX<T> RigidBodyTree<T>::transformQDotToVelocity(
    const KinematicsCache<T>& cache,
    const VectorX<T>& qdot) {
  VectorX<T> v(cache.get_num_velocities());
  int qdot_start = 0;
  int v_start = 0;
  for (int body_id = 0; body_id < cache.get_num_cache_elements(); ++body_id) {
    const auto& element = cache.get_element(body_id);
    v.segment(v_start, element.get_num_velocities()).noalias() =
        element.qdot_to_v *
            qdot.segment(qdot_start, element.get_num_positions());
    qdot_start += element.get_num_positions();
    v_start += element.get_num_velocities();
  }
  return v;
}

template <typename T>
template <typename Derived>
MatrixX<typename Derived::Scalar>
RigidBodyTree<T>::transformVelocityMappingToQDotMapping(
    const KinematicsCache<typename Derived::Scalar>& cache,
    const Eigen::MatrixBase<Derived>& Av) {
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
      Ap(Av.rows(), cache.get_num_positions());
  DRAKE_DEMAND(Av.cols() == cache.get_num_velocities());
  int Ap_col_start = 0;
  int Av_col_start = 0;
  for (int body_id = 0; body_id < cache.get_num_cache_elements(); ++body_id) {
    const auto& element = cache.get_element(body_id);
    Ap.middleCols(Ap_col_start, element.get_num_positions()).noalias() =
            Av.middleCols(Av_col_start, element.get_num_velocities()) *
                element.qdot_to_v;
    Ap_col_start += element.get_num_positions();
    Av_col_start += element.get_num_velocities();
  }
  return Ap;
}

template <typename T>
template <typename Derived>
MatrixX<typename Derived::Scalar>
RigidBodyTree<T>::transformQDotMappingToVelocityMapping(
    const KinematicsCache<typename Derived::Scalar>& cache,
    const Eigen::MatrixBase<Derived>& Ap) {
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
      Av(Ap.rows(), cache.get_num_velocities());
  DRAKE_DEMAND(Ap.cols() == cache.get_num_positions());
  int Av_col_start = 0;
  int Ap_col_start = 0;
  for (int body_id = 0; body_id < cache.get_num_cache_elements(); ++body_id) {
    const auto& element = cache.get_element(body_id);
    Av.middleCols(Av_col_start, element.get_num_velocities()).noalias() =
        Ap.middleCols(Ap_col_start, element.get_num_positions()) *
            element.v_to_qdot;
    Av_col_start += element.get_num_velocities();
    Ap_col_start += element.get_num_positions();
  }
  return Av;
}

template <typename T>
template <typename Scalar>
MatrixX<Scalar> RigidBodyTree<T>::GetVelocityToQDotMapping(
        const KinematicsCache<Scalar>& cache) {
  return transformQDotMappingToVelocityMapping(
      cache,
      MatrixX<Scalar>::Identity(cache.get_num_positions(),
                                cache.get_num_positions()));
}

template <typename T>
template <typename Scalar>
MatrixX<Scalar> RigidBodyTree<T>::GetQDotToVelocityMapping(
        const KinematicsCache<Scalar>& cache) {
  return transformVelocityMappingToQDotMapping(
      cache,
      MatrixX<Scalar>::Identity(cache.get_num_velocities(),
                                cache.get_num_velocities()));
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, kSpaceDimension, Eigen::Dynamic>
RigidBodyTree<T>::centerOfMassJacobian(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const std::set<int>& model_instance_id_set,
    bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "centerOfMassJacobian");
  auto A = worldMomentumMatrix(cache, model_instance_id_set, in_terms_of_qdot);
  double total_mass = getMass(model_instance_id_set);
  return A.template bottomRows<kSpaceDimension>() / total_mass;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, kSpaceDimension, 1>
RigidBodyTree<T>::centerOfMassJacobianDotTimesV(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const std::set<int>& model_instance_id_set) const {
  CheckCacheValidity(cache);
  // kinematics cache checks are already being done in
  // centroidalMomentumMatrixDotTimesV
  auto cmm_dot_times_v =
      centroidalMomentumMatrixDotTimesV(cache, model_instance_id_set);
  double total_mass = getMass(model_instance_id_set);
  return cmm_dot_times_v.template bottomRows<kSpaceDimension>() / total_mass;
}

template <typename T>
template <typename DerivedNormal, typename DerivedPoint>
std::pair<Eigen::Vector3d, double> RigidBodyTree<T>::resolveCenterOfPressure(
    const KinematicsCache<double>& cache,
    const std::vector<ForceTorqueMeasurement>& force_torque_measurements,
    const Eigen::MatrixBase<DerivedNormal>& normal,
    const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane) const {
  CheckCacheValidity(cache);
  // kinematics cache checks are already being done in relativeTransform
  typedef typename DerivedNormal::Scalar Scalar;
  TwistVector<Scalar> total_wrench = TwistVector<Scalar>::Zero();
  for (auto it = force_torque_measurements.begin();
       it != force_torque_measurements.end(); ++it) {
    auto transform_to_world = relativeTransform(cache, 0, it->frame_idx);
    total_wrench += transformSpatialForce(transform_to_world, it->wrench);
  }
  return ::resolveCenterOfPressure(total_wrench.template head<3>(),
                                   total_wrench.template tail<3>(), normal,
                                   point_on_contact_plane);
}

template <typename T>
int RigidBodyTree<T>::getNumContacts(const set<int>& body_idx) const {
  size_t num_contacts = 0, nb = body_idx.size(), bi;
  if (nb == 0) nb = bodies.size();
  set<int>::iterator iter = body_idx.begin();
  for (size_t i = 0; i < nb; ++i) {
    if (body_idx.size() == 0)
      bi = i;
    else
      bi = *iter++;
    num_contacts += bodies[bi]->get_contact_points().cols();
  }
  return static_cast<int>(num_contacts);
}

/* [body_ind, Tframe] = parseBodyOrFrameID(body_or_frame_id) */
template <typename T>
template <typename Scalar>
int RigidBodyTree<T>::parseBodyOrFrameID(
    const int body_or_frame_id,
    Eigen::Transform<Scalar, 3, Isometry>* Tframe) const {
  int body_ind = 0;
  if (body_or_frame_id == -1) {
    cerr << "parseBodyOrFrameID got a -1, which should have been reserved for "
        "COM.  Shouldn't have gotten here."
         << endl;
  } else if (body_or_frame_id < 0) {
    int frame_ind = -body_or_frame_id - 2;
    // check that this is in range
    if (frame_ind >= static_cast<int>(frames.size())) {
      std::ostringstream stream;
      stream << "Got a frame ind greater than available!\n";
      throw std::runtime_error(stream.str());
    }
    body_ind = frames[frame_ind]->get_rigid_body().get_body_index();

    if (Tframe) {
      (*Tframe) =
          frames[frame_ind]->get_transform_to_body().template cast<Scalar>();
    }
  } else {
    body_ind = body_or_frame_id;
    if (Tframe) Tframe->setIdentity();
  }
  return body_ind;
}

template <typename T>
int RigidBodyTree<T>::parseBodyOrFrameID(const int body_or_frame_id) const {
  return parseBodyOrFrameID<double>(body_or_frame_id, nullptr);
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindAncestorBodies(
    int body_index) const {
  // Verifies that body_index is valid. Aborts if it is invalid.
  DRAKE_DEMAND(body_index >= 0 &&
      body_index < static_cast<int>(bodies.size()));

  std::vector<int> ancestor_body_list;
  const RigidBody<T>* current_body = bodies[body_index].get();
  while (current_body->has_parent_body()) {
    ancestor_body_list.push_back(current_body->get_parent()->get_body_index());
    current_body = current_body->get_parent();
  }
  return ancestor_body_list;
}

// TODO(liang.fok) Remove this deprecated method prior to Release 1.0.
template <typename T>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void RigidBodyTree<T>::findAncestorBodies(std::vector<int>& ancestor_bodies,
                                          int body_idx) const {
  ancestor_bodies = FindAncestorBodies(body_idx);
}

template <typename T>
KinematicPath RigidBodyTree<T>::findKinematicPath(
    int start_body_or_frame_idx, int end_body_or_frame_idx) const {
  // find all ancestors of start_body and end_body
  int start_body = parseBodyOrFrameID(start_body_or_frame_idx);

  std::vector<int> start_body_ancestors = FindAncestorBodies(start_body);
  start_body_ancestors.insert(start_body_ancestors.begin(), start_body);

  int end_body = parseBodyOrFrameID(end_body_or_frame_idx);
  std::vector<int> end_body_ancestors = FindAncestorBodies(end_body);
  end_body_ancestors.insert(end_body_ancestors.begin(), end_body);

  // find least common ancestor
  size_t common_size =
      std::min(start_body_ancestors.size(), end_body_ancestors.size());
  bool least_common_ancestor_found = false;
  std::vector<int>::iterator start_body_lca_it =
      start_body_ancestors.end() - common_size;
  std::vector<int>::iterator end_body_lca_it =
      end_body_ancestors.end() - common_size;

  for (size_t i = 0; i < common_size; ++i) {
    if (*start_body_lca_it == *end_body_lca_it) {
      least_common_ancestor_found = true;
      break;
    }
    start_body_lca_it++;
    end_body_lca_it++;
  }

  if (!least_common_ancestor_found) {
    std::ostringstream stream;
    stream << "There is no path between " << bodies[start_body]->get_name()
           << " and " << bodies[end_body]->get_name() << ".";
    throw std::runtime_error(stream.str());
  }
  int least_common_ancestor = *start_body_lca_it;

  // compute path
  KinematicPath path;

  std::vector<int>::iterator it = start_body_ancestors.begin();
  for (; it != start_body_lca_it; ++it) {
    path.joint_path.push_back(*it);
    path.joint_direction_signs.push_back(-1);
    path.body_path.push_back(*it);
  }

  path.body_path.push_back(least_common_ancestor);

  std::vector<int>::reverse_iterator reverse_it(end_body_lca_it);
  for (; reverse_it != end_body_ancestors.rend(); ++reverse_it) {
    path.joint_path.push_back(*reverse_it);
    path.joint_direction_signs.push_back(1);
    path.body_path.push_back(*reverse_it);
  }
  return path;
}

template <typename T>
template <typename Scalar>
TwistMatrix<Scalar> RigidBodyTree<T>::geometricJacobian(
    const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
    int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind,
    bool in_terms_of_qdot, std::vector<int>* v_or_qdot_indices) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "geometricJacobian");

  KinematicPath kinematic_path =
      findKinematicPath(base_body_or_frame_ind, end_effector_body_or_frame_ind);

  int cols = 0;
  int body_index;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); ++i) {
    body_index = kinematic_path.joint_path[i];
    const RigidBody<T>& body = *bodies[body_index];
    const DrakeJoint& joint = body.getJoint();
    cols +=
        in_terms_of_qdot ? joint.get_num_positions() :
        joint.get_num_velocities();
  }

  TwistMatrix<Scalar> J(kTwistSize, cols);

  if (v_or_qdot_indices != nullptr) {
    v_or_qdot_indices->clear();
    v_or_qdot_indices->reserve(cols);
  }

  int col_start = 0;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); ++i) {
    body_index = kinematic_path.joint_path[i];
    RigidBody<T>& body = *bodies[body_index];
    const auto& element = cache.get_element(body.get_body_index());
    const DrakeJoint& joint = body.getJoint();
    int ncols_block =
        in_terms_of_qdot ? joint.get_num_positions() :
        joint.get_num_velocities();
    int sign = kinematic_path.joint_direction_signs[i];
    auto J_block = J.template block<kTwistSize, Dynamic>(
        0, col_start, kTwistSize, ncols_block);
    if (in_terms_of_qdot) {
      J_block.noalias() =
          sign * element.motion_subspace_in_world * element.qdot_to_v;
    } else {
      J_block.noalias() = sign * element.motion_subspace_in_world;
    }

    if (v_or_qdot_indices != nullptr) {
      int cols_block_start =
          in_terms_of_qdot ? body.get_position_start_index() :
          body.get_velocity_start_index();
      for (int j = 0; j < ncols_block; ++j) {
        v_or_qdot_indices->push_back(cols_block_start + j);
      }
    }
    col_start += ncols_block;
  }

  if (expressed_in_body_or_frame_ind != 0) {
    auto T_world_to_frame =
        relativeTransform(cache, expressed_in_body_or_frame_ind, 0);
    J = transformSpatialMotion(T_world_to_frame, J);
  }

  return J;
}

template <typename T>
template <typename Scalar>
TwistVector<Scalar> RigidBodyTree<T>::geometricJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
    int end_effector_body_or_frame_ind,
    int expressed_in_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true, "geometricJacobianDotTimesV");

  TwistVector<Scalar> ret(kTwistSize, 1);

  int base_body_ind = parseBodyOrFrameID(base_body_or_frame_ind);
  int end_effector_body_ind =
      parseBodyOrFrameID(end_effector_body_or_frame_ind);

  const auto& base_element = cache.get_element(base_body_ind);
  const auto& end_effector_element =
      cache.get_element(end_effector_body_ind);

  ret = end_effector_element.motion_subspace_in_world_dot_times_v -
      base_element.motion_subspace_in_world_dot_times_v;

  int world_ind = 0;
  return transformSpatialAcceleration(cache, ret, base_body_ind,
                                      end_effector_body_ind, world_ind,
                                      expressed_in_body_or_frame_ind);
}

template <typename T>
template <typename Scalar>
TwistVector<Scalar> RigidBodyTree<T>::relativeTwist(
    const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
    int body_or_frame_ind, int expressed_in_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, false, "relativeTwist");

  int base_ind = parseBodyOrFrameID(base_or_frame_ind);
  int body_ind = parseBodyOrFrameID(body_or_frame_ind);
  int world = 0;
  auto Tr = relativeTransform(cache, expressed_in_body_or_frame_ind, world);

  const auto& base_element = cache.get_element(base_ind);
  const auto& body_element = cache.get_element(body_ind);
  TwistVector<Scalar> relative_twist_in_world =
      body_element.twist_in_world - base_element.twist_in_world;
  return transformSpatialMotion(Tr, relative_twist_in_world);
}

template <typename T>
template <typename Scalar>
TwistVector<Scalar> RigidBodyTree<T>::transformSpatialAcceleration(
    const KinematicsCache<Scalar>& cache,
    const TwistVector<Scalar>& spatial_acceleration, int base_ind, int body_ind,
    int old_expressed_in_body_or_frame_ind,
    int new_expressed_in_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "transformSpatialAcceleration");

  if (old_expressed_in_body_or_frame_ind ==
      new_expressed_in_body_or_frame_ind) {
    return spatial_acceleration;
  }

  auto twist_of_body_wrt_base = relativeTwist(
      cache, base_ind, body_ind, old_expressed_in_body_or_frame_ind);
  auto twist_of_old_wrt_new = relativeTwist(
      cache, new_expressed_in_body_or_frame_ind,
      old_expressed_in_body_or_frame_ind, old_expressed_in_body_or_frame_ind);
  auto T_old_to_new =
      relativeTransform(cache, new_expressed_in_body_or_frame_ind,
                        old_expressed_in_body_or_frame_ind);

  TwistVector<Scalar> spatial_accel_temp =
      crossSpatialMotion(twist_of_old_wrt_new, twist_of_body_wrt_base);
  spatial_accel_temp += spatial_acceleration;
  return transformSpatialMotion(T_old_to_new, spatial_accel_temp);
}

template <typename T>
template <typename Scalar>
Transform<Scalar, 3, Isometry> RigidBodyTree<T>::relativeTransform(
    const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
    int body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "relativeTransform");

  Transform<Scalar, 3, Isometry> Tbase_frame;
  int base_ind = parseBodyOrFrameID(base_or_frame_ind, &Tbase_frame);
  Transform<Scalar, 3, Isometry> Tbody_frame;
  int body_ind = parseBodyOrFrameID(body_or_frame_ind, &Tbody_frame);

  const auto& body_element = cache.get_element(body_ind);
  const auto& base_element = cache.get_element(base_ind);

  Transform<Scalar, 3, Isometry> Tbaseframe_to_world =
      base_element.transform_to_world * Tbase_frame;
  Transform<Scalar, 3, Isometry> Tworld_to_baseframe =
      Tbaseframe_to_world.inverse();
  Transform<Scalar, 3, Isometry> Tbodyframe_to_world =
      body_element.transform_to_world * Tbody_frame;
  return Tworld_to_baseframe * Tbodyframe_to_world;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyTree<T>::massMatrix(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "massMatrix");

  int nv = num_velocities_;
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(nv, nv);
  ret.setZero();

  updateCompositeRigidBodyInertias(cache);

  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    RigidBody<T>& body_i = *bodies[i];
    if (body_i.has_parent_body()) {
      const auto& element_i = cache.get_element(i);
      int v_start_i = body_i.get_velocity_start_index();
      int nv_i = body_i.getJoint().get_num_velocities();
      auto F =
          (element_i.crb_in_world * element_i.motion_subspace_in_world).eval();

      // Hii
      ret.block(v_start_i, v_start_i, nv_i, nv_i).noalias() =
          (element_i.motion_subspace_in_world.transpose() * F).eval();

      // Hij
      const RigidBody<T>* body_j(body_i.get_parent());
      while (body_j->has_parent_body()) {
        const auto& element_j = cache.get_element(body_j->get_body_index());
        int v_start_j = body_j->get_velocity_start_index();
        int nv_j = body_j->getJoint().get_num_velocities();
        auto Hji = (element_j.motion_subspace_in_world.transpose() * F).eval();
        ret.block(v_start_j, v_start_i, nv_j, nv_i) = Hji;
        ret.block(v_start_i, v_start_j, nv_i, nv_j) = Hji.transpose();

        body_j = body_j->get_parent();
      }
    }
  }

  return ret;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree<T>::dynamicsBiasTerm(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const drake::eigen_aligned_std_unordered_map<
        RigidBody<T> const*, WrenchVector<Scalar>>& external_wrenches,
    bool include_velocity_terms) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, 1> vd(num_velocities_, 1);
  vd.setZero();
  return inverseDynamics(cache, external_wrenches, vd, include_velocity_terms);
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree<T>::inverseDynamics(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache,
    const drake::eigen_aligned_std_unordered_map<
        RigidBody<T> const*, WrenchVector<Scalar>>& external_wrenches,
    const Matrix<Scalar, Eigen::Dynamic, 1>& vd,
    bool include_velocity_terms) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(
      include_velocity_terms, include_velocity_terms, "inverseDynamics");

  // Let w denote the world rigid body. Note that all quantities used in this
  // algorithm are expressed in world frame. For each body i, compute its
  // rigid body inertia in world frame, denoted I^w_i.
  // Composite rigid body inertias are computed as well, but note that we don't
  // actually need composite rigid body inertias for this algorithm.
  updateCompositeRigidBodyInertias(cache);

  // TODO(#3114) pass this in as an argument
  const bool include_acceleration_terms = true;

  // Compute spatial accelerations and net wrenches that should be exerted to
  // achieve those accelerations.
  // TODO(tkoolen) should preallocate:
  Matrix6X<Scalar> body_accelerations(kTwistSize, bodies.size());
  Matrix6X<Scalar> net_wrenches(kTwistSize, bodies.size());
  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    const RigidBody<T>& body = *bodies[i];
    if (body.has_parent_body()) {
      const RigidBody<T>& parent_body = *(body.get_parent());
      const auto& cache_element = cache.get_element(i);

      // Denote the spatial acceleration twist derivative) of body i with
      // respect to the world as Tdot^w_i. Let lambda(i) denote the parent of
      // body i. The spatial acceleration of body i is the sum of the spatial
      // acceleration of lambda(i) and the relative acceleration of i with
      // respect to lambda(i):
      //
      //    Tdot^w_i = Tdot^w_lambda(i) + Tdot^lambda(i)_i
      //
      // The relative acceleration Tdot^lambda(i)_i is the derivative of the
      // twist T^lambda(i)_i of i with respect to lambda(i). The twist
      // T^lambda(i)_i can be written as
      //
      //    T^lambda(i)_i = J^lambda(i)_i v_i
      //
      // where J^lambda(i)_i is the motion subspace, expressed in world
      // frame, of the joint between i and lambda(i) (a geometric Jacobian),
      // and v_i is the joint velocity vector for this joint. Differentiating
      // results in
      //
      //    Tdot^lambda(i)_i = Jdot^lambda(i)_i v_i + J^lambda(i)_i vdot_i
      //
      // where the term Jdot^lambda(i)_i v_i will be referred to as the bias
      // acceleration of body i w.r.t. lambda(i), and is seen to be the
      // spatial acceleration of body i w.r.t lambda(i) when there is no
      // joint acceleration (vdot_i = 0).
      // KinematicsCache stores the bias acceleration of each body with
      // respect to the world: Jdot^w_i v^w_i, where v^w_i denotes the vector of
      // joint velocities on the path between body i and the world. The bias
      // acceleration with respect to lambda(i) can be obtained simply as
      //
      //    Jdot^lambda(i)_i v_i = Jdot^w_i v^w_i - Jdot^w_lambda(i)
      //        v^w_lambda(i)
      //
      // In the code, we refer to:
      // * Tdot^w_lambda(i) as parent_acceleration,
      // * Jdot^w_i v^w_i as motion_subspace_in_world_dot_times_v,
      // * vdot_i as vd_joint.
      auto body_acceleration = body_accelerations.col(i);

      // Initialize body acceleration to acceleration of parent body.
      auto parent_acceleration =
          body_accelerations.col(parent_body.get_body_index());
      body_acceleration = parent_acceleration;

      // Add bias acceleration relative to parent body.
      if (include_velocity_terms) {
        const auto& parent_cache_element =
            cache.get_element(parent_body.get_body_index());
        body_acceleration += cache_element.motion_subspace_in_world_dot_times_v;
        body_acceleration -=
            parent_cache_element.motion_subspace_in_world_dot_times_v;
      }

      // Add component due to joint acceleration.
      if (include_acceleration_terms) {
        const DrakeJoint& joint = body.getJoint();
        auto vd_joint = vd.middleRows(body.get_velocity_start_index(),
                                      joint.get_num_velocities());
        body_acceleration.noalias() +=
            cache_element.motion_subspace_in_world * vd_joint;
      }

      // Now that the spatial acceleration of the body with respect to the
      // world is known, we compute the net wrench on the body required to
      // effect the acceleration.
      // Denote the (spatial) momentum of body i as h_i. h_i can be computed as
      //
      //    h_i = I^w_i T^w_i
      //
      // The Newton-Euler equations state that
      //
      //    hdot_i = W_i
      //
      // where W_i is the net wrench exerted upon body i. Differentiating, we
      // find
      //
      //   W_i = hdot_i = Idot^w_i T^w_i + I^w_i Tdot^w_i
      //
      // The term Idot^w_i T^w_i is
      //
      //   Idot^w_i = cross(T^w_i, I^w_i T^w_i)
      //
      // where cross denotes a spatial force cross product. This can be
      // derived by differentiating the transformation rule for spatial
      // inertias.

      auto net_wrench = net_wrenches.col(i);
      const auto& body_inertia = cache_element.inertia_in_world;
      net_wrench.noalias() = body_inertia * body_acceleration;
      if (include_velocity_terms) {
        const auto& body_twist = cache_element.twist_in_world;
        auto inertia_times_twist = (body_inertia * body_twist).eval();
        net_wrench += crossSpatialForce(body_twist, inertia_times_twist);
      }

      // Subtract off any external wrench that may be acting on body (and
      // hence contributing to the achievement of the net wrench).
      auto external_wrench_it = external_wrenches.find(&body);
      if (external_wrench_it != external_wrenches.end()) {
        const auto& external_wrench = external_wrench_it->second;
        const auto& body_to_world = cache_element.transform_to_world;
        net_wrench -= transformSpatialForce(body_to_world, external_wrench);
      }
    } else {
      body_accelerations.col(i) = -a_grav.cast<Scalar>();
      net_wrenches.col(i).setZero();
    }
  }

  // Do a backwards pass to compute joint wrenches from net wrenches (update in
  // place), and project the joint wrenches onto the joint's motion subspace to
  // find the joint torque.
  auto& joint_wrenches = net_wrenches;
  // the following will eliminate the need for another explicit instantiation:
  const auto& joint_wrenches_const = net_wrenches;

  VectorX<Scalar> torques(num_velocities_, 1);
  for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; --i) {
    RigidBody<T>& body = *bodies[i];
    if (body.has_parent_body()) {
      const auto& cache_element = cache.get_element(i);
      const auto& joint = body.getJoint();
      auto joint_wrench = joint_wrenches_const.col(i);

      // Compute joint torques associated with the joint wrench. Joint torque
      // tau_i can be computed from the wrench across the joint between i and
      // lambda(i) as
      //
      //    J^lambda(i)_i ' * W_i
      //
      // This can be derived from a power balance.
      const auto& motion_subspace = cache_element.motion_subspace_in_world;
      auto joint_torques = torques.middleRows(body.get_velocity_start_index(),
                                              joint.get_num_velocities());
      joint_torques.noalias() = motion_subspace.transpose() * joint_wrench;

      // The joint wrench W exerted upon body i through the joint between i
      // and lambda(i) acts in the opposite direction on lambda(i) (Newton's
      // third law). This wrench, -W, should be subtracted from the net
      // wrench exerted upon lambda(i) (similar to external wrenches), so W
      // should be *added* to the net wrench.
      const RigidBody<T>& parent_body = *(body.get_parent());
      auto parent_joint_wrench =
          joint_wrenches.col(parent_body.get_body_index());
      parent_joint_wrench += joint_wrench;
    }
  }

  if (include_velocity_terms) {
    // TODO(#1476) frictionTorques uses abs. To support e.g. TrigPoly, there
    // should be a version of inverseDynamics that doesn't call frictionTorques.
    torques += frictionTorques(cache.getV());
  }

  return torques;
}

template <typename T>
template <typename DerivedV>
Matrix<typename DerivedV::Scalar, Dynamic, 1> RigidBodyTree<T>::frictionTorques(
    Eigen::MatrixBase<DerivedV> const& v) const {
  typedef typename DerivedV::Scalar Scalar;
  Matrix<Scalar, Dynamic, 1> ret(num_velocities_, 1);

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      const DrakeJoint& joint = body.getJoint();
      int nv_joint = joint.get_num_velocities();
      int v_start_joint = body.get_velocity_start_index();
      auto v_body = v.middleRows(v_start_joint, nv_joint);
      ret.middleRows(v_start_joint, nv_joint) = joint.frictionTorque(v_body);
    }
  }

  return ret;
}

template <typename T>
template <typename Scalar, typename DerivedPoints>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::transformPointsJacobian(
    const KinematicsCache<Scalar>& cache,
    const Eigen::MatrixBase<DerivedPoints>& points, int from_body_or_frame_ind,
    int to_body_or_frame_ind, bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  int cols = in_terms_of_qdot ? num_positions_ : num_velocities_;
  int npoints = static_cast<int>(points.cols());

  auto points_base = transformPoints(cache, points, from_body_or_frame_ind,
                                     to_body_or_frame_ind);

  int body_ind = parseBodyOrFrameID(from_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(to_body_or_frame_ind);
  std::vector<int> v_or_q_indices;
  auto J_geometric =
      geometricJacobian(cache, base_ind, body_ind, to_body_or_frame_ind,
                        in_terms_of_qdot, &v_or_q_indices);

  auto Jomega = J_geometric.template topRows<kSpaceDimension>();
  auto Jv = J_geometric.template bottomRows<kSpaceDimension>();

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J(
      points_base.size(), cols);  // TODO(tkoolen): size at compile time
  J.setZero();

  int row_start = 0;
  for (int i = 0; i < npoints; ++i) {
    // translation part
    int col = 0;
    for (std::vector<int>::iterator it = v_or_q_indices.begin();
         it != v_or_q_indices.end(); ++it) {
      J.template block<kSpaceDimension, 1>(row_start, *it) = Jv.col(col);
      J.template block<kSpaceDimension, 1>(row_start, *it).noalias() +=
          Jomega.col(col).cross(points_base.col(i));
      col++;
    }
    row_start += kSpaceDimension;
  }

  return J;
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, kQuaternionSize, Eigen::Dynamic>
RigidBodyTree<T>::relativeQuaternionJacobian(
    const KinematicsCache<Scalar>& cache,
    int from_body_or_frame_ind, int to_body_or_frame_ind,
    bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  int body_ind = parseBodyOrFrameID(from_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(to_body_or_frame_ind);
  KinematicPath kinematic_path = findKinematicPath(base_ind, body_ind);
  auto J_geometric = geometricJacobian(cache, base_ind, body_ind,
                                       to_body_or_frame_ind, in_terms_of_qdot);
  auto Jomega = J_geometric.template topRows<kSpaceDimension>();
  auto quat =
      relativeQuaternion(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, kQuaternionSize, kSpaceDimension> Phi;
  angularvel2quatdotMatrix(
      quat, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr));
  return compactToFull((Phi * Jomega).eval(), kinematic_path.joint_path,
                       in_terms_of_qdot);
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, kRpySize, Eigen::Dynamic>
RigidBodyTree<T>::relativeRollPitchYawJacobian(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind, bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  int body_ind = parseBodyOrFrameID(from_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(to_body_or_frame_ind);
  KinematicPath kinematic_path = findKinematicPath(base_ind, body_ind);
  auto J_geometric = geometricJacobian(cache, base_ind, body_ind,
                                       to_body_or_frame_ind, in_terms_of_qdot);
  auto Jomega = J_geometric.template topRows<kSpaceDimension>();
  auto rpy =
      relativeRollPitchYaw(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, kRpySize, kSpaceDimension> Phi;
  angularvel2rpydotMatrix(
      rpy, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr),
      static_cast<typename Gradient<decltype(Phi), Dynamic, 2>::type*>(
          nullptr));
  return compactToFull((Phi * Jomega).eval(), kinematic_path.joint_path,
                       in_terms_of_qdot);
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::forwardKinPositionGradient(
    const KinematicsCache<Scalar>& cache,
    int npoints, int from_body_or_frame_ind, int to_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false,
                                      "forwardKinPositionGradient");

  auto Tinv =
      relativeTransform(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(kSpaceDimension * npoints,
                                                     kSpaceDimension * npoints);
  ret.setZero();
  for (int i = 0; i < npoints; ++i) {
    ret.template block<kSpaceDimension, kSpaceDimension>(
        kSpaceDimension * i, kSpaceDimension * i) = Tinv.linear();
  }
  return ret;
}

template <typename T>
template <typename Scalar, typename DerivedPoints>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::transformPointsJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache,
    const Eigen::MatrixBase<DerivedPoints>& points, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "transformPointsJacobianDotTimesV");

  auto r = transformPoints(cache, points, from_body_or_frame_ind,
                           to_body_or_frame_ind);
  int expressed_in = to_body_or_frame_ind;
  const auto twist =
      relativeTwist(cache, to_body_or_frame_ind, from_body_or_frame_ind,
                    to_body_or_frame_ind);
  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(
      cache, to_body_or_frame_ind, from_body_or_frame_ind, expressed_in);

  auto omega_twist = twist.template topRows<kSpaceDimension>();
  auto v_twist = twist.template bottomRows<kSpaceDimension>();

  auto rdots = (-r.colwise().cross(omega_twist)).eval();
  rdots.colwise() += v_twist;
  auto Jposdot_times_v_mat = (-rdots.colwise().cross(omega_twist)).eval();
  Jposdot_times_v_mat -=
      (r.colwise().cross(
          J_geometric_dot_times_v.template topRows<kSpaceDimension>()))
          .eval();
  Jposdot_times_v_mat.colwise() +=
      J_geometric_dot_times_v.template bottomRows<kSpaceDimension>();

  return Map<Matrix<Scalar, Dynamic, 1>>(Jposdot_times_v_mat.data(), r.size(),
                                         1);
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::relativeQuaternionJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "relativeQuaternionJacobianDotTimesV");

  auto quat =
      relativeQuaternion(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, kQuaternionSize, kSpaceDimension> Phi;
  angularvel2quatdotMatrix(
      quat, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr));

  int expressed_in = to_body_or_frame_ind;
  const auto twist = relativeTwist(cache, to_body_or_frame_ind,
                                   from_body_or_frame_ind, expressed_in);
  auto omega_twist = twist.template topRows<kSpaceDimension>();
  auto quatdot = (Phi * omega_twist).eval();

  using ADScalar = AutoDiffScalar<Matrix<Scalar, Dynamic,
                                         1>>;  // would prefer to use 1 instead
  // of Dynamic, but this causes
  // issues related to
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1006 on MSVC
  // 32 bit
  auto quat_autodiff = quat.template cast<ADScalar>().eval();
  gradientMatrixToAutoDiff(quatdot, quat_autodiff);
  Matrix<ADScalar, kQuaternionSize, kSpaceDimension> Phi_autodiff;
  angularvel2quatdotMatrix(
      quat_autodiff, Phi_autodiff,
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic>::type*>(
          nullptr));
  auto Phidot_vector = autoDiffToGradientMatrix(Phi_autodiff);
  Map<Matrix<Scalar, kQuaternionSize, kSpaceDimension>> Phid(
      Phidot_vector.data());

  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(
      cache, to_body_or_frame_ind, from_body_or_frame_ind, expressed_in);
  auto ret = (Phid * omega_twist).eval();
  ret.noalias() +=
      Phi * J_geometric_dot_times_v.template topRows<kSpaceDimension>();
  return ret;
}

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::relativeRollPitchYawJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "relativeRollPitchYawJacobianDotTimesV");

  auto rpy =
      relativeRollPitchYaw(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, kRpySize, kSpaceDimension> Phi;
  angularvel2rpydotMatrix(
      rpy, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr),
      static_cast<typename Gradient<decltype(Phi), Dynamic, 2>::type*>(
          nullptr));

  int expressed_in = to_body_or_frame_ind;
  const auto twist = relativeTwist(cache, to_body_or_frame_ind,
                                   from_body_or_frame_ind, expressed_in);
  auto omega_twist = twist.template topRows<kSpaceDimension>();
  auto rpydot = (Phi * omega_twist).eval();

  using ADScalar = AutoDiffScalar<Matrix<Scalar, Dynamic,
                                         1>>;  // would prefer to use 1 instead
  // of Dynamic, but this causes
  // issues related to
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1006 on MSVC
  // 32 bit
  auto rpy_autodiff = rpy.template cast<ADScalar>().eval();
  gradientMatrixToAutoDiff(rpydot, rpy_autodiff);
  Matrix<ADScalar, kRpySize, kSpaceDimension> Phi_autodiff;
  angularvel2rpydotMatrix(
      rpy_autodiff, Phi_autodiff,
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic>::type*>(
          nullptr),
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic, 2>::type*>(
          nullptr));
  auto Phidot_vector = autoDiffToGradientMatrix(Phi_autodiff);
  Map<Matrix<Scalar, kRpySize, kSpaceDimension>> Phid(Phidot_vector.data());

  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(
      cache, to_body_or_frame_ind, from_body_or_frame_ind, expressed_in);
  auto ret = (Phid * omega_twist).eval();
  ret.noalias() +=
      Phi * J_geometric_dot_times_v.template topRows<kSpaceDimension>();
  return ret;
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::FindBody(const std::string& body_name,
                                      const std::string& model_name,
                                      int model_instance_id) const {
  // Obtains lower case versions of the body name and model name.
  std::string body_name_lower = body_name;
  std::string model_name_lower = model_name;
  std::transform(body_name_lower.begin(), body_name_lower.end(),
                 body_name_lower.begin(), ::tolower);  // convert to lower case
  std::transform(model_name_lower.begin(), model_name_lower.end(),
                 model_name_lower.begin(), ::tolower);  // convert to lower case

  // Instantiates a variable that keeps track of the index within the frames
  // array that contains the desired frame. It is initialized to be -1 to
  // indicate that no frame was found.
  int match_index = -1;

  for (int i = 0; i < static_cast<int>(bodies.size()); ++i) {
    // Skips the current body if model_instance_id is not -1 and the body's
    // robot ID is not equal to the desired model instance ID.
    if (model_instance_id != -1 &&
        model_instance_id != bodies[i]->get_model_instance_id()) {
      continue;
    }

    // Obtains a lower case version of the current body's model name.
    string current_model_name = bodies[i]->get_model_name();
    std::transform(current_model_name.begin(), current_model_name.end(),
                   current_model_name.begin(), ::tolower);

    // Skips the current body if model_name is not empty and the body's model
    // name is not equal to the desired model name.
    if (!model_name_lower.empty() && model_name_lower != current_model_name)
      continue;

    // Obtains a lower case version of the current body's name.
    string current_body_name = bodies[i]->get_name();
    std::transform(current_body_name.begin(), current_body_name.end(),
                   current_body_name.begin(), ::tolower);

    // Checks if the body names match. If so, checks whether this is the first
    // match. If so, it saves the current body's index. Otherwise it throws
    // an exception indicating there are multiple matches.
    if (current_body_name == body_name_lower) {
      if (match_index < 0) {
        match_index = i;
      } else {
        throw std::logic_error(
            "RigidBodyTree::FindBody: ERROR: found multiple bodys named \"" +
                body_name + "\", model name = \"" + model_name +
                "\", model instance id = " + std::to_string(model_instance_id) +
                ".");
      }
    }
  }

  // Checks if a match was found. If so, returns a pointer to the matching
  // body. Otherwise, throws an exception indicating no match was found.
  if (match_index >= 0) {
    return bodies[match_index].get();
  } else {
    throw std::logic_error(
        "RigidBodyTree::FindBody: ERROR: Could not find body named \"" +
        body_name + "\", model name = \"" + model_name +
        "\", model instance id = " + std::to_string(model_instance_id) + ".");
  }
}

template <typename T>
const RigidBody<double>* RigidBodyTree<T>::FindBody(
    DrakeCollision::ElementId element_id) const {
  auto element = collision_model_->FindElement(element_id);
  if (element != nullptr) {
    return element->get_body();
  }
  throw std::logic_error(
      "RigidBodyTree::FindBody: ERROR: Could not find body for collision "
      "element id: " +
      std::to_string(element_id) + ".");
}

template <typename T>
std::vector<const RigidBody<T>*>
RigidBodyTree<T>::FindModelInstanceBodies(int model_instance_id) const {
  std::vector<const RigidBody<T>*> result;

  for (const auto& rigid_body : bodies) {
    // TODO(liang.fok): Remove the world name check once the world is assigned
    // its own model instance ID. See:
    // https://github.com/RobotLocomotion/drake/issues/3088
    if (rigid_body->get_name() != RigidBodyTreeConstants::kWorldName &&
        rigid_body->get_model_name() != RigidBodyTreeConstants::kWorldName &&
        rigid_body->get_model_instance_id() == model_instance_id) {
      result.push_back(rigid_body.get());
    }
  }
  return result;
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::findLink(const std::string& link_name,
                                      const std::string& model_name,
                                      int model_instance_id) const {
  return FindBody(link_name, model_name, model_instance_id);
}

template <typename T>
shared_ptr<RigidBodyFrame<T>> RigidBodyTree<T>::findFrame(
    const std::string& frame_name, int model_instance_id) const {
  std::string frame_name_lower = frame_name;

  // Obtains a lower case version of frame_name.
  std::transform(frame_name_lower.begin(), frame_name_lower.end(),
                 frame_name_lower.begin(), ::tolower);

  // Instantiates a variable that keeps track of the index within the frames
  // array that contains the desired frame. It is initialized to be -1 to
  // indicate that no matching frame was found.
  int match_index = -1;

  for (int i = 0; i < static_cast<int>(frames.size()); ++i) {
    // Skips the current frame if model_instance_id is not -1 and the frame's
    // model instance ID is not equal to the desired model instance ID.
    if (model_instance_id != -1 &&
        model_instance_id != frames[i]->get_model_instance_id()) {
      continue;
    }

    // Obtains a lower case version of the current frame.
    std::string current_frame_name = frames[i]->get_name();
    std::transform(current_frame_name.begin(), current_frame_name.end(),
                   current_frame_name.begin(), ::tolower);

    // Checks if the frame names match. If so, checks whether this is the first
    // match. If so, it saves the current frame's index. Otherwise it throws
    // an exception indicating there are multiple matches.
    if (frame_name_lower == current_frame_name) {
      if (match_index < 0) {
        match_index = i;
      } else {
        throw std::logic_error(
            "RigidBodyTree::findFrame: ERROR: Found multiple frames named \""
                + frame_name + "\", model_instance_id = " +
                std::to_string(model_instance_id));
      }
    }
  }

  // Checks if a match was found. If so, returns a pointer to the matching
  // frame. Otherwise, throws an exception indicating no match was found.
  if (match_index >= 0) {
    return frames[match_index];
  } else {
    throw std::logic_error(
        "RigidBodyTree::findFrame: ERROR: could not find frame named \"" +
            frame_name + "\", model instance id = " +
            std::to_string(model_instance_id) + ".");
  }
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindBaseBodies(int model_instance_id)
const {
  return FindChildrenOfBody(RigidBodyTreeConstants::kWorldBodyIndex,
                            model_instance_id);
}

template <typename T>
int RigidBodyTree<T>::FindBodyIndex(const std::string& body_name,
                                    int model_instance_id) const {
  RigidBody<T>* body = FindBody(body_name, "", model_instance_id);
  if (body == nullptr) {
    throw std::logic_error(
        "RigidBodyTree::FindBodyIndex: ERROR: Could not find index for "
            "rigid body \"" +
            body_name + "\", model_instance_id = " +
            std::to_string(model_instance_id) + ".");
  }
  return body->get_body_index();
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindChildrenOfBody(int parent_body_index,
    int model_instance_id) const {
  // Verifies that parameter parent_body_index is valid.
  DRAKE_DEMAND(parent_body_index >= 0 &&
      parent_body_index < get_num_bodies());

  // Obtains a reference to the parent body.
  const RigidBody<T>& parent_body = get_body(parent_body_index);

  // Checks every rigid body in this tree. If the rigid body is a child of
  // parent_body and its model instance ID matches model_instance_id, save its
  // index in the result vector.
  std::vector<int> children_indexes;
  for (int ii = 0; ii < static_cast<int>(bodies.size()); ++ii) {
    if (bodies.at(ii)->has_as_parent(parent_body)) {
      if (model_instance_id != -1) {
        if (bodies.at(ii)->get_model_instance_id() == model_instance_id) {
          children_indexes.push_back(ii);
        }
      } else {
        children_indexes.push_back(ii);
      }
    }
  }
  return children_indexes;
}

// TODO(liang.fok) Remove this method prior to Release 1.0.
template <typename T>
int RigidBodyTree<T>::findLinkId(const std::string& link_name,
                                 int model_instance_id) const {
  return FindBodyIndex(link_name, model_instance_id);
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::FindChildBodyOfJoint(
        const std::string& joint_name, int model_instance_id) const {
  // Obtains a lower case version of joint_name.
  std::string joint_name_lower = joint_name;
  std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                 joint_name_lower.begin(), ::tolower);

  vector<bool> name_match;
  name_match.resize(bodies.size());

  // For each rigid body in this RigidBodyTree, the following code saves a
  // `true` or `false` in vector `name_match` based on whether the body's parent
  // joint's name matches @p joint_name.
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (bodies[i]->has_parent_body()) {
      // Obtains the name of the rigid body's parent joint and then converts it
      // to be lower case.
      std::string current_joint_name = bodies[i]->getJoint().get_name();
      std::transform(current_joint_name.begin(), current_joint_name.end(),
                     current_joint_name.begin(), ::tolower);
      if (current_joint_name == joint_name_lower) {
        name_match[i] = true;
      } else {
        name_match[i] = false;
      }
    }
  }

  // If model_instance_id is specified, go through the matching joints and
  // remove those that do not belong to the specified model instance.
  if (model_instance_id != -1) {
    for (size_t i = 0; i < bodies.size(); ++i) {
      if (name_match[i]) {
        name_match[i] =
            (bodies[i]->get_model_instance_id() == model_instance_id);
      }
    }
  }

  // Checks to ensure only one match was found. Throws an `std::runtime_error`
  // if more than one match was found.
  size_t ind_match = 0;
  bool match_found = false;
  for (size_t i = 0; i < bodies.size(); ++i) {
    if (name_match[i]) {
      if (match_found) {
        throw std::runtime_error(
            "RigidBodyTree::FindChildBodyOfJoint: ERROR: Multiple joints "
                "found named \"" + joint_name + "\", model instance ID = " +
                std::to_string(model_instance_id) + ".");
      }
      ind_match = i;
      match_found = true;
    }
  }

  // Throws a `std::runtime_error` if no match was found. Otherwise, return
  // a pointer to the matching rigid body.
  if (!match_found) {
    throw std::runtime_error(
        "RigidBodyTree::FindChildBodyOfJoint: ERROR: Could not find unique "
            "joint named \"" + joint_name + "\", model_instance_id = " +
            std::to_string(model_instance_id) + ".");
  } else {
    return bodies[ind_match].get();
  }
}

template <typename T>
int RigidBodyTree<T>::FindIndexOfChildBodyOfJoint(const std::string& joint_name,
                                                  int model_instance_id) const {
  RigidBody<T>* link = FindChildBodyOfJoint(joint_name, model_instance_id);
  return link->get_body_index();
}

template <typename T>
const RigidBody<T>& RigidBodyTree<T>::get_body(int body_index) const {
  DRAKE_DEMAND(body_index >= 0 && body_index < get_num_bodies());
  return *bodies[body_index].get();
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::get_mutable_body(int body_index) {
  DRAKE_DEMAND(body_index >= 0 && body_index < get_num_bodies());
  return bodies[body_index].get();
}

template <typename T>
int RigidBodyTree<T>::get_num_bodies() const {
  return static_cast<int>(bodies.size());
}

// TODO(liang.fok) Remove this method prior to Release 1.0.
template <typename T>
int RigidBodyTree<T>::get_number_of_bodies() const {
  return get_num_bodies();
}

template <typename T>
int RigidBodyTree<T>::get_num_frames() const {
  return static_cast<int>(frames.size());
}

// TODO(liang.fok) Remove this method prior to Release 1.0.
template <typename T>
RigidBody<T>* RigidBodyTree<T>::findJoint(const std::string& joint_name,
                                       int model_id) const {
  return FindChildBodyOfJoint(joint_name, model_id);
}

// TODO(liang.fok) Remove this method prior to Release 1.0.
template <typename T>
int RigidBodyTree<T>::findJointId(const std::string& joint_name, int model_id)
const {
  return  FindIndexOfChildBodyOfJoint(joint_name, model_id);
}

template <typename T>
std::string RigidBodyTree<T>::getBodyOrFrameName(int body_or_frame_id) const {
  if (body_or_frame_id >= 0) {
    return bodies[body_or_frame_id]->get_name();
  } else if (body_or_frame_id < -1) {
    return frames[-body_or_frame_id - 2]->get_name();
  } else {
    return "COM";
  }
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree<T>::positionConstraints(
    const KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, 1> ret(6 * loops.size(), 1);
  for (size_t i = 0; i < loops.size(); ++i) {
    {  // position constraint
      auto ptA_in_B = transformPoints(cache, Vector3d::Zero(),
                                      loops[i].frameA_->get_frame_index(),
                                      loops[i].frameB_->get_frame_index());
      ret.template middleRows<3>(6 * i) = ptA_in_B;
    }
    {  // second position constraint (to constrain orientation)
      auto axis_A_end_in_B = transformPoints(
          cache, loops[i].axis_, loops[i].frameA_->get_frame_index(),
          loops[i].frameB_->get_frame_index());
      ret.template middleRows<3>(6 * i + 3) = axis_A_end_in_B - loops[i].axis_;
    }
  }
  return ret;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::positionConstraintsJacobian(
    const KinematicsCache<Scalar>& cache, bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(
      6 * loops.size(), in_terms_of_qdot ? num_positions_ : num_velocities_);

  for (size_t i = 0; i < loops.size(); ++i) {
    // position constraint
    ret.template middleRows<3>(6 * i) = transformPointsJacobian(
        cache, Vector3d::Zero(), loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index(), in_terms_of_qdot);
    // second position constraint (to constrain orientation)
    ret.template middleRows<3>(6 * i + 3) = transformPointsJacobian(
        cache, loops[i].axis_, loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index(), in_terms_of_qdot);
  }
  return ret;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::positionConstraintsJacDotTimesV(
    const KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, 1> ret(6 * loops.size(), 1);

  for (size_t i = 0; i < loops.size(); ++i) {
    // position constraint
    ret.template middleRows<3>(6 * i) = transformPointsJacobianDotTimesV(
        cache, Vector3d::Zero(), loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index());
    // second position constraint (to constrain orientation)
    ret.template middleRows<3>(6 * i + 3) = transformPointsJacobianDotTimesV(
        cache, loops[i].axis_, loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index());
  }
  return ret;
}

template <typename T>
template <typename DerivedA, typename DerivedB, typename DerivedC>
void RigidBodyTree<T>::jointLimitConstraints(
    MatrixBase<DerivedA> const& q,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    MatrixBase<DerivedB>& phi,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    MatrixBase<DerivedC>& J) const {
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  const size_t numFiniteMin = finite_min_index.size();
  const size_t numFiniteMax = finite_max_index.size();

  phi = VectorXd::Zero(numFiniteMin + numFiniteMax);
  J = MatrixXd::Zero(phi.size(), num_positions_);
  for (size_t i = 0; i < numFiniteMin; ++i) {
    const int fi = finite_min_index[i];
    phi[i] = q[fi] - joint_limit_min[fi];
    J(i, fi) = 1.0;
  }

  for (size_t i = 0; i < numFiniteMax; ++i) {
    const int fi = finite_max_index[i];
    phi[i + numFiniteMin] = joint_limit_max[fi] - q[fi];
    J(i + numFiniteMin, fi) = -1.0;
  }
}

template <typename T>
size_t RigidBodyTree<T>::getNumJointLimitConstraints() const {
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  return finite_min_index.size() + finite_max_index.size();
}

template <typename T>
size_t RigidBodyTree<T>::getNumPositionConstraints() const {
  return loops.size() * 6;
}

template <typename T>
void RigidBodyTree<T>::addFrame(std::shared_ptr<RigidBodyFrame<T>> frame) {
  frames.push_back(frame);
  // yuck!!
  frame->set_frame_index(-(static_cast<int>(frames.size()) - 1) - 2);
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::add_rigid_body(
        std::unique_ptr<RigidBody<T>> body) {
  // TODO(amcastro-tri): body indexes should not be initialized here but on an
  // initialize call after all bodies and RigidBodySystem's are defined.
  // This initialize call will make sure that all global and local indexes are
  // properly computed taking into account a RigidBodySystem could be part of a
  // larger RigidBodySystem (a system within a tree of systems).
  body->set_body_index(static_cast<int>(bodies.size()));

  // bodies will be sorted by SortTree by generation. Therefore bodies[0]
  // (world) will be at the top and subsequent generations of children will
  // follow.
  bodies.push_back(std::move(body));
  return bodies.back().get();
}

template <typename T>
int RigidBodyTree<T>::get_num_positions() const {
  return num_positions_;
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
template <typename T>
int RigidBodyTree<T>::number_of_positions() const {
  return get_num_positions();
}

template <typename T>
int RigidBodyTree<T>::get_num_velocities() const {
  return num_velocities_;
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
template <typename T>
int RigidBodyTree<T>::number_of_velocities() const {
  return get_num_velocities();
}

template <typename T>
int RigidBodyTree<T>::get_num_actuators() const {
  return static_cast<int>(actuators.size());
}

template <typename T>
int RigidBodyTree<T>::add_model_instance() {
  return num_model_instances_++;
}

// TODO(liang.fok) Update this method implementation once the world is assigned
// its own model instance ID (#3088). It should return num_model_instances_ - 1.
template <typename T>
int RigidBodyTree<T>::get_num_model_instances() const {
  return num_model_instances_;
}

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
template <typename T>
int RigidBodyTree<T>::get_number_of_model_instances() const {
  return get_num_model_instances();
}

template <typename T>
Isometry3<T> RigidBodyTree<T>::CalcFramePoseInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF) const {
  cache.checkCachedKinematicsSettings(
      false, false, "CalcFramePoseInWorldFrame");

  const auto& body_element = cache.get_element(body.get_body_index());
  const Isometry3<T> X_WB = body_element.transform_to_world;
  return X_WB * X_BF;
}

template <typename T>
Vector6<T> RigidBodyTree<T>::CalcBodySpatialVelocityInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body) const {
  cache.checkCachedKinematicsSettings(
      true, false, "CalcBodySpatialVelocityInWorldFrame");

  const auto& body_element = cache.get_element(body.get_body_index());

  // Position of the frame B's origin in the world frame.
  const auto& p_WB = body_element.transform_to_world.translation();

  // body_element.twist_in_world is the spatial velocity of frame Bwo measured
  // and expressed in the world frame, where Bwo is rigidly attached to B and
  // instantaneously coincides with the world frame.
  const Vector6<T>& V_WBwo = body_element.twist_in_world;

  Vector6<T> V_WB = V_WBwo;

  // Computes V_WB from V_WBwo.
  const auto& w_WB = V_WBwo.template topRows<3>();
  V_WB.template bottomRows<3>() += w_WB.cross(p_WB);

  return V_WB;
}

template <typename T>
drake::Vector6<T> RigidBodyTree<T>::CalcFrameSpatialVelocityInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF) const {
  // Spatial velocity of body B with respect to the world W, expressed in
  // the world frame W.
  Vector6<T> V_WB =
      CalcBodySpatialVelocityInWorldFrame(cache, body);

  // Angular velocity of frame B with respect to W, expressed in W.
  const auto& w_WB = V_WB.template topRows<3>();
  // Linear velocity of frame B with respect to W, expressed in W.
  const auto& v_WB = V_WB.template bottomRows<3>();

  // Body pose measured and expressed in the world frame.
  Isometry3<T> X_WB = CalcBodyPoseInWorldFrame(cache, body);
  // Vector from Bo to Fo expressed in B.
  Vector3<T> p_BF = X_BF.translation();
  // Vector from Bo to Fo expressed in W.
  Vector3<T> p_BF_W = X_WB.linear() * p_BF;

  // Spatial velocity of frame F with respect to the world frame W, expressed in
  // the world frame.
  Vector6<T> V_WF;
  // Aliases to angular and linear components in the spatial velocity vector.
  auto w_WF = V_WF.template topRows<3>();
  auto v_WF = V_WF.template bottomRows<3>();

  // Compute the spatial velocity of frame F.
  w_WF = w_WB;
  v_WF = v_WB + w_WB.cross(p_BF_W);

  return V_WF;
}

template <typename T> drake::Matrix6X<T>
RigidBodyTree<T>::CalcFrameSpatialVelocityJacobianInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF, bool in_terms_of_qdot) const {
  const int world_index = world().get_body_index();
  const int num_col =
      in_terms_of_qdot ? get_num_positions() : get_num_velocities();

  drake::Vector3<T> p_WF =
      CalcFramePoseInWorldFrame(cache, body, X_BF).translation();

  std::vector<int> v_or_q_indices;
  // J_WBwo is the Jacobian of the spatial velocity of frame Bwo measured
  // and expressed in the world frame, where Bwo is rigidly attached to B and
  // instantaneously coincides with the world frame.
  drake::MatrixX<T> J_WBwo = geometricJacobian(
      cache, world_index, body.get_body_index(), world_index, in_terms_of_qdot,
      &v_or_q_indices);

  int col = 0;
  drake::Matrix6X<T> J_WF = MatrixX<T>::Zero(6, num_col);
  for (int idx : v_or_q_indices) {
    // Angular velocity stays the same.
    J_WF.col(idx) = J_WBwo.col(col);
    // Linear velocity needs an additional cross product term.
    J_WF.col(idx).template tail<3>() +=
        J_WBwo.col(col).template head<3>().cross(p_WF);
    col++;
  }
  return J_WF;
}

template <typename T> drake::Vector6<T>
RigidBodyTree<T>::CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF) const {
  const int world_index = world().get_body_index();
  const int body_index = body.get_body_index();
  Vector3<T> p_WF = CalcFramePoseInWorldFrame(cache, body, X_BF).translation();
  Vector6<T> V_WF = CalcFrameSpatialVelocityInWorldFrame(cache, body, X_BF);
  Vector3<T> pdot_WF = V_WF.template tail<3>();
  Vector3<T> w_WF = V_WF.template head<3>();

  // Define frame Bwo, which is rigidly attached to B and instantaneously
  // coincides with the world frame. V_WBwo is the spatial velocity of Bwo
  // measured and expressed in the world frame. Jdv_WBwo is the Jacobian dot of
  // V_WBwo times the generalized velocity.
  TwistVector<T> Jdv_WBwo =
      geometricJacobianDotTimesV(cache, world_index, body_index, world_index);

  // For column i of J_WF,
  // J_WF(i) = [J_WBwo_ang(i); J_WBwo_lin(i) + J_WBwo_ang(i) x p_WF],
  // where _ang and _lin are the angular and linear components respectively.
  // Thus, for Jdv_WF, the angular part is the same with the angular part of
  // J_WBwo. For the linear part:
  //  = [Jdot_WBwo_lin + Jdot_WBwo_ang x p_WF + J_WBwo_ang x pdot_WF] * v
  //  = [Jdv_WBwo_lin + Jdv_WBwo_ang x p_WF + w_WF x pdot_WF]
  TwistVector<T> Jdv_WF = Jdv_WBwo;
  Jdv_WF.template tail<3>() += w_WF.template head<3>().cross(pdot_WF) +
                               Jdv_WBwo.template head<3>().cross(p_WF);
  return Jdv_WF;
}

// Explicit template instantiations for massMatrix.
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::massMatrix<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&) const;
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::massMatrix<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&) const;
template MatrixXd
RigidBodyTree<double>::massMatrix<double>(KinematicsCache<double>&) const;

// Explicit template instantiations for centerOfMass.
template Vector3<AutoDiffUpTo73d>
RigidBodyTree<double>::centerOfMass<AutoDiffUpTo73d>(
    const KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&) const;
template Vector3<AutoDiffXd>
RigidBodyTree<double>::centerOfMass<AutoDiffXd>(
    const KinematicsCache<AutoDiffXd>&,
    set<int, less<int>, allocator<int>> const&) const;
template Vector3d RigidBodyTree<double>::centerOfMass<double>(
    const KinematicsCache<double>&,
    set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for GetVelocityToQDotMapping.
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::GetVelocityToQDotMapping(
        const KinematicsCache<AutoDiffUpTo73d>&);
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::GetVelocityToQDotMapping(
        const KinematicsCache<AutoDiffXd>&);
template MatrixX<double>
RigidBodyTree<double>::GetVelocityToQDotMapping(
        const KinematicsCache<double>&);

// Explicit template instantiations for GetQDotToVelocityMapping
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::GetQDotToVelocityMapping(
        const KinematicsCache<AutoDiffUpTo73d>&);
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::GetQDotToVelocityMapping(
        const KinematicsCache<AutoDiffXd>&);
template MatrixX<double>
RigidBodyTree<double>::GetQDotToVelocityMapping(
        const KinematicsCache<double>&);

// Explicit template instantiations for dynamicsBiasTerm.
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::dynamicsBiasTerm<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    unordered_map<
        RigidBody<double> const*, WrenchVector<AutoDiffUpTo73d>,
        hash<RigidBody<double> const*>,
        equal_to<RigidBody<double> const*>,
        Eigen::aligned_allocator<pair<RigidBody<double> const* const,
                                      WrenchVector<AutoDiffUpTo73d>>>> const&,
    bool) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::dynamicsBiasTerm<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    unordered_map<
        RigidBody<double> const*, WrenchVector<AutoDiffXd>,
        hash<RigidBody<double> const*>,
        equal_to<RigidBody<double> const*>,
        Eigen::aligned_allocator<
        pair<RigidBody<double> const* const, WrenchVector<AutoDiffXd>>>> const&,
    bool) const;
template VectorXd RigidBodyTree<double>::dynamicsBiasTerm<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody<double> const*, WrenchVector<double>,
                  hash<RigidBody<double> const*>,
                  equal_to<RigidBody<double> const*>,
                  Eigen::aligned_allocator<pair<RigidBody<double> const* const,
                                                WrenchVector<double>>>> const&,
    bool) const;

// Explicit template instantiations for geometricJacobian.
template TwistMatrix<AutoDiffUpTo73d>
RigidBodyTree<double>::geometricJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int, bool,
    vector<int, allocator<int>>*) const;
template TwistMatrix<AutoDiffXd>
RigidBodyTree<double>::geometricJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int, bool,
    vector<int, allocator<int>>*) const;
template TwistMatrix<double>
RigidBodyTree<double>::geometricJacobian<double>(
    KinematicsCache<double> const&,
    int, int, int, bool, vector<int, allocator<int>>*) const;

// Explicit template instantiations for relativeTransform.
template Eigen::Transform<AutoDiffUpTo73d, 3, 1, 0>
RigidBodyTree<double>::relativeTransform<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;
template Eigen::Transform<AutoDiffXd, 3, 1, 0>
RigidBodyTree<double>::relativeTransform<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int) const;
template Eigen::Transform<double, 3, 1, 0>
RigidBodyTree<double>::relativeTransform<double>(
    KinematicsCache<double> const&, int, int) const;

// Explicit template instantiations for centerOfMassJacobian.
template Matrix3X<AutoDiffUpTo73d>
RigidBodyTree<double>::centerOfMassJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&, bool) const;
template Matrix3X<AutoDiffXd>
RigidBodyTree<double>::centerOfMassJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template Matrix3Xd
RigidBodyTree<double>::centerOfMassJacobian<double>(
    KinematicsCache<double>&,
    set<int, less<int>, allocator<int>> const&, bool) const;

// Explicit template instantiations for centroidalMomentumMatrix.
template TwistMatrix<AutoDiffUpTo73d>
RigidBodyTree<double>::centroidalMomentumMatrix<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&, bool) const;
template TwistMatrix<AutoDiffXd>
RigidBodyTree<double>::centroidalMomentumMatrix<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template TwistMatrix<double>
RigidBodyTree<double>::centroidalMomentumMatrix<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;

// Explicit template instantiations for forwardKinPositionGradient.
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::forwardKinPositionGradient<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int) const;
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::forwardKinPositionGradient<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int) const;
template MatrixXd
RigidBodyTree<double>::forwardKinPositionGradient<double>(
    KinematicsCache<double> const&, int, int, int) const;

// Explicit template instantiations for geometricJacobianDotTimesV.
template TwistVector<AutoDiffUpTo73d>
RigidBodyTree<double>::geometricJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int) const;
template TwistVector<AutoDiffXd>
RigidBodyTree<double>::geometricJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int) const;
template TwistVector<double>
RigidBodyTree<double>::geometricJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int, int) const;

// Explicit template instantiations for centerOfMassJacobianDotTimesV.
template Vector3<AutoDiffUpTo73d>
RigidBodyTree<double>::centerOfMassJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&) const;
template Vector3<AutoDiffXd>
RigidBodyTree<double>::centerOfMassJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    set<int, less<int>, allocator<int>> const&) const;
template Vector3d
RigidBodyTree<double>::centerOfMassJacobianDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for centroidalMomentumMatrixDotTimesV.
template TwistVector<AutoDiffUpTo73d>
RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&) const;
template TwistVector<AutoDiffXd>
RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    set<int, less<int>, allocator<int>> const&) const;
template TwistVector<double>
RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template VectorXd
RigidBodyTree<double>::positionConstraints<double>(
    KinematicsCache<double> const&) const;

// Explicit template instantiations for positionConstraintsJacobian.
template MatrixXd
RigidBodyTree<double>::positionConstraintsJacobian<double>(
    KinematicsCache<double> const&, bool) const;

// Explicit template instantiations for positionConstraintsJacDotTimesV.
template VectorXd
RigidBodyTree<double>::positionConstraintsJacDotTimesV<double>(
    KinematicsCache<double> const&) const;
template void RigidBodyTree<double>::jointLimitConstraints<
    VectorXd, VectorXd, MatrixXd>(Eigen::MatrixBase<VectorXd> const&,
                                  Eigen::MatrixBase<VectorXd>&,
                                  Eigen::MatrixBase<MatrixXd>&) const;

// Explicit template instantiations for relativeTwist.
template TwistVector<double>
RigidBodyTree<double>::relativeTwist<double>(
    KinematicsCache<double> const&, int, int, int) const;

// Explicit template instantiations for worldMomentumMatrix.
template TwistMatrix<AutoDiffUpTo73d>
RigidBodyTree<double>::worldMomentumMatrix<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&, bool) const;
template TwistMatrix<AutoDiffXd>
RigidBodyTree<double>::worldMomentumMatrix<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template TwistMatrix<double>
RigidBodyTree<double>::worldMomentumMatrix<double>(KinematicsCache<double>&,
            set<int, less<int>, allocator<int>> const&, bool) const;

// Explicit template instantiations for worldMomentumMatrixDotTimesV.
template TwistVector<double>
RigidBodyTree<double>::worldMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for transformSpatialAcceleration.
template TwistVector<double>
RigidBodyTree<double>::transformSpatialAcceleration<double>(
    KinematicsCache<double> const&, TwistVector<double> const&, int, int, int,
    int) const;

// Explicit template instantiations for frictionTorques
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::frictionTorques(
    Eigen::MatrixBase<VectorX<AutoDiffUpTo73d>> const& v) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::frictionTorques(
    Eigen::MatrixBase<VectorX<AutoDiffXd>> const& v) const;
template VectorX<double> RigidBodyTree<double>::frictionTorques(
    Eigen::MatrixBase<VectorX<double>> const& v) const;

// Explicit template instantiations for inverseDynamics.
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::inverseDynamics<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    unordered_map<
        RigidBody<double> const*, TwistVector<AutoDiffUpTo73d>,
        hash<RigidBody<double> const*>,
        equal_to<RigidBody<double> const*>,
        Eigen::aligned_allocator<
        pair<RigidBody<double> const* const,
                TwistVector<AutoDiffUpTo73d>>>> const&,
    VectorX<AutoDiffUpTo73d> const&, bool) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::inverseDynamics<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    unordered_map<RigidBody<double> const*, TwistVector<AutoDiffXd>,
                  hash<RigidBody<double> const*>,
                  equal_to<RigidBody<double> const*>,
                  Eigen::aligned_allocator<pair<
                  RigidBody<double> const* const,
                  TwistVector<AutoDiffXd>>>> const&,
    VectorX<AutoDiffXd> const&, bool) const;
template VectorX<double>
RigidBodyTree<double>::inverseDynamics<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody<double> const*, WrenchVector<double>,
                  hash<RigidBody<double> const*>,
                  equal_to<RigidBody<double> const*>,
                  Eigen::aligned_allocator<pair<RigidBody<double> const* const,
                                                WrenchVector<double>>>> const&,
    VectorX<double> const&, bool) const;

// Explicit template instantiations for jointLimitConstraints.
template void RigidBodyTree<double>::jointLimitConstraints<
    Eigen::Map<VectorXd>, Eigen::Map<VectorXd>, Eigen::Map<MatrixXd>>(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd>>&,
    Eigen::MatrixBase<Eigen::Map<MatrixXd>>&) const;

// Explicit template instantiations for resolveCenterOfPressure.
template pair<Vector3d, double>
RigidBodyTree<double>::resolveCenterOfPressure<Vector3d, Vector3d>(
    KinematicsCache<double> const&,
    vector<ForceTorqueMeasurement, allocator<ForceTorqueMeasurement>> const&,
    Eigen::MatrixBase<Vector3d> const&,
    Eigen::MatrixBase<Vector3d> const&) const;

// Explicit template instantiations for transformVelocityMappingToQDotMapping.
template MatrixX<double>
RigidBodyTree<double>::transformVelocityMappingToQDotMapping<VectorXd>(
    const KinematicsCache<double>&,
    const Eigen::MatrixBase<VectorXd>&);
template MatrixX<double>
RigidBodyTree<double>::transformVelocityMappingToQDotMapping<
    Eigen::RowVectorXd>(
    const KinematicsCache<double>&,
    const Eigen::MatrixBase<Eigen::RowVectorXd>&);

// Explicit template instantiations for transformQDotMappingToVelocityMapping.
template MatrixX<double>
RigidBodyTree<double>::transformQDotMappingToVelocityMapping<VectorXd>(
    const KinematicsCache<double>&,
    const Eigen::MatrixBase<VectorXd>&);
template MatrixX<double>
RigidBodyTree<double>::transformQDotMappingToVelocityMapping<
    Eigen::RowVectorXd>(
    const KinematicsCache<double>&,
    const Eigen::MatrixBase<Eigen::RowVectorXd>&);
template MatrixX<double>
RigidBodyTree<double>::transformQDotMappingToVelocityMapping<MatrixXd>(
        const KinematicsCache<double>&,
        const Eigen::MatrixBase<MatrixXd>&);
template MatrixX<double>
RigidBodyTree<double>::transformQDotMappingToVelocityMapping<
        Eigen::Map<MatrixXd const>>(
        const KinematicsCache<double>&,
        const Eigen::MatrixBase<Eigen::Map<MatrixXd const>>&);
template MatrixX<double>
RigidBodyTree<double>::transformQDotMappingToVelocityMapping<
        Eigen::Map<MatrixXd>>(
        const KinematicsCache<double>&,
        const Eigen::MatrixBase<Eigen::Map<MatrixXd>>&);

// Explicit template instantiations for transformPointsJacobian.
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::transformPointsJacobian<AutoDiffUpTo73d, Matrix3Xd>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Matrix3Xd> const&, int, int, bool) const;
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::transformPointsJacobian<AutoDiffXd, Matrix3Xd>(
    KinematicsCache<AutoDiffXd> const&, Eigen::MatrixBase<Matrix3Xd> const&,
    int, int, bool) const;
template MatrixXd
RigidBodyTree<double>::transformPointsJacobian<double, Matrix3Xd>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int,
    int, bool) const;
template MatrixXd
RigidBodyTree<double>::transformPointsJacobian<double, Vector3d>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Vector3d> const&, int,
    int, bool) const;
template MatrixXd RigidBodyTree<double>::transformPointsJacobian<
    double, Eigen::Block<Matrix3Xd, 3, 1, true>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Block<Matrix3Xd, 3, 1, true>> const&, int, int,
    bool) const;
template MatrixX<AutoDiffUpTo73d>
RigidBodyTree<double>::transformPointsJacobian<AutoDiffUpTo73d,
                                               Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;
template MatrixX<AutoDiffXd>
RigidBodyTree<double>::transformPointsJacobian<
    AutoDiffXd, Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffXd> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;
template MatrixXd
RigidBodyTree<double>::transformPointsJacobian<
    double, Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;

// Explicit template instantiations for transformPointsJacobianDotTimesV.
template VectorXd
RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Matrix3Xd>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int,
    int) const;
template VectorXd
RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Vector3d>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Vector3d> const&, int,
    int) const;
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::transformPointsJacobianDotTimesV<AutoDiffUpTo73d,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::transformPointsJacobianDotTimesV<AutoDiffXd,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffXd> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;
template VectorXd
RigidBodyTree<double>::transformPointsJacobianDotTimesV<double,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;

// Explicit template instantiations for relativeQuaternionJacobian.
template Matrix4Xd
RigidBodyTree<double>::relativeQuaternionJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template Matrix4X<AutoDiffUpTo73d>
RigidBodyTree<double>::relativeQuaternionJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, bool) const;
template Matrix4X<AutoDiffXd>
RigidBodyTree<double>::relativeQuaternionJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, bool) const;

// Explicit template instantiations for relativeRollPitchYawJacobian.
template Matrix3Xd
RigidBodyTree<double>::relativeRollPitchYawJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template Matrix3X<AutoDiffUpTo73d>
RigidBodyTree<double>::relativeRollPitchYawJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, bool) const;
template Matrix3X<AutoDiffXd>
RigidBodyTree<double>::relativeRollPitchYawJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, bool) const;

// Explicit template instantiations for relativeRollPitchYawJacobianDotTimesV.
template VectorXd
RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int) const;
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;

// Explicit template instantiations for relativeQuaternionJacobianDotTimesV.
template VectorXd
RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template VectorX<AutoDiffUpTo73d>
RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;
template VectorX<AutoDiffXd>
RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int) const;

// Explicit template instantiations for CheckCacheValidity(cache).
template void RigidBodyTree<double>::CheckCacheValidity(
    const KinematicsCache<double>&) const;
template void RigidBodyTree<double>::CheckCacheValidity(
    const KinematicsCache<AutoDiffXd>&) const;
template void RigidBodyTree<double>::CheckCacheValidity(
    const KinematicsCache<AutoDiffUpTo73d>&) const;

// Explicit template instantiations for doKinematics(cache).
template void RigidBodyTree<double>::doKinematics(
    KinematicsCache<double>&, bool) const;
template void RigidBodyTree<double>::doKinematics(
    KinematicsCache<AutoDiffXd>&, bool) const;
template void RigidBodyTree<double>::doKinematics(
    KinematicsCache<AutoDiffUpTo73d>&, bool) const;

// Explicit template instantiations for doKinematics(q).
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<VectorXd> const&) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Block<MatrixXd const, -1, 1, true>> const&) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Block<MatrixXd, -1, 1, true>> const&) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&) const;
template KinematicsCache<AutoDiffXd>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<VectorX<AutoDiffXd>> const&) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Block<VectorXd, -1, 1, false>> const&) const;

// Explicit template instantiations for doKinematics(q, v).
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<VectorXd> const&, Eigen::MatrixBase<VectorXd> const&,
    bool) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&,
    Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&,
    bool) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Block<VectorXd, -1, 1, false>> const&,
    Eigen::MatrixBase<Eigen::Block<VectorXd, -1, 1, false>> const&, bool) const;
template KinematicsCache<AutoDiffXd>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<VectorX<AutoDiffXd>> const&,
    Eigen::MatrixBase<VectorX<AutoDiffXd>> const&, bool) const;
template KinematicsCache<AutoDiffUpTo73d>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<VectorX<AutoDiffUpTo73d>> const&,
    Eigen::MatrixBase<VectorX<AutoDiffUpTo73d>> const&, bool) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&, bool) const;
template KinematicsCache<double>
RigidBodyTree<double>::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd const>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd const>> const&, bool) const;

// Explicit template instantiations for parseBodyOrFrameID.
template int RigidBodyTree<double>::parseBodyOrFrameID(
    const int body_or_frame_id,
    Eigen::Transform<double, 3, Eigen::Isometry>* Tframe) const;

// Explicit template instantiations for CreateKinematicsCacheWithType.
template
KinematicsCache<AutoDiffXd>
RigidBodyTree<double>::CreateKinematicsCacheWithType<AutoDiffXd>() const;
template
KinematicsCache<AutoDiffUpTo73d>
RigidBodyTree<double>::CreateKinematicsCacheWithType<AutoDiffUpTo73d>() const;

// Explicitly instantiates on the most common scalar types.
template class RigidBodyTree<double>;
