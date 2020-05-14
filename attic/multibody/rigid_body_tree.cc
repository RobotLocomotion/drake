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

#include "drake/attic_warning.h"
#include "drake/common/autodiff.h"
#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/gradient.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/joints/fixed_joint.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematics_cache-inl.h"
#include "drake/multibody/resolve_center_of_pressure.h"
#include "drake/util/drakeGeometryUtil.h"

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

using std::allocator;
using std::cerr;
using std::cout;
using std::equal_to;
using std::hash;
using std::less;
using std::make_shared;
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

// We compile rigid_body_tree.cc in shards (only a portion of the source code
// in the file is compiled at a time) in order to reduce build latency and
// mitigate the memory footprint of any single compiler instance.
//
// The build system will compile this file multiple times, each with a
// different numeric value for DRAKE_RBT_SHARD.
//
// Within this file, we guard various functions or instantiations with
// preprocessor statements like "if DRAKE_RBT_SHARD == 0".
//
// In shard 0 we compile all class methods that are not also method templates,
// as well as some method templates.  In shard 1 we compile the remainder of
// the method templates.
//
// The shard assignments are chosen so that methods that might benefit from
// inlining their calls are within the same shard.  (In other words, the shards
// draw a partition through the call graph that crosses as few performance-
// critical edges as practical.)  When assigning or updating new methods to a
// shard, try to assign the new method to the same shard as the methods it
// calls.  If it does not need to call anything else, assign it to the shard
// that compiles most quickly currently.

#ifndef DRAKE_RBT_SHARD
#error Missing DRAKE_RBT_SHARD definition
#endif

#if DRAKE_RBT_SHARD == 0

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

// Suppress deprecation warnings when constructing `bodies` and `frames`.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename T>
RigidBodyTree<T>::RigidBodyTree()
    : collision_model_(drake::multibody::collision::newModel()) {
#pragma GCC diagnostic pop  // pop -Wdeprecated-declarations

  drake::internal::WarnOnceAboutAtticCode();

  // Sets the gravity vector.
  a_grav << 0, 0, 0, 0, 0, -9.81;

  // Adds the rigid body representing the world. It has model instance ID 0.
  std::unique_ptr<RigidBody<T>> world_body(new RigidBody<T>());
  world_body->set_name(RigidBodyTreeConstants::kWorldName);
  world_body->set_model_name(RigidBodyTreeConstants::kWorldName);

  // TODO(liang.fok): Assign the world body a unique model instance ID of zero.
  // See: https://github.com/RobotLocomotion/drake/issues/3088

  add_rigid_body(std::move(world_body));
}

template <typename T>
RigidBodyTree<T>::~RigidBodyTree() {}

template <>
unique_ptr<RigidBodyTree<double>> RigidBodyTree<double>::Clone() const {
  auto clone = make_unique<RigidBodyTree<double>>();
  // The following is necessary to remove the world link from the clone. The
  // world link will be re-added when the bodies are cloned below.
  clone->bodies_.clear();
  clone->frames_.clear();

  clone->joint_limit_min = this->joint_limit_min;
  clone->joint_limit_max = this->joint_limit_max;
  clone->a_grav = this->a_grav;
  clone->B = this->B;
  clone->num_positions_ = this->num_positions_;
  clone->num_velocities_ = this->num_velocities_;
  clone->num_model_instances_ = this->num_model_instances_;
  clone->initialized_ = this->initialized_;

  // N.B. `add_rigid_body` is not used here because this may change the ordering
  // of frames, and the clone tests require that they maintain their original
  // ordering / indices.

  // Clones the rigid bodies.
  for (const auto& body : bodies_) {
    clone->bodies_.push_back(body->Clone());
  }

  // Clones the joints and adds them to the cloned RigidBody objects.
  for (const auto& original_body : bodies_) {
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

  for (const auto& original_frame : frames_) {
    // Find the body in the RigidBodyTree clone that corresponds to our
    // original_frame.
    const RigidBody<double>& original_frame_body =
        original_frame->get_rigid_body();
    const int cloned_frame_body_index =
        clone->FindBodyIndex(original_frame_body.get_name(),
                             original_frame_body.get_model_instance_id());
    RigidBody<double>* cloned_frame_body =
        clone->get_mutable_body(cloned_frame_body_index);
    DRAKE_DEMAND(cloned_frame_body != nullptr);

    // Clone original_frame and fix up its body pointer to refer to the body
    // from the cloned RigidBodyTree instead of us.
    auto cloned_frame = make_shared<RigidBodyFrame<double>>(*original_frame);
    cloned_frame->set_rigid_body(cloned_frame_body);
    clone->frames_.push_back(cloned_frame);
  }

  for (const auto& actuator : actuators) {
    const RigidBody<double>& cloned_body =
        clone->get_body(actuator.body_->get_body_index());
    clone->actuators.emplace_back(
        actuator.name_, &cloned_body, actuator.reduction_,
        actuator.effort_limit_min_, actuator.effort_limit_max_);
  }

  for (const auto& loop : loops) {
    std::shared_ptr<RigidBodyFrame<double>> frame_a = clone->findFrame(
        loop.frameA_->get_name(), loop.frameA_->get_model_instance_id());
    std::shared_ptr<RigidBodyFrame<double>> frame_b = clone->findFrame(
        loop.frameB_->get_name(), loop.frameB_->get_model_instance_id());
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
    drake::multibody::collision::Element* element = *body_itr;
    if (!collision_model_->TransformCollisionFrame(element->getId(),
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
  if (bodies_.size() == 0) return;  // no-op if there are no RigidBody's

  for (size_t i = 0; i < bodies_.size() - 1;) {
    if (bodies_[i]->has_parent_body()) {
      auto iter = std::find_if(bodies_.begin() + i + 1, bodies_.end(),
                               [&](std::unique_ptr<RigidBody<T>> const& p) {
                                 return bodies_[i]->has_as_parent(*p);
                               });
      if (iter != bodies_.end()) {
        std::unique_ptr<RigidBody<T>> parent = std::move(*iter);
        bodies_.erase(iter);
        bodies_.insert(bodies_.begin() + i, std::move(parent));
        --i;
      }
    }
    ++i;
  }

  // Re-assign body_index to be the i-th entry in RBT::bodies
  for (size_t i = 0; i < bodies_.size(); ++i) {
    bodies_[i]->set_body_index(static_cast<int>(i));
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
  RigidBody<T>* body = bodies_[body_index].get();
  if (!collision_group_manager_.AddCollisionFilterGroupMember(group_name,
                                                              *body)) {
    throw std::logic_error(
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
void RigidBodyTree<T>::compile() {
  SortTree();

  // Welds joints for links that have zero inertia and no children (as seen in
  // pr2_simplified.urdf)
  // TODO(amcastro-tri): this is O(n^2). RigidBody should contain a list of
  // children
  // TODO(amcastro-tri): the order in which these loops should be performed
  // should be stated more clearly with an iterator.
  // An option would be to have:
  //   RigidBodyTree::upwards_body_iterator: travels the tree upwards towards
  //   the root.
  //   RigidBodyTree::downwards_body_iterator: travels the tree downwards
  //   from the root towards the last leaf.
  for (size_t i = 0; i < bodies_.size(); ++i) {
    const auto& body = bodies_[i];

    if (body->has_parent_body() && !body->has_joint()) {
      throw runtime_error("ERROR: RigidBodyTree::compile(): Rigid body \"" +
                          body->get_name() + "\" in model " +
                          body->get_model_name() + " has no joint!");
    }

    if (body->has_parent_body() && body->get_spatial_inertia().isConstant(0)) {
      bool hasChild = false;
      for (size_t j = i + 1; j < bodies_.size(); ++j) {
        if (bodies_[j]->has_as_parent(*body)) {
          hasChild = true;
          break;
        }
      }
      if (!hasChild) {
        // now check if this body is attached by a loop joint
        for (const auto& loop : loops) {
          if ((loop.frameA_->has_as_rigid_body(body.get())) ||
              (loop.frameB_->has_as_rigid_body(body.get()))) {
            hasChild = true;
            break;
          }
        }
      }
      if (!hasChild) {
        if (print_weld_diasnostics_) {
          drake::log()->info("Welding joint {}", body->getJoint().get_name());
        } else {
          drake::log()->debug("Welding joint {}", body->getJoint().get_name());
        }
        std::unique_ptr<DrakeJoint> joint_unique_ptr(
            new FixedJoint(body->getJoint().get_name(),
                           body->getJoint().get_transform_to_parent_body()));
        body->setJoint(std::move(joint_unique_ptr));
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
  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
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
  for (size_t i = 0; i < bodies_.size(); ++i) {
    auto& body = bodies_[i];
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
  using drake::multibody::collision::bitmask;
  using drake::multibody::collision::Element;

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

  std::unordered_set<RigidBody<T>*> needs_recompile;

  // Update the collision filter groups for the *compiled* elements and mark
  // those bodies for collision filter notification.
  for (const std::pair<const RigidBody<T>* const, std::pair<bitmask, bitmask>>&
           item : collision_group_manager_.body_groups()) {
    RigidBody<T>& body = *get_mutable_body(item.first->get_body_index());
    const bitmask& group = item.second.first;
    const bitmask& ignore = item.second.second;
    for (auto itr = body.collision_elements_begin();
         itr != body.collision_elements_end(); ++itr) {
      (*itr)->merge_collision_filter(group, ignore);
    }
    needs_recompile.insert(&body);
  }

  // Set the collision filter group data for the uncompiled collision elements
  // based on the *current* state of the collision group manager and any
  // previously-compiled collision elements on the same bodies.
  for (auto& pair : body_collision_map_) {
    RigidBody<T>& body = *pair.first;
    bitmask group = collision_group_manager_.get_group_mask(body);
    bitmask ignore = collision_group_manager_.get_ignore_mask(body);

    // Set the collision filter groups for the *uncompiled* elements.
    BodyCollisions& elements = pair.second;
    for (const auto& collision_item : elements) {
      Element& element = *element_order_[collision_item.element];
      DRAKE_DEMAND(element.get_body() == &body);
      element.set_collision_filter(group, ignore);

      if (body.get_num_collision_elements() > 0) {
        // The body has already been compiled with collision elements. Include
        // previous collision filter groups into the element. This assumes that
        // all collision elements for a body must have *identical* collision
        // filter group info.
        const auto& compiled_element = *body.collision_elements_begin();
        element.merge_collision_filter(
            compiled_element->get_collision_filter_group(),
            compiled_element->get_collision_filter_ignores());
      }
    }
  }

  collision_group_manager_.Clear();

  // Builds cliques for collision filtering.
  CreateCollisionCliques(&needs_recompile);

  // Update all previously compiled bodies that have had their filters modified.
  for (RigidBody<T>* body_ptr : needs_recompile) {
    RigidBody<double>& body = *body_ptr;
    for (auto itr = body.collision_elements_begin();
         itr != body.collision_elements_end(); ++itr) {
      collision_model_->NotifyFilterCriteriaChanged((*itr)->getId());
    }
  }

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
void RigidBodyTree<T>::CreateCollisionCliques(
    std::unordered_set<RigidBody<T>*>* recompile_set) {
  using drake::multibody::collision::Element;
  // If a body with collision geometry is adjacent to another such body, their
  // collision geometries will *all* be added to a common clique. That means
  // if either body has *multiple* geometries, they are already excluded from
  // self-collision. Thus, process adjacency issues first and only add
  // self-collision cliques as necessary.
  //
  // This assumes that any previously compiled bodies and collision elements
  // are correctly configured. This merely updates to include the new data
  // in the body_collision_map_. Part of "correctness" currently requires
  // *every* collision element which belongs to a single body to have the *same*
  // collision cliques.
  //
  // In this algorithm, for a particular body B, there are two types of
  // collision elements: B_Q and B_C, the queued and compiled collision
  // elements, respectively (where B_Q and B_C are *sets*, possibly empty).
  // The process of updating the compiled state of the tree requires evaluating
  // the combination of bodies and their compiled and queued collision elements.

  for (auto& pair : body_collision_map_) {
    RigidBody<T>& body = *pair.first;

    // First, assign the elements B_Q to the same cliques as one of the elements
    // in B_C. The choice is irrelevant because all elements in B_C should have
    // the same cliques. NOTE: this *doesn't* address self-collisions!
    if (body.get_num_collision_elements() > 0) {
      BodyCollisions& collisions = pair.second;
      const Element& compiled_element = **body.collision_elements_begin();
      for (auto& item : collisions) {
        element_order_[item.element]->AddCliquesFromElement(compiled_element);
      }
    }

    const int num_B_C = body.get_num_collision_elements();
    // We need the chance to process this body against all other bodies -- but
    // we want to do it only once. Duplication is only a risk with bodies that
    // are also in body_collision_map_. So, we'll detect which are in the map
    // and skip those bodies with *smaller* indices guaranteeing that for
    // body pair (B, O) we'll only process them if B.index < O.index.
    for (int i = 1; i < static_cast<int>(bodies_.size()); ++i) {
      if (body_collision_map_.count(bodies_[i].get()) > 0 &&
          i <= body.get_body_index()) {
        continue;
      }

      RigidBody<T>& test_body = *bodies_[i];
      if (!test_body.CanCollideWith(body)) {
        // The action that needs to be taken depends on the sizes of B_Q, B_C,
        // O_Q, and O_C. Because we're in this loop, we know |B_Q| > 0. So, that
        // leaves us with 8 cases:
        // │ │|B_C|│|O_Q|│|O_C|│
        // ├─┼─────┼─────┼─────┤
        // │1│░░░0░│░░░0░│░░░0░│ Only new elements for B; no op.
        // │2│   0 │   0 │ > 0 │ Queued elements adjacent; add clique B, O.
        // │3│   0 │ > 0 │   0 │ Queued elements adjacent; add clique B, O.
        // │4│   0 │ > 0 │ > 0 │ Queued elements adjacent; add clique B, O.
        // │5│░>░0░│░░░0░│░░░0░│ Only new elements for B; no op.
        // │6│░>░0░│░░░0░│░>░0░│ Compiled elements already adjacent; no op.
        // │7│ > 0 │ > 0 │   0 │ Queued elements adjacent; add clique B, O.
        // │8│░>░0░│░>░0░│░>░0░│ Compiled elements already adjacent; no op.
        //
        // Notes:
        //  1. If I already had compiled collision elements on both bodies, then
        //     I already have cliques to handle them and those cliques get
        //     copied in the previous stage. No further action required.
        //     (rows 6 & 8)
        //  2. If there are no collision elements on O (queued or compiled),
        //     there is no adjacency that actually needs to be filtered. (rows 1
        //     & 5)
        //  3. In all other cases, new adjacency is being *added* (i.e.,
        //     previously there were no collision elements in one or both bodies
        //     and elements are being added that give *both* adjacent bodies
        //     collision elements.
        //  4. If clique set on a body with compiled elements are changed, that
        //     body needs to have its collision elements "recompiled".

        const int num_O_C = test_body.get_num_collision_elements();
        auto iter = body_collision_map_.find(&test_body);
        const int num_O_Q =
            iter == body_collision_map_.end()
                ? 0
                : static_cast<int>(body_collision_map_[&test_body].size());

        // Detect criteria for the no-op rows.
        if ((num_B_C == 0 && num_O_C == 0 && num_O_Q == 0) ||
            (num_B_C > 0 && (num_O_Q == 0 || num_O_C > 0))) {
          continue;
        }

        int clique_id = get_next_clique_id();

        // Add elements in B_Q to clique.
        for (const auto& item : body_collision_map_[&body]) {
          element_order_[item.element]->AddToCollisionClique(clique_id);
        }

        // Add elements in O_Q to clique.
        if (num_O_Q) {
          for (const auto& item : body_collision_map_[&test_body]) {
            element_order_[item.element]->AddToCollisionClique(clique_id);
          }
        }

        // Add elements in O_C to clique.
        if (num_O_C) {
          for (auto e_iter = test_body.collision_elements_begin();
               e_iter != test_body.collision_elements_end(); ++e_iter) {
            (*e_iter)->AddToCollisionClique(clique_id);
          }
          recompile_set->insert(&test_body);
        }

        // Add elements in B_C to clique.
        if (num_B_C) {
          for (auto e_iter = body.collision_elements_begin();
               e_iter != body.collision_elements_end(); ++e_iter) {
            (*e_iter)->AddToCollisionClique(clique_id);
          }
          recompile_set->insert(&body);
        }
      }
    }
  }

  // Now clean up self-collision. The queued collision elements (B_Q) have
  // had the chance to be assigned to cliques due to adjacency (or previous
  // cliques). We need self-collision cliques iff:
  //  1. the total number of cliques > 1. i.e., |B_C| + |B_Q| > 1, and
  //  2. the elements in B_Q haven't been assigned to any cliques yet
  //     (determined by examining the "first" element in B_Q).
  for (auto& pair : body_collision_map_) {
    RigidBody<T>& body = *pair.first;
    BodyCollisions& body_collisions = pair.second;
    Element& element = *element_order_[body_collisions[0].element];
    if (element.get_num_cliques() == 0) {
      int num_B_Q = static_cast<int>(body_collisions.size());
      int num_B_C = body.get_num_collision_elements();
      if (num_B_Q + num_B_C > 1) {
        // This element hasn't been assigned to *any* cliques but _now_ requires
        // one.
        int clique_id = get_next_clique_id();
        for (const BodyCollisionItem& item : body_collisions) {
          element_order_[item.element]->AddToCollisionClique(clique_id);
        }
        if (num_B_C) {
          // If |B_C| > 1, then it should already have a clique.
          DRAKE_DEMAND(num_B_C == 1);
          for (auto e_iter = body.collision_elements_begin();
               e_iter != body.collision_elements_end(); ++e_iter) {
            (*e_iter)->AddToCollisionClique(clique_id);
          }
          recompile_set->insert(&body);
        }
      }
    }
  }
}

template <typename T>
Eigen::VectorXd RigidBodyTree<T>::getZeroConfiguration() const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies_) {
    if (body_ptr->has_parent_body()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(body_ptr->get_position_start_index(),
                   joint.get_num_positions()) = joint.zeroConfiguration();
    }
  }
  return q;
}

template <typename T>
Eigen::VectorXd RigidBodyTree<T>::getRandomConfiguration(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator) const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies_) {
    if (body_ptr->has_parent_body()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(body_ptr->get_position_start_index(),
                   joint.get_num_positions()) =
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
  while (body_index + 1 < bodies_.size() &&
         bodies_[body_index + 1]->get_position_start_index() <= position_num)
    body_index++;

  return bodies_[body_index]->getJoint().get_position_name(
      position_num - bodies_[body_index]->get_position_start_index());
}

template <typename T>
string RigidBodyTree<T>::get_velocity_name(int velocity_num) const {
  if (velocity_num < 0 || velocity_num >= num_velocities_)
    throw std::runtime_error("velocity_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies_.size() &&
         bodies_[body_index + 1]->get_velocity_start_index() <= velocity_num)
    body_index++;

  return bodies_[body_index]->getJoint().get_velocity_name(
      velocity_num - bodies_[body_index]->get_velocity_start_index());
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
  for (const auto& body : bodies_) {
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
  for (const auto& frame : frames_) {
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
    const drake::multibody::collision::Element& element, RigidBody<T>& body,
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
          "Unable to add the collision element to the body: " +
          body.get_name());
    }
  }

  BodyCollisions& body_collisions = itr->second;
  size_t id = element_order_.size();
  element_order_.emplace_back(
      std::unique_ptr<drake::multibody::collision::Element>(element.clone()));
  element_order_.back()->set_body(&body);
  body_collisions.emplace_back(group_name, id);
}

template <typename T>
template <typename U>
void RigidBodyTree<T>::updateCollisionElements(
    const RigidBody<T>& body,
    const Eigen::Transform<U, 3, Eigen::Isometry>& transform_to_world,
    bool throw_if_missing_gradient) {
  for (auto id_iter = body.get_collision_element_ids().begin();
       id_iter != body.get_collision_element_ids().end(); ++id_iter) {
    if (throw_if_missing_gradient) {
      collision_model_->UpdateElementWorldTransform(
          *id_iter, drake::math::DiscardZeroGradient(transform_to_world));
    } else {
      collision_model_->UpdateElementWorldTransform(
          *id_iter, drake::math::DiscardGradient(transform_to_world));
    }
  }
}

template <typename T>
template <typename U>
void RigidBodyTree<T>::updateDynamicCollisionElements(
    const KinematicsCache<U>& cache, bool throw_if_missing_gradient) {
  CheckCacheValidity(cache);
  // todo: this is currently getting called many times with the same cache
  // object.  and it's presumably somewhat expensive.
  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      updateCollisionElements(
          body, cache.get_element(body.get_body_index()).transform_to_world,
          throw_if_missing_gradient);
    }
  }
  collision_model_->UpdateModel();
}

template <typename T>
void RigidBodyTree<T>::getTerrainContactPoints(
    const RigidBody<T>& body, Eigen::Matrix3Xd* terrain_points,
    const std::string& group_name) const {
  // Ensures terrain_points is a valid pointer.
  DRAKE_DEMAND(terrain_points);

  // Clears matrix before filling it again.
  size_t num_points = 0;
  terrain_points->resize(Eigen::NoChange, 0);

  vector<drake::multibody::collision::ElementId> element_ids;

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
  for (auto id_iter = element_ids.begin(); id_iter != element_ids.end();
       ++id_iter) {
    Matrix3Xd element_points;
    collision_model_->GetTerrainContactPoints(*id_iter, &element_points);
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

  vector<drake::multibody::collision::PointPair<double>> closest_points;

  collision_model_->CollisionDetectFromPoints(points, use_margins,
                                              &closest_points);
  x.resize(3, closest_points.size());
  body_x.resize(3, closest_points.size());
  normal.resize(3, closest_points.size());
  phi.resize(closest_points.size());
  body_idx.resize(closest_points.size());

  for (size_t i = 0; i < closest_points.size(); ++i) {
    x.col(i) = closest_points[i].ptB;
    body_x.col(i) = closest_points[i].ptA;
    normal.col(i) = closest_points[i].normal;
    phi[i] = closest_points[i].distance;
    const drake::multibody::collision::Element* elementB =
        closest_points[i].elementB;
    // In the case that no closest point was found, elementB will come back
    // as a null pointer which we should not attempt to access.
    if (elementB) {
      body_idx[i] = elementB->get_body()->get_body_index();
    } else {
      body_idx[i] = -1;
    }
  }
}

template <typename T>
bool RigidBodyTree<T>::collisionRaycast(
    const KinematicsCache<double>& cache, const Matrix3Xd& origins,
    const Matrix3Xd& ray_endpoints,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& distances, bool use_margins) {
  Matrix3Xd normals;
  updateDynamicCollisionElements(cache);
  return collision_model_->CollisionRaycast(origins, ray_endpoints, use_margins,
                                            &distances, &normals);
}

template <typename T>
bool RigidBodyTree<T>::collisionRaycast(
    const KinematicsCache<double>& cache, const Matrix3Xd& origins,
    const Matrix3Xd& ray_endpoints,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    VectorXd& distances, Matrix3Xd& normals, bool use_margins) {
  updateDynamicCollisionElements(cache);
  return collision_model_->CollisionRaycast(origins, ray_endpoints, use_margins,
                                            &distances, &normals);
}

template <typename T>
bool RigidBodyTree<T>::collisionDetect(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    vector<int>& bodyB_idx,
    const vector<drake::multibody::collision::ElementId>& ids_to_check,
    bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<drake::multibody::collision::PointPair<double>> points;
  bool points_found = collision_model_->ClosestPointsAllToAll(
      ids_to_check, use_margins, &points);

  xA = MatrixXd::Zero(3, points.size());
  xB = MatrixXd::Zero(3, points.size());
  normal = MatrixXd::Zero(3, points.size());
  phi = VectorXd::Zero(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    xA.col(i) = points[i].ptA;
    xB.col(i) = points[i].ptB;
    normal.col(i) = points[i].normal;
    phi[i] = points[i].distance;
    const drake::multibody::collision::Element* elementA = points[i].elementA;
    bodyA_idx.push_back(elementA->get_body()->get_body_index());
    const drake::multibody::collision::Element* elementB = points[i].elementB;
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
  vector<drake::multibody::collision::ElementId> ids_to_check;
  for (const int& body_idx : bodies_idx) {
    if (body_idx >= 0 && body_idx < static_cast<int>(bodies_.size())) {
      for (const string& group : active_element_groups) {
        bodies_[body_idx]->appendCollisionElementIdsFromThisBody(group,
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
  vector<drake::multibody::collision::ElementId> ids_to_check;
  for (const int& body_idx : bodies_idx) {
    if (body_idx >= 0 && body_idx < static_cast<int>(bodies_.size())) {
      bodies_[body_idx]->appendCollisionElementIdsFromThisBody(ids_to_check);
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
    vector<int>& bodyB_idx, const set<string>& active_element_groups,
    bool use_margins) {
  CheckCacheValidity(cache);
  vector<drake::multibody::collision::ElementId> ids_to_check;
  for (auto body_iter = bodies_.begin(); body_iter != bodies_.end();
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
  vector<drake::multibody::collision::ElementId> ids_to_check;
  for (auto body_iter = bodies_.begin(); body_iter != bodies_.end();
       ++body_iter) {
    (*body_iter)->appendCollisionElementIdsFromThisBody(ids_to_check);
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

template <typename T>
template <typename U>
std::vector<drake::multibody::collision::PointPair<U>>
RigidBodyTree<T>::ComputeMaximumDepthCollisionPoints(
    const KinematicsCache<U>& cache, bool use_margins,
    bool throw_if_missing_gradient) {
  updateDynamicCollisionElements(cache);
  vector<drake::multibody::collision::PointPair<double>> contact_points;
  collision_model_->ComputeMaximumDepthCollisionPoints(use_margins,
                                                       &contact_points);

  vector<drake::multibody::collision::PointPair<U>>
      contact_points_in_body_frame;

  // For each contact pair, map contact point from world frame to each body's
  // frame.
  for (size_t i = 0; i < contact_points.size(); ++i) {
    auto& pair = contact_points[i];
    if (pair.elementA->CanCollideWith(pair.elementB)) {
      // For the autodiff version, throw if collision gradients are being
      // arbitrarily set to zero.  Intelligent callers (that are careful to
      // handle the gradients) may disable this with the throw_if_missing
      // argument.
      if (!std::is_same<U, double>::value && throw_if_missing_gradient) {
        std::runtime_error(
            "Potential collisions exist, but the collision engine does not "
            "support autodiff.  Gradients would have been inaccurate.");
      }

      drake::multibody::collision::PointPair<U> pair_in_body_frame(pair);

      // Get bodies' transforms.
      const int bodyA_id = pair.elementA->get_body()->get_body_index();
      const Isometry3<U>& TA = cache.get_element(bodyA_id).transform_to_world;

      const int bodyB_id = pair.elementB->get_body()->get_body_index();
      const Isometry3<U>& TB = cache.get_element(bodyB_id).transform_to_world;

      // Transform to bodies' frames.
      // Note: Eigen assumes aliasing by default and therefore this operation
      // is safe.
      pair_in_body_frame.ptA = TA.inverse() * contact_points[i].ptA.cast<U>();
      pair_in_body_frame.ptB = TB.inverse() * contact_points[i].ptB.cast<U>();

      // TODO(russt): Shouldn't the normal be transformed, too?

      contact_points_in_body_frame.push_back(pair_in_body_frame);
    }
  }
  return contact_points_in_body_frame;
}

template <typename T>
bool RigidBodyTree<T>::collidingPointsCheckOnly(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model_->CollidingPointsCheckOnly(points,
                                                    collision_threshold);
}

template <typename T>
vector<size_t> RigidBodyTree<T>::collidingPoints(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model_->CollidingPoints(points, collision_threshold);
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

  vector<drake::multibody::collision::PointPair<double>> points;
  bool points_found = collision_model_->ComputeMaximumDepthCollisionPoints(
      use_margins, &points);

  xA_in_world = Matrix3Xd::Zero(3, points.size());
  xB_in_world = Matrix3Xd::Zero(3, points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    xA_in_world.col(i) = points[i].ptA;
    xB_in_world.col(i) = points[i].ptB;

    const drake::multibody::collision::Element* elementA = points[i].elementA;
    bodyA_idx.push_back(elementA->get_body()->get_body_index());
    const drake::multibody::collision::Element* elementB = points[i].elementB;
    bodyB_idx.push_back(elementB->get_body()->get_body_index());
  }
  return points_found;
}

template <typename T>
template <typename Scalar>
void RigidBodyTree<T>::CheckCacheValidity(
    const KinematicsCache<Scalar>& cache) const {
  if (cache.get_num_cache_elements() != get_num_bodies()) {
    throw std::runtime_error(
        "RigidBodyTree::CheckCacheValidity: Number of "
        "cache elements (" +
        std::to_string(cache.get_num_cache_elements()) +
        ") does not equal the number of bodies in the RigidBodyTree (" +
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
        throw std::runtime_error(
            "RigidBodyTree::CheckCacheValidity: Cache "
            "element " +
            std::to_string(i) + " for joint " + joint.get_name() +
            " has incorrect number of joint positions or velocities.\n" +
            "  - num positions: cache has " +
            std::to_string(cache_element.get_num_positions()) + ", joint has " +
            std::to_string(joint.get_num_positions()) +
            "\n"
            "  - num velocities: cache has " +
            std::to_string(cache.get_num_velocities()) + ", joint has " +
            std::to_string(joint.get_num_velocities()));
      }
    }
  }
}

template <typename T>
KinematicsCache<T> RigidBodyTree<T>::CreateKinematicsCacheFromBodiesVector(
    const std::vector<std::unique_ptr<RigidBody<T>>>& bodies) {
  std::vector<int> num_joint_positions, num_joint_velocities;
  for (const auto& body : bodies) {
    int np = body->has_parent_body() ? body->getJoint().get_num_positions() : 0;
    int nv =
        body->has_parent_body() ? body->getJoint().get_num_velocities() : 0;
    num_joint_positions.push_back(np);
    num_joint_velocities.push_back(nv);
  }
  const int num_positions = std::accumulate(num_joint_positions.begin(),
                                            num_joint_positions.end(), 0);
  const int num_velocities = std::accumulate(num_joint_positions.begin(),
                                             num_joint_positions.end(), 0);
  KinematicsCache<T> cache(num_positions, num_velocities, num_joint_positions,
                           num_joint_velocities);
  return cache;
}

template <typename T>
template <typename CacheT>
KinematicsCache<CacheT> RigidBodyTree<T>::CreateKinematicsCacheWithType()
    const {
  DRAKE_DEMAND(initialized_ &&
               "This RigidBodyTree was not initialized."
               " RigidBodyTree::compile() must be called first.");
  std::vector<int> num_joint_positions;
  std::vector<int> num_joint_velocities;
  for (const auto& body_unique_ptr : bodies_) {
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
  DRAKE_DEMAND(initialized_ &&
               "This RigidBodyTree was not initialized."
               " RigidBodyTree::compile() must be called first.");
  return CreateKinematicsCacheWithType<T>();
}

#endif
#if DRAKE_RBT_SHARD == 1

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

  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    RigidBody<T>& body = *bodies_[i];
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

#endif
#if DRAKE_RBT_SHARD == 0

template <typename T>
template <typename Scalar>
void RigidBodyTree<T>::updateCompositeRigidBodyInertias(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false,
                                      "updateCompositeRigidBodyInertias");

  if (!cache.areInertiasCached()) {
    for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
      const RigidBody<T>& body = **it;
      auto element = cache.get_mutable_element(body.get_body_index());
      element->inertia_in_world = transformSpatialInertia(
          element->transform_to_world,
          body.get_spatial_inertia().template cast<Scalar>());
      element->crb_in_world = element->inertia_in_world;
    }

    // N.B. Reverse iteration.
    for (auto it = bodies_.rbegin(); it != bodies_.rend(); ++it) {
      const RigidBody<T>& body = **it;
      if (body.has_parent_body()) {
        const auto element = cache.get_mutable_element(body.get_body_index());
        auto parent_element =
            cache.get_mutable_element(body.get_parent()->get_body_index());
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

  for (const auto& body : bodies_) {
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
  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      const auto& element = cache.get_element(body.get_body_index());
      const DrakeJoint& joint = body.getJoint();
      int ncols_joint = in_terms_of_qdot ? joint.get_num_positions()
                                         : joint.get_num_velocities();
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
    KinematicsCache<Scalar>& cache,
    const std::set<int>& model_instance_id_set) const {
  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(true, true,
                                      "worldMomentumMatrixDotTimesV");
  updateCompositeRigidBodyInertias(cache);

  TwistVector<Scalar> ret;
  ret.setZero();
  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
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
  auto ret =
      worldMomentumMatrix(cache, model_instance_id_set, in_terms_of_qdot);

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
    KinematicsCache<Scalar>& cache,
    const std::set<int>& model_instance_id_set) const {
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
  for (const auto& body : bodies_) {
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

  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    RigidBody<T>& body = *bodies_[i];
    if (is_part_of_model_instances(body, model_instance_id_set)) {
      if (body.get_mass() > 0) {
        com.noalias() +=
            body.get_mass() *
            transformPoints(
                cache, body.get_center_of_mass().template cast<Scalar>(), i, 0);
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
  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    const RigidBody<T>& body = *bodies_[i];
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
template <typename Derived>
drake::VectorX<typename Derived::Scalar>
RigidBodyTree<T>::transformVelocityToQDot(
    const KinematicsCache<typename Derived::Scalar>& cache,
    const Eigen::MatrixBase<Derived>& v) {
  VectorX<typename Derived::Scalar> qdot(cache.get_num_positions());
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
template <typename Derived>
drake::VectorX<typename Derived::Scalar>
RigidBodyTree<T>::transformQDotToVelocity(
    const KinematicsCache<typename Derived::Scalar>& cache,
    const Eigen::MatrixBase<Derived>& qdot) {
  VectorX<typename Derived::Scalar> v(cache.get_num_velocities());
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
      cache, MatrixX<Scalar>::Identity(cache.get_num_positions(),
                                       cache.get_num_positions()));
}

template <typename T>
template <typename Scalar>
MatrixX<Scalar> RigidBodyTree<T>::GetQDotToVelocityMapping(
    const KinematicsCache<Scalar>& cache) {
  return transformVelocityMappingToQDotMapping(
      cache, MatrixX<Scalar>::Identity(cache.get_num_velocities(),
                                       cache.get_num_velocities()));
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, kSpaceDimension, Eigen::Dynamic>
RigidBodyTree<T>::centerOfMassJacobian(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    KinematicsCache<Scalar>& cache, const std::set<int>& model_instance_id_set,
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
  if (nb == 0) nb = bodies_.size();
  set<int>::iterator iter = body_idx.begin();
  for (size_t i = 0; i < nb; ++i) {
    if (body_idx.size() == 0)
      bi = i;
    else
      bi = *iter++;
    num_contacts += bodies_[bi]->get_contact_points().cols();
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
    if (frame_ind >= static_cast<int>(frames_.size())) {
      std::ostringstream stream;
      stream << "Got a frame ind greater than available!\n";
      throw std::runtime_error(stream.str());
    }
    body_ind = frames_[frame_ind]->get_rigid_body().get_body_index();

    if (Tframe) {
      (*Tframe) =
          frames_[frame_ind]->get_transform_to_body().template cast<Scalar>();
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
void
RigidBodyTree<T>::FindAncestorBodies(int body_index,
                                     std::vector<int>* ancestor_bodies) const {
  // Verifies that body_index is valid. Aborts if it is invalid.
  DRAKE_DEMAND(body_index >= 0 &&
               body_index < static_cast<int>(bodies_.size()));
  DRAKE_DEMAND(ancestor_bodies != nullptr);

  ancestor_bodies->clear();
  const RigidBody<T>* current_body = bodies_[body_index].get();
  while (current_body->has_parent_body()) {
    ancestor_bodies->push_back(current_body->get_parent()->get_body_index());
    current_body = current_body->get_parent();
  }
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindAncestorBodies(int body_index) const {
  std::vector<int> ancestor_bodies;
  FindAncestorBodies(body_index, &ancestor_bodies);
  return ancestor_bodies;
}

template <typename T>
KinematicPath RigidBodyTree<T>::findKinematicPath(
    int start_body_or_frame_idx, int end_body_or_frame_idx) const {
  KinematicPath path;
  std::vector<int> start_body_ancestors;
  std::vector<int> end_body_ancestors;
  FindKinematicPath(start_body_or_frame_idx, end_body_or_frame_idx,
                    &start_body_ancestors, &end_body_ancestors, &path);
  return path;
}

template <typename T>
void RigidBodyTree<T>::FindKinematicPath(int start_body_or_frame_idx,
                                         int end_body_or_frame_idx,
                                         std::vector<int>* start_body_ancestors,
                                         std::vector<int>* end_body_ancestors,
                                         KinematicPath* path) const {
  DRAKE_DEMAND(start_body_ancestors != nullptr);
  DRAKE_DEMAND(end_body_ancestors != nullptr);
  DRAKE_DEMAND(path != nullptr);

  // find all ancestors of start_body and end_body
  int start_body = parseBodyOrFrameID(start_body_or_frame_idx);

  start_body_ancestors->clear();
  FindAncestorBodies(start_body, start_body_ancestors);
  start_body_ancestors->insert(start_body_ancestors->begin(), start_body);

  int end_body = parseBodyOrFrameID(end_body_or_frame_idx);
  end_body_ancestors->clear();
  FindAncestorBodies(end_body, end_body_ancestors);
  end_body_ancestors->insert(end_body_ancestors->begin(), end_body);

  // find least common ancestor
  size_t common_size =
      std::min(start_body_ancestors->size(), end_body_ancestors->size());
  bool least_common_ancestor_found = false;
  std::vector<int>::iterator start_body_lca_it =
      start_body_ancestors->end() - common_size;
  std::vector<int>::iterator end_body_lca_it =
      end_body_ancestors->end() - common_size;

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
    stream << "There is no path between " << bodies_[start_body]->get_name()
           << " and " << bodies_[end_body]->get_name() << ".";
    throw std::runtime_error(stream.str());
  }
  int least_common_ancestor = *start_body_lca_it;

  // compute path
  path->joint_path.clear();
  path->joint_direction_signs.clear();
  path->body_path.clear();

  std::vector<int>::iterator it = start_body_ancestors->begin();
  for (; it != start_body_lca_it; ++it) {
    path->joint_path.push_back(*it);
    path->joint_direction_signs.push_back(-1);
    path->body_path.push_back(*it);
  }

  path->body_path.push_back(least_common_ancestor);

  std::vector<int>::reverse_iterator reverse_it(end_body_lca_it);
  for (; reverse_it != end_body_ancestors->rend(); ++reverse_it) {
    path->joint_path.push_back(*reverse_it);
    path->joint_direction_signs.push_back(1);
    path->body_path.push_back(*reverse_it);
  }
}

#endif
#if DRAKE_RBT_SHARD == 1

template <typename T>
template <typename Scalar>
TwistMatrix<Scalar> RigidBodyTree<T>::geometricJacobian(
    const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
    int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind,
    bool in_terms_of_qdot, std::vector<int>* v_or_qdot_indices) const {
  TwistMatrix<Scalar> J(kTwistSize, in_terms_of_qdot ? get_num_positions()
                            : get_num_velocities());
  std::vector<int> indices;
  GeometricJacobian(cache, base_body_or_frame_ind,
                    end_effector_body_or_frame_ind,
                    expressed_in_body_or_frame_ind, in_terms_of_qdot,
                    v_or_qdot_indices != nullptr ? v_or_qdot_indices :
                    &indices, &J);
  return J.block(0, 0, kTwistSize, v_or_qdot_indices != nullptr ?
                v_or_qdot_indices->size() : indices.size());
}

namespace {

template <typename Scalar>
void TransformJacobian(
  const Eigen::Transform<Scalar, 3, Eigen::Isometry>& transform,
  drake::Matrix6X<Scalar>* J) {
    DRAKE_DEMAND(J != nullptr);

    // This block of computation is similar to transformSpatialMotion but J
    // could have more than 6 columns, so we do not call transformSpatialMotion
    // directly. A for loop is used here instead of matrix operations to
    // prevent a dynamically sized temporary from being allocated.
    for (int col = 0; col < J->cols(); ++col) {
      J->col(col).template head<3>() =
        transform.linear() * J->col(col).template head<3>();
      J->col(col).template tail<3>() =
        transform.linear() * J->col(col).template tail<3>();
      J->col(col).template tail<3>().noalias() +=
         -J->col(col).template head<3>().cross(transform.translation());
    }
}

}  // namespace

template <typename T>
template <typename Scalar>
void RigidBodyTree<T>::GeometricJacobian(const KinematicsCache<Scalar>& cache,
                                         int base_body_or_frame_ind,
                                         int end_effector_body_or_frame_ind,
                                         int expressed_in_body_or_frame_ind,
                                         bool in_terms_of_qdot,
                                         std::vector<int>* v_or_q_indices,
                                         Matrix6X<Scalar>* J) const {
  DRAKE_DEMAND(J != nullptr);

  CheckCacheValidity(cache);
  cache.checkCachedKinematicsSettings(false, false, "GeometricJacobian");

  KinematicPath& kinematic_path = cache.geometric_jacobian_temp.kinematic_path;
  std::vector<int>& start_body_ancestors =
      cache.geometric_jacobian_temp.start_body_ancestors;
  std::vector<int>& end_body_ancestors =
      cache.geometric_jacobian_temp.end_body_ancestors;

  FindKinematicPath(base_body_or_frame_ind, end_effector_body_or_frame_ind,
                    &start_body_ancestors, &end_body_ancestors,
                    &kinematic_path);

  int cols = 0;
  int body_index;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); ++i) {
    body_index = kinematic_path.joint_path[i];
    const RigidBody<T>& body = *bodies_[body_index];
    const DrakeJoint& joint = body.getJoint();
    cols += in_terms_of_qdot ? joint.get_num_positions()
                             : joint.get_num_velocities();
  }

  DRAKE_DEMAND(J->cols() >= cols);
  J->setZero();

  if (v_or_q_indices != nullptr) {
    v_or_q_indices->clear();
    v_or_q_indices->reserve(cols);
  }

  int col_start = 0;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); ++i) {
    body_index = kinematic_path.joint_path[i];
    RigidBody<T>& body = *bodies_[body_index];
    const auto& element = cache.get_element(body.get_body_index());
    const DrakeJoint& joint = body.getJoint();
    int ncols_block = in_terms_of_qdot ? joint.get_num_positions()
                                       : joint.get_num_velocities();
    int sign = kinematic_path.joint_direction_signs[i];
    auto J_block = J->template block<kTwistSize, Dynamic>(
        0, col_start, kTwistSize, ncols_block);
    if (in_terms_of_qdot) {
      J_block.noalias() =
          sign * element.motion_subspace_in_world * element.qdot_to_v;
    } else {
      J_block.noalias() = sign * element.motion_subspace_in_world;
    }

    if (v_or_q_indices != nullptr) {
      int cols_block_start = in_terms_of_qdot ? body.get_position_start_index()
                                              : body.get_velocity_start_index();
      for (int j = 0; j < ncols_block; ++j) {
        v_or_q_indices->push_back(cols_block_start + j);
      }
    }
    col_start += ncols_block;
  }

  if (expressed_in_body_or_frame_ind != 0) {
    auto T_world_to_frame =
        relativeTransform(cache, expressed_in_body_or_frame_ind, 0);
    TransformJacobian(T_world_to_frame, J);
  }
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
  const auto& end_effector_element = cache.get_element(end_effector_body_ind);

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

#endif
#if DRAKE_RBT_SHARD == 0

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

  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    RigidBody<T>& body_i = *bodies_[i];
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
  Matrix6X<Scalar> body_accelerations(kTwistSize, bodies_.size());
  Matrix6X<Scalar> net_wrenches(kTwistSize, bodies_.size());
  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    const RigidBody<T>& body = *bodies_[i];
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
  for (int i = static_cast<int>(bodies_.size()) - 1; i >= 0; --i) {
    RigidBody<T>& body = *bodies_[i];
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

  // SpringTorques added with a negative sign since they are being included in
  // the bias terms
  torques -= CalcGeneralizedSpringForces(cache.getQ());

  return torques;
}

template <typename T>
template <typename DerivedV>
Matrix<typename DerivedV::Scalar, Dynamic, 1> RigidBodyTree<T>::frictionTorques(
    Eigen::MatrixBase<DerivedV> const& v) const {
  typedef typename DerivedV::Scalar Scalar;
  Matrix<Scalar, Dynamic, 1> ret(num_velocities_, 1);

  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
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
template <typename Scalar>
VectorX<Scalar> RigidBodyTree<T>::CalcGeneralizedSpringForces(
    const VectorX<Scalar>& q) const {
  VectorX<Scalar> generalized_force(num_velocities_);

  for (auto it = bodies_.begin(); it != bodies_.end(); ++it) {
    const RigidBody<T>& body = **it;
    if (body.has_parent_body()) {
      const DrakeJoint& joint = body.getJoint();
      int nv_joint = joint.get_num_velocities();
      int v_start_joint = body.get_velocity_start_index();
      int nq_joint = joint.get_num_positions();
      int q_start_joint = body.get_position_start_index();

      auto q_body = q.middleRows(q_start_joint, nq_joint);

      // generalized spring forces each depend on the joint position, but are
      // in velocity coordinates
      generalized_force.middleRows(v_start_joint, nv_joint) =
          joint.SpringTorque(q_body);
    }
  }
  return generalized_force;
}

#endif
#if DRAKE_RBT_SHARD == 1

template <typename T>
template <typename Scalar, typename PointScalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::DoTransformPointsJacobian(
    const KinematicsCache<Scalar>& cache,
    const Eigen::Ref<const Matrix3X<PointScalar>>& points,
    int from_body_or_frame_ind, int to_body_or_frame_ind,
    bool in_terms_of_qdot) const {
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
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind, bool in_terms_of_qdot) const {
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

#endif
#if DRAKE_RBT_SHARD == 0

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::forwardKinPositionGradient(
    const KinematicsCache<Scalar>& cache, int npoints,
    int from_body_or_frame_ind, int to_body_or_frame_ind) const {
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

#endif
#if DRAKE_RBT_SHARD == 1

template <typename T>
template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::DoTransformPointsJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache,
    const Eigen::Ref<const Eigen::Matrix3Xd>& points,
    int from_body_or_frame_ind, int to_body_or_frame_ind) const {
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

#endif
#if DRAKE_RBT_SHARD == 0

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

  for (int i = 0; i < static_cast<int>(bodies_.size()); ++i) {
    // Skips the current body if model_instance_id is not -1 and the body's
    // robot ID is not equal to the desired model instance ID.
    if (model_instance_id != -1 &&
        model_instance_id != bodies_[i]->get_model_instance_id()) {
      continue;
    }

    // Obtains a lower case version of the current body's model name.
    string current_model_name = bodies_[i]->get_model_name();
    std::transform(current_model_name.begin(), current_model_name.end(),
                   current_model_name.begin(), ::tolower);

    // Skips the current body if model_name is not empty and the body's model
    // name is not equal to the desired model name.
    if (!model_name_lower.empty() && model_name_lower != current_model_name)
      continue;

    // Obtains a lower case version of the current body's name.
    string current_body_name = bodies_[i]->get_name();
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
    return bodies_[match_index].get();
  } else {
    throw std::logic_error(
        "RigidBodyTree::FindBody: ERROR: Could not find body named \"" +
        body_name + "\", model name = \"" + model_name +
        "\", model instance id = " + std::to_string(model_instance_id) + ".");
  }
}

template <typename T>
const RigidBody<double>* RigidBodyTree<T>::FindBody(
    drake::multibody::collision::ElementId element_id) const {
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
std::vector<const RigidBody<T>*> RigidBodyTree<T>::FindModelInstanceBodies(
    int model_instance_id) const {
  std::vector<const RigidBody<T>*> result;

  for (const auto& rigid_body : bodies_) {
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

  for (int i = 0; i < static_cast<int>(frames_.size()); ++i) {
    // Skips the current frame if model_instance_id is not -1 and the frame's
    // model instance ID is not equal to the desired model instance ID.
    if (model_instance_id != -1 &&
        model_instance_id != frames_[i]->get_model_instance_id()) {
      continue;
    }

    // Obtains a lower case version of the current frame.
    std::string current_frame_name = frames_[i]->get_name();
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
            "RigidBodyTree::findFrame: ERROR: Found multiple frames named \"" +
            frame_name +
            "\", model_instance_id = " + std::to_string(model_instance_id));
      }
    }
  }

  // Checks if a match was found. If so, returns a pointer to the matching
  // frame. Otherwise, throws an exception indicating no match was found.
  if (match_index >= 0) {
    return frames_[match_index];
  } else {
    throw std::logic_error(
        "RigidBodyTree::findFrame: ERROR: could not find frame named \"" +
        frame_name +
        "\", model instance id = " + std::to_string(model_instance_id) + ".");
  }
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindBaseBodies(int model_instance_id) const {
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
        body_name +
        "\", model_instance_id = " + std::to_string(model_instance_id) + ".");
  }
  return body->get_body_index();
}

template <typename T>
std::vector<int> RigidBodyTree<T>::FindChildrenOfBody(
    int parent_body_index, int model_instance_id) const {
  // Verifies that parameter parent_body_index is valid.
  DRAKE_DEMAND(parent_body_index >= 0 && parent_body_index < get_num_bodies());

  // Obtains a reference to the parent body.
  const RigidBody<T>& parent_body = get_body(parent_body_index);

  // Checks every rigid body in this tree. If the rigid body is a child of
  // parent_body and its model instance ID matches model_instance_id, save its
  // index in the result vector.
  std::vector<int> children_indexes;
  for (int ii = 0; ii < static_cast<int>(bodies_.size()); ++ii) {
    if (bodies_.at(ii)->has_as_parent(parent_body)) {
      if (model_instance_id != -1) {
        if (bodies_.at(ii)->get_model_instance_id() == model_instance_id) {
          children_indexes.push_back(ii);
        }
      } else {
        children_indexes.push_back(ii);
      }
    }
  }
  return children_indexes;
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::FindChildBodyOfJoint(
    const std::string& joint_name, int model_instance_id) const {
  // Obtains a lower case version of joint_name.
  std::string joint_name_lower = joint_name;
  std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                 joint_name_lower.begin(), ::tolower);

  vector<bool> name_match;
  name_match.resize(bodies_.size());

  // For each rigid body in this RigidBodyTree, the following code saves a
  // `true` or `false` in vector `name_match` based on whether the body's parent
  // joint's name matches @p joint_name.
  for (size_t i = 0; i < bodies_.size(); ++i) {
    if (bodies_[i]->has_parent_body()) {
      // Obtains the name of the rigid body's parent joint and then converts it
      // to be lower case.
      std::string current_joint_name = bodies_[i]->getJoint().get_name();
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
    for (size_t i = 0; i < bodies_.size(); ++i) {
      if (name_match[i]) {
        name_match[i] =
            (bodies_[i]->get_model_instance_id() == model_instance_id);
      }
    }
  }

  // Checks to ensure only one match was found. Throws an `std::runtime_error`
  // if more than one match was found.
  size_t ind_match = 0;
  bool match_found = false;
  for (size_t i = 0; i < bodies_.size(); ++i) {
    if (name_match[i]) {
      if (match_found) {
        throw std::runtime_error(
            "RigidBodyTree::FindChildBodyOfJoint: ERROR: Multiple joints "
            "found named \"" +
            joint_name + "\", model instance ID = " +
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
        "joint named \"" +
        joint_name +
        "\", model_instance_id = " + std::to_string(model_instance_id) + ".");
  } else {
    return bodies_[ind_match].get();
  }
}

template <typename T>
int RigidBodyTree<T>::FindIndexOfChildBodyOfJoint(const std::string& joint_name,
                                                  int model_instance_id) const {
  RigidBody<T>* link = FindChildBodyOfJoint(joint_name, model_instance_id);
  return link->get_body_index();
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::get_mutable_body(int body_index) {
  DRAKE_DEMAND(body_index >= 0 && body_index < get_num_bodies());
  return bodies_[body_index].get();
}

template <typename T>
int RigidBodyTree<T>::get_num_frames() const {
  return static_cast<int>(frames_.size());
}

template <typename T>
std::string RigidBodyTree<T>::getBodyOrFrameName(int body_or_frame_id) const {
  if (body_or_frame_id >= 0) {
    return bodies_[body_or_frame_id]->get_name();
  } else if (body_or_frame_id < -1) {
    return frames_[-body_or_frame_id - 2]->get_name();
  } else {
    return "COM";
  }
}

template <typename T>
void RigidBodyTree<T>::addDistanceConstraint(int bodyA_index_in,
                                             const Eigen::Vector3d& r_AP_in,
                                             int bodyB_index_in,
                                             const Eigen::Vector3d& r_BQ_in,
                                             double distance_in) {
  distance_constraints.emplace_back(bodyA_index_in, r_AP_in, bodyB_index_in,
                                    r_BQ_in, distance_in);
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree<T>::positionConstraints(
    const KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, 1> ret(
      6 * loops.size() + distance_constraints.size(), 1);
  for (size_t i = 0; i < loops.size(); ++i) {
    {  // position constraint
      auto ptA_in_B = transformPoints(cache, Vector3<Scalar>::Zero(),
                                      loops[i].frameA_->get_frame_index(),
                                      loops[i].frameB_->get_frame_index());
      ret.template middleRows<3>(6 * i) = ptA_in_B;
    }
    {  // second position constraint (to constrain orientation)
      auto axis_A_end_in_B =
          transformPoints(cache, loops[i].axis_.template cast<Scalar>(),
                          loops[i].frameA_->get_frame_index(),
                          loops[i].frameB_->get_frame_index());
      ret.template middleRows<3>(6 * i + 3) = axis_A_end_in_B - loops[i].axis_;
    }
  }
  // Relative Distance Constraint
  // Constraint is of the form: f = |x| - d* where x is the distance between
  // the two points and d* is the constrained distance.
  for (size_t i = 0; i < distance_constraints.size(); ++i) {
    // Compute the displacement from point Q (a point on body B) to point P (a
    // point on body A).
    Eigen::Matrix<Scalar, 3, 1> r_QP_B =
        transformPoints(cache,
                        distance_constraints[i].r_AP.template cast<Scalar>(),
                        distance_constraints[i].bodyA_index,
                        distance_constraints[i].bodyB_index) -
        distance_constraints[i].r_BQ.template cast<Scalar>();
    ret(6 * loops.size() + i) =
        r_QP_B.norm() - distance_constraints[i].distance;
  }
  return ret;
}

#endif
#if DRAKE_RBT_SHARD == 1

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree<T>::positionConstraintsJacobian(
    const KinematicsCache<Scalar>& cache, bool in_terms_of_qdot) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(
      6 * loops.size() + distance_constraints.size(),
      in_terms_of_qdot ? num_positions_ : num_velocities_);

  for (size_t i = 0; i < loops.size(); ++i) {
    // position constraint
    ret.template middleRows<3>(6 * i) = transformPointsJacobian(
        cache, Vector3d::Zero(), loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index(), in_terms_of_qdot);
    // second position constraint (to constrain orientation)
    ret.template middleRows<3>(6 * i + 3) = transformPointsJacobian(
        cache, loops[i].axis_.template cast<Scalar>(),
        loops[i].frameA_->get_frame_index(),
        loops[i].frameB_->get_frame_index(), in_terms_of_qdot);
  }
  // Relative Distance Constraint
  // Jacobian of the constraint is: ∂f/∂q = (xᵀ J) / |x| where J = ∂x/∂q
  for (size_t i = 0; i < distance_constraints.size(); ++i) {
    // Compute the displacement from point Q (a point on body B) to point P (a
    // point on body A).
    Eigen::Matrix<Scalar, 3, 1> r_QP_B =
        transformPoints(cache,
                        distance_constraints[i].r_AP.template cast<Scalar>(),
                        distance_constraints[i].bodyA_index,
                        distance_constraints[i].bodyB_index) -
        distance_constraints[i].r_BQ.template cast<Scalar>();
    auto J = transformPointsJacobian(
        cache, distance_constraints[i].r_AP.template cast<Scalar>(),
        distance_constraints[i].bodyA_index,
        distance_constraints[i].bodyB_index, in_terms_of_qdot);
    ret.template middleRows<1>(6 * loops.size() + i) =
        (r_QP_B.transpose() / r_QP_B.norm()) * J;
  }
  return ret;
}

template <typename T>
template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree<T>::positionConstraintsJacDotTimesV(
    const KinematicsCache<Scalar>& cache) const {
  CheckCacheValidity(cache);
  Matrix<Scalar, Eigen::Dynamic, 1> ret(
      6 * loops.size() + distance_constraints.size(), 1);
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
  // Relative Distance Constraint
  // JacobianDotTimesV of the constraint is derived as:
  // d(∂f/∂q)/dt * v
  // = d((xᵀ J) / |x|)/dt * v
  // = xᵀ/|x| * d(J)/dt * v + d(xᵀ)/dt * J*v/|x| + xᵀ*J*v * d(1/|x|)/dt
  // = xᵀJ̇v/|x| + ẋᵀJv/|x| - ẋᵀxxᵀJv/|x|^3
  for (size_t i = 0; i < distance_constraints.size(); ++i) {
    // Compute the displacement from point Q (a point on body B) to point P (a
    // point on body A).
    Eigen::Matrix<Scalar, 3, 1> r_QP_B =
        transformPoints(cache,
                        distance_constraints[i].r_AP.template cast<Scalar>(),
                        distance_constraints[i].bodyA_index,
                        distance_constraints[i].bodyB_index) -
        distance_constraints[i].r_BQ.template cast<Scalar>();
    auto J = transformPointsJacobian(
        cache, distance_constraints[i].r_AP.template cast<Scalar>(),
        distance_constraints[i].bodyA_index,
        distance_constraints[i].bodyB_index, false);
    auto Jdotv =
        transformPointsJacobianDotTimesV(cache, distance_constraints[i].r_AP,
                                         distance_constraints[i].bodyA_index,
                                         distance_constraints[i].bodyB_index);
    const VectorX<Scalar> v_QP_B = J * cache.getV();
    auto norm_r_QP_B = r_QP_B.norm();
    ret.template middleRows<1>(6 * loops.size() + i) =
        r_QP_B.transpose() * Jdotv / norm_r_QP_B +
        v_QP_B.transpose() * v_QP_B / norm_r_QP_B -
        v_QP_B.transpose() * r_QP_B * r_QP_B.transpose() * v_QP_B /
            pow(norm_r_QP_B, 3);
  }
  return ret;
}

#endif
#if DRAKE_RBT_SHARD == 0

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
  return loops.size() * 6 + distance_constraints.size();
}

namespace {

std::string strlower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

}  // namespace

template <typename T>
void RigidBodyTree<T>::addFrame(std::shared_ptr<RigidBodyFrame<T>> frame) {
  // Ensure there are no duplicates.
  const std::string model_name =
      strlower(frame->get_rigid_body().get_model_name());
  // TODO(eric.cousineau): Provide RigidBodyFrame::get_model_name(), incorporate
  // this into `findFrame`.
  for (auto& other : frames_) {
    const std::string other_model_name =
        strlower(other->get_rigid_body().get_model_name());
    if (other->get_name() == frame->get_name()) {
      if (other_model_name == model_name &&
          other->get_model_instance_id() == frame->get_model_instance_id()) {
        throw std::runtime_error(fmt::format(
            "Frame '{}', with model instance id {} ('{}'), already "
            "registered!",
            frame->get_name(), frame->get_model_instance_id(), model_name));
      }
    }
  }

  frames_.push_back(frame);
  // yuck!!
  frame->set_frame_index(-(static_cast<int>(frames_.size()) - 1) - 2);
}

template <typename T>
RigidBody<T>* RigidBodyTree<T>::add_rigid_body(
    std::unique_ptr<RigidBody<T>> body) {
  // Ensure there are no duplicates.
  const std::string model_name = strlower(body->get_model_name());
  for (auto& other : bodies_) {
    if (other->get_name() == body->get_name()) {
      if (strlower(other->get_model_name()) == model_name &&
          other->get_model_instance_id() == body->get_model_instance_id()) {
        throw std::runtime_error(fmt::format(
            "Body '{}', with model instance id {} ('{}'), already "
            "registered!",
            body->get_name(), body->get_model_instance_id(), model_name));
      }
    }
  }

  // Create a default frame for the given body.
  // N.B. We must add the frame here before the body is registered, so that we
  // can quickly fail if there is a duplicate frame (transaction integrity).
  auto body_frame =
      make_shared<RigidBodyFrame<T>>(body->get_name(), body.get());
  addFrame(body_frame);

  // TODO(amcastro-tri): body indexes should not be initialized here but on an
  // initialize call after all bodies and RigidBodySystem's are defined.
  // This initialize call will make sure that all global and local indexes are
  // properly computed taking into account a RigidBodySystem could be part of a
  // larger RigidBodySystem (a system within a tree of systems).
  body->set_body_index(static_cast<int>(bodies_.size()));

  // bodies will be sorted by SortTree by generation. Therefore bodies[0]
  // (world) will be at the top and subsequent generations of children will
  // follow.
  bodies_.push_back(std::move(body));
  return bodies_.back().get();
}

template <typename T>
int RigidBodyTree<T>::add_model_instance() {
  return num_model_instances_++;
}

template <typename T>
Isometry3<T> RigidBodyTree<T>::CalcFramePoseInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF) const {
  cache.checkCachedKinematicsSettings(false, false,
                                      "CalcFramePoseInWorldFrame");

  const auto& body_element = cache.get_element(body.get_body_index());
  const Isometry3<T> X_WB = body_element.transform_to_world;
  return X_WB * X_BF;
}

template <typename T>
Vector6<T> RigidBodyTree<T>::CalcBodySpatialVelocityInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body) const {
  cache.checkCachedKinematicsSettings(true, false,
                                      "CalcBodySpatialVelocityInWorldFrame");

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
  Vector6<T> V_WB = CalcBodySpatialVelocityInWorldFrame(cache, body);

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

template <typename T>
drake::Matrix6X<T>
RigidBodyTree<T>::CalcFrameSpatialVelocityJacobianInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF, bool in_terms_of_qdot) const {
  Matrix6X<T> J(kTwistSize, in_terms_of_qdot ? get_num_positions() :
                get_num_velocities());
  CalcFrameSpatialVelocityJacobianInWorldFrame(cache, body, X_BF,
                                               in_terms_of_qdot, &J);
  return J;
}

template <typename T>
void RigidBodyTree<T>::CalcFrameSpatialVelocityJacobianInWorldFrame(
    const KinematicsCache<T>& cache, const RigidBody<T>& body,
    const drake::Isometry3<T>& X_BF, bool in_terms_of_qdot,
    Matrix6X<T>* J_WF) const {
  DRAKE_DEMAND(J_WF != nullptr);
  const int world_index = world().get_body_index();
  const int num_col =
      in_terms_of_qdot ? get_num_positions() : get_num_velocities();
  DRAKE_DEMAND(J_WF->cols() == num_col);

  drake::Vector3<T> p_WF =
      CalcFramePoseInWorldFrame(cache, body, X_BF).translation();

  std::vector<int>& v_or_q_indices =
    cache.spatial_velocity_jacobian_temp.v_or_q_indices;

  // J_WBwo is the Jacobian of the spatial velocity of frame Bwo measured
  // and expressed in the world frame, where Bwo is rigidly attached to B and
  // instantaneously coincides with the world frame.
  auto& J_WBwo = in_terms_of_qdot ?
                    cache.spatial_velocity_jacobian_temp.J_positions :
                    cache.spatial_velocity_jacobian_temp.J_velocities;
  GeometricJacobian(cache, world_index, body.get_body_index(), world_index,
                    in_terms_of_qdot, &v_or_q_indices, &J_WBwo);

  J_WF->setZero();
  int col = 0;
  for (int idx : v_or_q_indices) {
    // Angular velocity stays the same.
    J_WF->col(idx) = J_WBwo.col(col);
    // Linear velocity needs an additional cross product term.
    J_WF->col(idx).template tail<3>() +=
        J_WBwo.col(col).template head<3>().cross(p_WF);
    col++;
  }
}

template <typename T>
drake::Vector6<T>
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

// clang-format off

#endif
#if DRAKE_RBT_SHARD == 0

// N.B. The following order of instantiations is reflected in
// `rigid_body_tree_py.cc`. If you change this order, also update this binding
// file for traceability purposes.

// Explicit template instantiations for massMatrix.
template MatrixX<AutoDiffXd     > RigidBodyTree<double>::massMatrix<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&) const;  // NOLINT
template MatrixXd                 RigidBodyTree<double>::massMatrix<double         >(KinematicsCache<double         >&) const;  // NOLINT

// Explicit template instantiations for centerOfMass.
template Vector3<AutoDiffXd     > RigidBodyTree<double>::centerOfMass<AutoDiffXd     >(const KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT
template Vector3d                 RigidBodyTree<double>::centerOfMass<double         >(const KinematicsCache<double         >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT

// Explicit template instantiations for transformVelocityToQDot.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::transformVelocityToQDot<VectorX<AutoDiffXd     >>(const KinematicsCache<AutoDiffXd     >&, const Eigen::MatrixBase<VectorX<AutoDiffXd     >>&);  // NOLINT
template Eigen::VectorXd          RigidBodyTree<double>::transformVelocityToQDot<Eigen::VectorXd         >(const KinematicsCache<double         >&, const Eigen::MatrixBase<Eigen::VectorXd         >&);  // NOLINT

// Explicit template instantiations for transformQDotToVelocity.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::transformQDotToVelocity<VectorX<AutoDiffXd     >>(const KinematicsCache<AutoDiffXd     >&, const Eigen::MatrixBase<VectorX<AutoDiffXd     >>&);  // NOLINT
template Eigen::VectorXd          RigidBodyTree<double>::transformQDotToVelocity<Eigen::VectorXd         >(const KinematicsCache<double         >&, const Eigen::MatrixBase<Eigen::VectorXd         >&);  // NOLINT

// Explicit template instantiations for GetVelocityToQDotMapping.
template MatrixX<AutoDiffXd     > RigidBodyTree<double>::GetVelocityToQDotMapping(const KinematicsCache<AutoDiffXd     >&);  // NOLINT
template MatrixX<double         > RigidBodyTree<double>::GetVelocityToQDotMapping(const KinematicsCache<double         >&);  // NOLINT

// Explicit template instantiations for GetQDotToVelocityMapping
template MatrixX<AutoDiffXd     > RigidBodyTree<double>::GetQDotToVelocityMapping(const KinematicsCache<AutoDiffXd     >&);  // NOLINT
template MatrixX<double         > RigidBodyTree<double>::GetQDotToVelocityMapping(const KinematicsCache<double         >&);  // NOLINT

// Explicit template instantiations for dynamicsBiasTerm.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::dynamicsBiasTerm<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, unordered_map<RigidBody<double> const*, WrenchVector<AutoDiffXd     >, hash<RigidBody<double> const*>, equal_to<RigidBody<double> const*>, Eigen::aligned_allocator<pair<RigidBody<double> const* const, WrenchVector<AutoDiffXd     >>>> const&, bool) const;  // NOLINT
template VectorXd                 RigidBodyTree<double>::dynamicsBiasTerm<double         >(KinematicsCache<double         >&, unordered_map<RigidBody<double> const*, WrenchVector<double         >, hash<RigidBody<double> const*>, equal_to<RigidBody<double> const*>, Eigen::aligned_allocator<pair<RigidBody<double> const* const, WrenchVector<double         >>>> const&, bool) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for geometricJacobian.
template TwistMatrix<AutoDiffXd     > RigidBodyTree<double>::geometricJacobian<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, int, bool, vector<int, allocator<int>>*) const;  // NOLINT
template TwistMatrix<double         > RigidBodyTree<double>::geometricJacobian<double         >(KinematicsCache<double         > const&, int, int, int, bool, vector<int, allocator<int>>*) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for relativeTransform.
template Eigen::Transform<AutoDiffXd     , 3, 1, 0> RigidBodyTree<double>::relativeTransform<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int) const;  // NOLINT
template Eigen::Transform<double         , 3, 1, 0> RigidBodyTree<double>::relativeTransform<double         >(KinematicsCache<double         > const&, int, int) const;  // NOLINT

// Explicit template instantiations for centerOfMassJacobian.
template Matrix3X<AutoDiffXd     > RigidBodyTree<double>::centerOfMassJacobian<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT
template Matrix3Xd                 RigidBodyTree<double>::centerOfMassJacobian<double         >(KinematicsCache<double         >&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT

// Explicit template instantiations for centroidalMomentumMatrix.
template TwistMatrix<AutoDiffXd     > RigidBodyTree<double>::centroidalMomentumMatrix<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT
template TwistMatrix<double         > RigidBodyTree<double>::centroidalMomentumMatrix<double>(KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT

// Explicit template instantiations for forwardKinPositionGradient.
template MatrixX<AutoDiffXd     > RigidBodyTree<double>::forwardKinPositionGradient<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, int) const;  // NOLINT
template MatrixXd                 RigidBodyTree<double>::forwardKinPositionGradient<double         >(KinematicsCache<double         > const&, int, int, int) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for geometricJacobianDotTimesV.
template TwistVector<AutoDiffXd     > RigidBodyTree<double>::geometricJacobianDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, int) const;  // NOLINT
template TwistVector<double         > RigidBodyTree<double>::geometricJacobianDotTimesV<double         >(KinematicsCache<double         > const&, int, int, int) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for centerOfMassJacobianDotTimesV.
template Vector3<AutoDiffXd     > RigidBodyTree<double>::centerOfMassJacobianDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT
template Vector3d                 RigidBodyTree<double>::centerOfMassJacobianDotTimesV<double         >(KinematicsCache<double         >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT

// Explicit template instantiations for centroidalMomentumMatrixDotTimesV.
template TwistVector<AutoDiffXd     > RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT
template TwistVector<double         > RigidBodyTree<double>::centroidalMomentumMatrixDotTimesV<double         >(KinematicsCache<double         >&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT

// Explicit template instantiations for positionConstraints.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::positionConstraints<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&) const;  // NOLINT
template VectorXd                 RigidBodyTree<double>::positionConstraints<double         >(KinematicsCache<double         > const&) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for positionConstraintsJacobian.
template MatrixX<AutoDiffXd     > RigidBodyTree<double>::positionConstraintsJacobian<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, bool) const;  // NOLINT
template MatrixXd                 RigidBodyTree<double>::positionConstraintsJacobian<double         >(KinematicsCache<double         > const&, bool) const;  // NOLINT

// Explicit template instantiations for positionConstraintsJacDotTimesV.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::positionConstraintsJacDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&) const;  // NOLINT
template VectorXd                 RigidBodyTree<double>::positionConstraintsJacDotTimesV<double         >(KinematicsCache<double         > const&) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for jointLimitConstriants.
template void RigidBodyTree<double>::jointLimitConstraints<VectorXd            , VectorXd            , MatrixXd            >(Eigen::MatrixBase<VectorXd            > const&, Eigen::MatrixBase<VectorXd            >&, Eigen::MatrixBase<MatrixXd            >&) const;  // NOLINT
template void RigidBodyTree<double>::jointLimitConstraints<Eigen::Map<VectorXd>, Eigen::Map<VectorXd>, Eigen::Map<MatrixXd>>(Eigen::MatrixBase<Eigen::Map<VectorXd>> const&, Eigen::MatrixBase<Eigen::Map<VectorXd>>&, Eigen::MatrixBase<Eigen::Map<MatrixXd>>&) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for relativeTwist.
template TwistVector<AutoDiffXd     > RigidBodyTree<double>::relativeTwist<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, int) const;  // NOLINT
template TwistVector<double         > RigidBodyTree<double>::relativeTwist<double         >(KinematicsCache<double         > const&, int, int, int) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for worldMomentumMatrix.
template TwistMatrix<AutoDiffXd     > RigidBodyTree<double>::worldMomentumMatrix<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT
template TwistMatrix<double         > RigidBodyTree<double>::worldMomentumMatrix<double         >(KinematicsCache<double         >&, set<int, less<int>, allocator<int>> const&, bool) const;  // NOLINT

// Explicit template instantiations for worldMomentumMatrixDotTimesV.
template TwistVector<double> RigidBodyTree<double>::worldMomentumMatrixDotTimesV<double>(KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for transformSpatialAcceleration.
template TwistVector<double> RigidBodyTree<double>::transformSpatialAcceleration<double>(KinematicsCache<double> const&, TwistVector<double> const&, int, int, int, int) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for frictionTorques
template VectorX<AutoDiffXd     > RigidBodyTree<double>::frictionTorques(Eigen::MatrixBase<VectorX<AutoDiffXd     >> const& v) const;  // NOLINT
template VectorX<double         > RigidBodyTree<double>::frictionTorques(Eigen::MatrixBase<VectorX<double         >> const& v) const;  // NOLINT

// Explicit template instantiations for CalcGeneralizedSpringForces
template VectorX<double         > RigidBodyTree<double>::CalcGeneralizedSpringForces(const VectorX<double    >& q) const;  // NOLINT
template VectorX<AutoDiffXd     > RigidBodyTree<double>::CalcGeneralizedSpringForces(const VectorX<AutoDiffXd>& q) const;  // NOLINT

// Explicit template instantiations for inverseDynamics.
template VectorX<AutoDiffXd     > RigidBodyTree<double>::inverseDynamics<AutoDiffXd     >(KinematicsCache<AutoDiffXd     >&, unordered_map<RigidBody<double> const*, TwistVector<AutoDiffXd     >, hash<RigidBody<double> const*>, equal_to<RigidBody<double> const*>, Eigen::aligned_allocator<pair<RigidBody<double> const* const, TwistVector<AutoDiffXd     >>>> const&, VectorX<AutoDiffXd     > const&, bool) const;  // NOLINT
template VectorX<double         > RigidBodyTree<double>::inverseDynamics<double         >(KinematicsCache<double         >&, unordered_map<RigidBody<double> const*, WrenchVector<double        >, hash<RigidBody<double> const*>, equal_to<RigidBody<double> const*>, Eigen::aligned_allocator<pair<RigidBody<double> const* const, WrenchVector<double        >>>> const&, VectorX<double         > const&, bool) const;  // NOLINT

// Explicit template instantiations for resolveCenterOfPressure.
template pair<Vector3d, double> RigidBodyTree<double>::resolveCenterOfPressure<Vector3d, Vector3d>(KinematicsCache<double> const&, vector<ForceTorqueMeasurement, allocator<ForceTorqueMeasurement>> const&, Eigen::MatrixBase<Vector3d> const&, Eigen::MatrixBase<Vector3d> const&) const;  // NOLINT

// Explicit template instantiations for transformVelocityMappingToQDotMapping.
template MatrixX<double>     RigidBodyTree<double>::transformVelocityMappingToQDotMapping<VectorXd           >(const KinematicsCache<double>&,     const Eigen::MatrixBase<VectorXd           >&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformVelocityMappingToQDotMapping<Eigen::RowVectorXd >(const KinematicsCache<double>&,     const Eigen::MatrixBase<Eigen::RowVectorXd >&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformVelocityMappingToQDotMapping<MatrixXd           >(const KinematicsCache<double>&,     const Eigen::MatrixBase<MatrixXd           >&);  // NOLINT
template MatrixX<AutoDiffXd> RigidBodyTree<double>::transformVelocityMappingToQDotMapping<MatrixX<AutoDiffXd>>(const KinematicsCache<AutoDiffXd>&, const Eigen::MatrixBase<MatrixX<AutoDiffXd>>&);  // NOLINT

// Explicit template instantiations for transformQDotMappingToVelocityMapping.
template MatrixX<double>     RigidBodyTree<double>::transformQDotMappingToVelocityMapping<VectorXd                  >(const KinematicsCache<double>&,     const Eigen::MatrixBase<VectorXd                  >&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformQDotMappingToVelocityMapping<Eigen::RowVectorXd        >(const KinematicsCache<double>&,     const Eigen::MatrixBase<Eigen::RowVectorXd        >&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformQDotMappingToVelocityMapping<MatrixXd                  >(const KinematicsCache<double>&,     const Eigen::MatrixBase<MatrixXd                  >&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformQDotMappingToVelocityMapping<Eigen::Map<MatrixXd const>>(const KinematicsCache<double>&,     const Eigen::MatrixBase<Eigen::Map<MatrixXd const>>&);  // NOLINT
template MatrixX<double>     RigidBodyTree<double>::transformQDotMappingToVelocityMapping<Eigen::Map<MatrixXd      >>(const KinematicsCache<double>&,     const Eigen::MatrixBase<Eigen::Map<MatrixXd      >>&);  // NOLINT
template MatrixX<AutoDiffXd> RigidBodyTree<double>::transformQDotMappingToVelocityMapping<MatrixX<AutoDiffXd       >>(const KinematicsCache<AutoDiffXd>&, const Eigen::MatrixBase<MatrixX<AutoDiffXd       >>&);  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for DoTransformPointsJacobian.
template MatrixX<double    > RigidBodyTree<double>::DoTransformPointsJacobian<double    , double    >(const KinematicsCache<double    >&, const Eigen::Ref<const Matrix3X<double    >>&, int, int, bool) const;  // NOLINT
template MatrixX<AutoDiffXd> RigidBodyTree<double>::DoTransformPointsJacobian<AutoDiffXd, double    >(const KinematicsCache<AutoDiffXd>&, const Eigen::Ref<const Matrix3X<double    >>&, int, int, bool) const;  // NOLINT
template MatrixX<AutoDiffXd> RigidBodyTree<double>::DoTransformPointsJacobian<AutoDiffXd, AutoDiffXd>(const KinematicsCache<AutoDiffXd>&, const Eigen::Ref<const Matrix3X<AutoDiffXd>>&, int, int, bool) const;  // NOLINT

// Explicit template instantiations for DoTransformPointsJacobianDotTimesV.
template VectorX<AutoDiffXd>      RigidBodyTree<double>::DoTransformPointsJacobianDotTimesV<AutoDiffXd     >(const KinematicsCache<AutoDiffXd     >&, const Eigen::Ref<const Matrix3Xd>&, int, int) const;  // NOLINT
template VectorX<double>          RigidBodyTree<double>::DoTransformPointsJacobianDotTimesV<double         >(const KinematicsCache<double         >&, const Eigen::Ref<const Matrix3Xd>&, int, int) const;  // NOLINT

// Explicit template instantiations for relativeQuaternionJacobian.
template Matrix4Xd                 RigidBodyTree<double>::relativeQuaternionJacobian<double         >(KinematicsCache<double         > const&, int, int, bool) const;  // NOLINT
template Matrix4X<AutoDiffXd     > RigidBodyTree<double>::relativeQuaternionJacobian<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, bool) const;  // NOLINT

// Explicit template instantiations for relativeRollPitchYawJacobian.
template Matrix3Xd                 RigidBodyTree<double>::relativeRollPitchYawJacobian<double         >(KinematicsCache<double         > const&, int, int, bool) const;  // NOLINT
template Matrix3X<AutoDiffXd     > RigidBodyTree<double>::relativeRollPitchYawJacobian<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int, bool) const;  // NOLINT

// Explicit template instantiations for relativeRollPitchYawJacobianDotTimesV.
template VectorXd                 RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<double         >(KinematicsCache<double         > const&, int, int) const;  // NOLINT
template VectorX<AutoDiffXd     > RigidBodyTree<double>::relativeRollPitchYawJacobianDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int) const;  // NOLINT

// Explicit template instantiations for relativeQuaternionJacobianDotTimesV.
template VectorXd                 RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<double         >(KinematicsCache<double         > const&, int, int) const;  // NOLINT
template VectorX<AutoDiffXd     > RigidBodyTree<double>::relativeQuaternionJacobianDotTimesV<AutoDiffXd     >(KinematicsCache<AutoDiffXd     > const&, int, int) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for CheckCacheValidity(cache).
template void RigidBodyTree<double>::CheckCacheValidity(const KinematicsCache<double         >&) const;  // NOLINT
template void RigidBodyTree<double>::CheckCacheValidity(const KinematicsCache<AutoDiffXd     >&) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 1

// Explicit template instantiations for doKinematics(cache).
template void RigidBodyTree<double>::doKinematics(KinematicsCache<double         >&, bool) const;  // NOLINT
template void RigidBodyTree<double>::doKinematics(KinematicsCache<AutoDiffXd     >&, bool) const;  // NOLINT

// Explicit template instantiations for doKinematics(q).
template KinematicsCache<double    > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<VectorXd                                  > const&) const;  // NOLINT
template KinematicsCache<double    > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Block<MatrixXd const, -1, 1, true >> const&) const;  // NOLINT
template KinematicsCache<double    > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Block<MatrixXd      , -1, 1, true >> const&) const;  // NOLINT
template KinematicsCache<double    > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Map<VectorXd                      >> const&) const;  // NOLINT
template KinematicsCache<AutoDiffXd> RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<VectorX<AutoDiffXd                       >> const&) const;  // NOLINT
template KinematicsCache<double    > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Block<VectorXd      , -1, 1, false>> const&) const;  // NOLINT

// Explicit template instantiations for doKinematics(q, v).
template KinematicsCache<double>          RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<VectorXd                                  > const&, Eigen::MatrixBase<VectorXd                                  > const&, bool) const;  // NOLINT
template KinematicsCache<double>          RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&, Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&, bool) const;  // NOLINT
template KinematicsCache<double>          RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Block<VectorXd      , -1, 1, false>> const&, Eigen::MatrixBase<Eigen::Block<VectorXd      , -1, 1, false>> const&, bool) const;  // NOLINT
template KinematicsCache<AutoDiffXd     > RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<VectorX<AutoDiffXd                       >> const&, Eigen::MatrixBase<VectorX<AutoDiffXd                       >> const&, bool) const;  // NOLINT
template KinematicsCache<double>          RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Map<VectorXd                      >> const&, Eigen::MatrixBase<Eigen::Map<VectorXd                      >> const&, bool) const;  // NOLINT
template KinematicsCache<double>          RigidBodyTree<double>::doKinematics(Eigen::MatrixBase<Eigen::Map<VectorXd const                >> const&, Eigen::MatrixBase<Eigen::Map<VectorXd const                >> const&, bool) const;  // NOLINT

#endif
#if DRAKE_RBT_SHARD == 0

// Explicit template instantiations for CreateKinematicsCacheWithType.
template KinematicsCache<AutoDiffXd     > RigidBodyTree<double>::CreateKinematicsCacheWithType<AutoDiffXd     >() const;  // NOLINT

// Explicit template instantiations for ComputeMaximumDepthCollisionPoints.
template std::vector<drake::multibody::collision::PointPair<AutoDiffXd     >> RigidBodyTree<double>::ComputeMaximumDepthCollisionPoints<AutoDiffXd     >(const KinematicsCache<AutoDiffXd     >&, bool, bool);  // NOLINT
template std::vector<drake::multibody::collision::PointPair<double         >> RigidBodyTree<double>::ComputeMaximumDepthCollisionPoints<double         >(const KinematicsCache<double         >&, bool, bool);  // NOLINT

// clang-format on

// Explicitly instantiates on the most common scalar types.
template class RigidBodyTree<double>;

#endif
