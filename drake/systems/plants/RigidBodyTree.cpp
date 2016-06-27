#include "drake/systems/plants/RigidBodyTree.h"

#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"
#include "drake/systems/plants/joints/DrakeJoints.h"
#include "drake/systems/plants/joints/FixedJoint.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/util/drakeUtil.h"

#include <algorithm>
#include <limits>
#include <string>
#include "KinematicsCache.h"

#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

using drake::AutoDiffUpTo73d;
using drake::AutoDiffXd;
using drake::Matrix3X;
using drake::Matrix4X;
using drake::MatrixX;
using drake::Vector3;
using drake::VectorX;

using drake::math::autoDiffToGradientMatrix;
using drake::math::Gradient;

/// A column vector consisting of one twist.
template <typename Scalar>
using TwistVector = Eigen::Matrix<Scalar, TWIST_SIZE, 1>;

/// A matrix with one twist per column, and dynamically many columns.
template <typename Scalar>
using TwistMatrix = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic>;

const set<int> RigidBodyTree::default_robot_num_set = {0};
const char* const RigidBodyTree::kWorldLinkName = "world";

template <typename T>
void getFiniteIndexes(T const& v, std::vector<int>& finite_indexes) {
  finite_indexes.clear();
  const size_t n = v.size();
  for (int x = 0; x < n; x++) {
    if (std::isfinite(static_cast<double>(v[x]))) {
      finite_indexes.push_back(x);
    }
  }
}

std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj) {
  os << "loop connects pt "
     << obj.frameA->transform_to_body.matrix().topRightCorner(3, 1).transpose()
     << " on " << obj.frameA->body->name_ << " to pt "
     << obj.frameB->transform_to_body.matrix().topRightCorner(3, 1).transpose()
     << " on " << obj.frameB->body->name_ << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const RigidBodyTree& tree) {
  os << *tree.collision_model.get();
  return os;
}

RigidBodyTree::RigidBodyTree(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type)
    : RigidBodyTree() {
  addRobotFromURDF(urdf_filename, floating_base_type);
}

RigidBodyTree::RigidBodyTree(void)
    : collision_model(DrakeCollision::newModel()) {
  // Sets the gravity vector;
  a_grav << 0, 0, 0, 0, 0, -9.81;

  // Adds the rigid body representing the world.
  std::unique_ptr<RigidBody> b(new RigidBody());
  b->name_ = std::string(RigidBodyTree::kWorldLinkName);
  b->robotnum = 0;
  b->body_index = 0;
  bodies.push_back(std::move(b));
}

RigidBodyTree::~RigidBodyTree(void) {}

bool RigidBodyTree::transformCollisionFrame(
    const DrakeCollision::ElementId& eid,
    const Eigen::Isometry3d& transform_body_to_joint) {
  return collision_model->transformCollisionFrame(eid, transform_body_to_joint);
}

// TODO(amcastro-tri): This implementation is very inefficient since member
// vector RigidBodyTree::bodies changes in size with the calls to bodies.erase
// and bodies.insert.
// A possibility would be to use std::sort or our own version of a quick sort.
void RigidBodyTree::SortTree() {
  if (bodies.size() == 0) return;  // no-op if there are no RigidBody's
  size_t i = 0;
  while (i < bodies.size() - 1) {
    if (bodies[i]->hasParent()) {
      auto iter = std::find_if(bodies.begin() + i + 1, bodies.end(),
                               [&](std::unique_ptr<RigidBody> const& p) {
                                 return bodies[i]->has_as_parent(*p);
                               });
      if (iter != bodies.end()) {
        std::unique_ptr<RigidBody> parent = std::move(*iter);
        bodies.erase(iter);
        bodies.insert(bodies.begin() + i, std::move(parent));
        --i;
      }
    }
    ++i;
  }

  // Reasign body_index to be the i-th entry in RBT::bodies
  for (size_t i = 0; i < bodies.size(); i++) {
    bodies[i]->body_index = static_cast<int>(i);
  }
}

const RigidBodyActuator& RigidBodyTree::GetActuator(
    const std::string& name) const {
  for (const auto& actuator : actuators) {
    if (actuator.name == name) {
      return actuator;
    }
  }
  throw std::invalid_argument("ERROR: Could not find actuator named \"" + name +
                              "\"");
}

void RigidBodyTree::compile(void) {
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
  //   RigidBodyTree::downwards_body_iterator: travels the tree downwards from
  //   the root towards the last leaf.
  for (size_t i = 0; i < bodies.size(); i++) {
    if (bodies[i]->hasParent() && bodies[i]->I.isConstant(0)) {
      bool hasChild = false;
      for (size_t j = i + 1; j < bodies.size(); j++) {
        if (bodies[j]->has_as_parent(*bodies[i])) {
          hasChild = true;
          break;
        }
      }
      if (!hasChild) {
        // now check if this body is attached by a loop joint
        for (const auto& loop : loops) {
          if ((loop.frameA->body == bodies[i].get()) ||
              (loop.frameB->body == bodies[i].get())) {
            hasChild = true;
            break;
          }
        }
      }
      if (!hasChild) {
        cout << "Welding joint " << bodies[i]->getJoint().getName() << endl;
        std::unique_ptr<DrakeJoint> joint_unique_ptr(
            new FixedJoint(bodies[i]->getJoint().getName(),
                           bodies[i]->getJoint().getTransformToParentBody()));
        bodies[i]->setJoint(std::move(joint_unique_ptr));
      }
    }
  }

  // Counts the number of position and velocity states in this rigid body tree.
  // Notice that the rigid bodies are accessed from the sorted vector
  // RigidBodyTree::bodies. The order that they appear in this vector determines
  // the values of RigidBody::position_num_start and
  // RigidBody::velocity_num_start, which the following code sets.
  num_positions_ = 0;
  num_velocities_ = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      body.position_num_start = num_positions_;
      num_positions_ += body.getJoint().getNumPositions();
      body.velocity_num_start = num_velocities_;
      num_velocities_ += body.getJoint().getNumVelocities();
    } else {
      body.position_num_start = 0;
      body.velocity_num_start = 0;
    }
  }

  B.resize(num_velocities_, actuators.size());
  B = MatrixXd::Zero(num_velocities_, actuators.size());
  for (size_t ia = 0; ia < actuators.size(); ia++)
    for (int i = 0; i < actuators[ia].body->getJoint().getNumVelocities(); i++)
      B(actuators[ia].body->velocity_num_start + i, ia) =
          actuators[ia].reduction;

  // Initializes the joint limit vectors.
  joint_limit_min = VectorXd::Constant(
      num_positions_, -std::numeric_limits<double>::infinity());
  joint_limit_max = VectorXd::Constant(num_positions_,
                                       std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < bodies.size(); i++) {
    auto& body = bodies[i];
    if (body->hasParent()) {
      const DrakeJoint& joint = body->getJoint();
      joint_limit_min.segment(body->position_num_start,
                              joint.getNumPositions()) =
          joint.getJointLimitMin();
      joint_limit_max.segment(body->position_num_start,
                              joint.getNumPositions()) =
          joint.getJointLimitMax();
    }
  }

  // Updates the static collision elements and terrain contact points.
  updateStaticCollisionElements();

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    getTerrainContactPoints(body, body.contact_pts);
  }

  initialized_ = true;
}

Eigen::VectorXd RigidBodyTree::getZeroConfiguration() const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies) {
    if (body_ptr->hasParent()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(body_ptr->position_num_start, joint.getNumPositions()) =
          joint.zeroConfiguration();
    }
  }
  return q;
}

Eigen::VectorXd RigidBodyTree::getRandomConfiguration(
    std::default_random_engine& generator) const {
  Eigen::VectorXd q(num_positions_);
  for (const auto& body_ptr : bodies) {
    if (body_ptr->hasParent()) {
      const DrakeJoint& joint = body_ptr->getJoint();
      q.middleRows(body_ptr->position_num_start, joint.getNumPositions()) =
          joint.randomConfiguration(generator);
    }
  }
  return q;
}

string RigidBodyTree::getPositionName(int position_num) const {
  if (position_num < 0 || position_num >= num_positions_)
    throw std::runtime_error("position_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
         bodies[body_index + 1]->position_num_start <= position_num)
    body_index++;

  return bodies[body_index]->getJoint().getPositionName(
      position_num - bodies[body_index]->position_num_start);
}

string RigidBodyTree::getVelocityName(int velocity_num) const {
  if (velocity_num < 0 || velocity_num >= num_velocities_)
    throw std::runtime_error("velocity_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
         bodies[body_index + 1]->velocity_num_start <= velocity_num)
    body_index++;

  return bodies[body_index]->getJoint().getVelocityName(
      velocity_num - bodies[body_index]->velocity_num_start);
}

string RigidBodyTree::getStateName(int state_num) const {
  if (state_num < num_positions_)
    return getPositionName(state_num);
  else
    return getVelocityName(state_num - num_positions_);
}

void RigidBodyTree::drawKinematicTree(
    std::string graphviz_dotfile_filename) const {
  ofstream dotfile;
  dotfile.open(graphviz_dotfile_filename);
  dotfile << "digraph {" << endl;
  for (const auto& body : bodies) {
    dotfile << "  " << body->name_ << " [label=\"" << body->name_ << endl;
    dotfile << "mass=" << body->mass << ", com=" << body->com.transpose()
            << endl;
    dotfile << "inertia=" << endl << body->I << endl;
    dotfile << "\"];" << endl;
    if (body->hasParent()) {
      const auto& joint = body->getJoint();
      dotfile << "  " << body->name_ << " -> " << body->parent->name_
              << " [label=\"" << joint.getName() << endl;
      dotfile << "transform_to_parent_body=" << endl
              << joint.getTransformToParentBody().matrix() << endl;
      //     dotfile << "axis=" << endl << joint.get().matrix() << endl;
      dotfile << "\"];" << endl;
    }
  }
  for (const auto& frame : frames) {
    dotfile << "  " << frame->name << " [label=\"" << frame->name
            << " (frame)\"];" << endl;
    dotfile << "  " << frame->name << " -> " << frame->body->name_
            << " [label=\"";
    dotfile << "transform_to_body=" << endl
            << frame->transform_to_body.matrix() << endl;
    dotfile << "\"];" << endl;
  }

  for (const auto& loop : loops) {
    dotfile << "  " << loop.frameA->body->name_ << " -> "
            << loop.frameB->body->name_ << " [label=\"loop " << endl;
    dotfile << "transform_to_parent_body=" << endl
            << loop.frameA->transform_to_body.matrix() << endl;
    dotfile << "transform_to_child_body=" << endl
            << loop.frameB->transform_to_body.matrix() << endl;
    dotfile << "\"];" << endl;
  }
  dotfile << "}" << endl;
  dotfile.close();
  cout << "Wrote kinematic tree to " << graphviz_dotfile_filename
       << "; run e.g. 'dot -Tpng -O " << graphviz_dotfile_filename
       << "' to generate an image." << endl;
}

map<string, int> RigidBodyTree::computePositionNameToIndexMap() const {
  map<string, int> name_to_index_map;

  for (int i = 0; i < this->num_positions_; ++i) {
    name_to_index_map[getPositionName(i)] = i;
  }
  return name_to_index_map;
}

DrakeCollision::ElementId RigidBodyTree::addCollisionElement(
    const RigidBody::CollisionElement& element, RigidBody& body,
    const string& group_name) {
  DrakeCollision::ElementId id = collision_model->addElement(element);
  if (id != 0) {
    body.collision_element_ids.push_back(id);
    body.collision_element_groups[group_name].push_back(id);
  }
  return id;
}

void RigidBodyTree::updateCollisionElements(
    const RigidBody& body,
    const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world) {
  for (auto id_iter = body.collision_element_ids.begin();
       id_iter != body.collision_element_ids.end(); ++id_iter) {
    collision_model->updateElementWorldTransform(*id_iter, transform_to_world);
  }
}

void RigidBodyTree::updateStaticCollisionElements() {
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (!body.hasParent()) {
      updateCollisionElements(body, Isometry3d::Identity());
    }
  }
}

void RigidBodyTree::updateDynamicCollisionElements(
    const KinematicsCache<double>& cache) {
  // todo: this is currently getting called many times with the same cache
  // object.  and it's presumably somewhat expensive.
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    const RigidBody& body = **it;
    if (body.hasParent()) {
      updateCollisionElements(body, cache.getElement(body).transform_to_world);
    }
  }
  collision_model->updateModel();
}

void RigidBodyTree::getTerrainContactPoints(
    const RigidBody& body, Eigen::Matrix3Xd& terrain_points) const {
  // clear matrix before filling it again
  size_t num_points = 0;
  terrain_points.resize(Eigen::NoChange, 0);

  for (auto id_iter = body.collision_element_ids.begin();
       id_iter != body.collision_element_ids.end(); ++id_iter) {
    Matrix3Xd element_points;
    collision_model->getTerrainContactPoints(*id_iter, element_points);
    terrain_points.conservativeResize(
        Eigen::NoChange, terrain_points.cols() + element_points.cols());
    terrain_points.block(0, num_points, terrain_points.rows(),
                         element_points.cols()) = element_points;
    num_points += element_points.cols();
  }
}

void RigidBodyTree::collisionDetectFromPoints(
    const KinematicsCache<double>& cache, const Matrix3Xd& points,
    VectorXd& phi, Matrix3Xd& normal, Matrix3Xd& x, Matrix3Xd& body_x,
    vector<int>& body_idx, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> closest_points;

  collision_model->collisionDetectFromPoints(points, use_margins,
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
    body_idx.push_back(elementB->get_body()->body_index);
  }
}

bool RigidBodyTree::collisionRaycast(const KinematicsCache<double>& cache,
                                     const Matrix3Xd& origins,
                                     const Matrix3Xd& ray_endpoints,
                                     VectorXd& distances, bool use_margins) {
  Matrix3Xd normals;
  updateDynamicCollisionElements(cache);
  return collision_model->collisionRaycast(origins, ray_endpoints, use_margins,
                                           distances, normals);
}

bool RigidBodyTree::collisionRaycast(const KinematicsCache<double>& cache,
                                     const Matrix3Xd& origins,
                                     const Matrix3Xd& ray_endpoints,
                                     VectorXd& distances, Matrix3Xd& normals,
                                     bool use_margins) {
  updateDynamicCollisionElements(cache);
  return collision_model->collisionRaycast(origins, ray_endpoints, use_margins,
                                           distances, normals);
}

bool RigidBodyTree::collisionDetect(
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    vector<int>& bodyB_idx,
    const vector<DrakeCollision::ElementId>& ids_to_check, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  bool points_found =
      collision_model->closestPointsAllToAll(ids_to_check, use_margins, points);

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
    bodyA_idx.push_back(elementA->get_body()->body_index);
    const DrakeCollision::Element* elementB = points[i].elementB;
    bodyB_idx.push_back(elementB->get_body()->body_index);
  }
  return points_found;
}

bool RigidBodyTree::collisionDetect(
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    vector<int>& bodyB_idx, const vector<int>& bodies_idx,
    const set<string>& active_element_groups, bool use_margins) {
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_idx_iter = bodies_idx.begin();
       body_idx_iter != bodies_idx.end(); ++body_idx_iter) {
    if (*body_idx_iter >= 0 && *body_idx_iter < bodies.size()) {
      for (auto group_iter = active_element_groups.begin();
           group_iter != active_element_groups.end(); ++group_iter) {
        bodies[*body_idx_iter]->appendCollisionElementIdsFromThisBody(
            *group_iter, ids_to_check);
      }
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

bool RigidBodyTree::collisionDetect(
    const KinematicsCache<double>& cache, VectorXd& phi, Matrix3Xd& normal,
    Matrix3Xd& xA, Matrix3Xd& xB, vector<int>& bodyA_idx,
    vector<int>& bodyB_idx, const vector<int>& bodies_idx, bool use_margins) {
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_idx_iter = bodies_idx.begin();
       body_idx_iter != bodies_idx.end(); ++body_idx_iter) {
    if (*body_idx_iter >= 0 && *body_idx_iter < bodies.size()) {
      bodies[*body_idx_iter]->appendCollisionElementIdsFromThisBody(
          ids_to_check);
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

bool RigidBodyTree::collisionDetect(const KinematicsCache<double>& cache,
                                    VectorXd& phi, Matrix3Xd& normal,
                                    Matrix3Xd& xA, Matrix3Xd& xB,
                                    vector<int>& bodyA_idx,
                                    vector<int>& bodyB_idx,
                                    const set<string>& active_element_groups,
                                    bool use_margins) {
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

bool RigidBodyTree::collisionDetect(const KinematicsCache<double>& cache,
                                    VectorXd& phi, Matrix3Xd& normal,
                                    Matrix3Xd& xA, Matrix3Xd& xB,
                                    vector<int>& bodyA_idx,
                                    vector<int>& bodyB_idx, bool use_margins) {
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_iter = bodies.begin(); body_iter != bodies.end();
       ++body_iter) {
    (*body_iter)->appendCollisionElementIdsFromThisBody(ids_to_check);
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx,
                         ids_to_check, use_margins);
}

void RigidBodyTree::potentialCollisions(const KinematicsCache<double>& cache,
                                        VectorXd& phi, Matrix3Xd& normal,
                                        Matrix3Xd& xA, Matrix3Xd& xB,
                                        vector<int>& bodyA_idx,
                                        vector<int>& bodyB_idx,
                                        bool use_margins) {
  updateDynamicCollisionElements(cache);
  vector<DrakeCollision::PointPair> potential_collisions;
  potential_collisions = collision_model->potentialCollisionPoints(use_margins);
  size_t num_potential_collisions = potential_collisions.size();

  phi = VectorXd::Zero(num_potential_collisions);
  normal = MatrixXd::Zero(3, num_potential_collisions);
  xA = Matrix3Xd(3, num_potential_collisions);
  xB = Matrix3Xd(3, num_potential_collisions);

  bodyA_idx.clear();
  bodyB_idx.clear();

  for (size_t i = 0; i < num_potential_collisions; i++) {
    const DrakeCollision::Element* elementA =
        potential_collisions[i].elementA;
    const DrakeCollision::Element* elementB =
        potential_collisions[i].elementB;
    xA.col(i) = potential_collisions[i].ptA;
    xB.col(i) = potential_collisions[i].ptB;
    normal.col(i) = potential_collisions[i].normal;
    phi[i] = potential_collisions[i].distance;
    bodyA_idx.push_back(elementA->get_body()->body_index);
    bodyB_idx.push_back(elementB->get_body()->body_index);
  }
}

std::vector<DrakeCollision::PointPair>
RigidBodyTree::ComputeMaximumDepthCollisionPoints(
    const KinematicsCache<double>& cache, bool use_margins) {
  updateDynamicCollisionElements(cache);
  vector<DrakeCollision::PointPair> contact_points;
  collision_model->ComputeMaximumDepthCollisionPoints(
      use_margins, contact_points);
  size_t num_contact_points = contact_points.size();

  for (size_t i = 0; i < num_contact_points; i++) {
    // Get bodies' transforms.
    const RigidBody& bodyA = *contact_points[i].elementA->get_body();
    Isometry3d TA;
    if (bodyA.hasParent()) {
      TA = cache.getElement(bodyA).transform_to_world;
    } else {  // body is the world.
      TA = Isometry3d::Identity();
    }

    const RigidBody& bodyB = *contact_points[i].elementB->get_body();
    Isometry3d TB;
    if (bodyB.hasParent()) {
      TB = cache.getElement(bodyB).transform_to_world;
    } else {  // body is the world.
      TB = Isometry3d::Identity();
    }

    // Transform to bodies' frames.
    // Note:
    // Eigen assumes aliasing by default and therefore this operation is safe.
    contact_points[i].ptA = TA.inverse() * contact_points[i].ptA;
    contact_points[i].ptB = TB.inverse() * contact_points[i].ptB;
  }
  return contact_points;
}

bool RigidBodyTree::collidingPointsCheckOnly(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model->collidingPointsCheckOnly(points, collision_threshold);
}

vector<size_t> RigidBodyTree::collidingPoints(
    const KinematicsCache<double>& cache, const vector<Vector3d>& points,
    double collision_threshold) {
  updateDynamicCollisionElements(cache);
  return collision_model->collidingPoints(points, collision_threshold);
}

bool RigidBodyTree::allCollisions(const KinematicsCache<double>& cache,
                                  vector<int>& bodyA_idx,
                                  vector<int>& bodyB_idx,
                                  Matrix3Xd& xA_in_world,
                                  Matrix3Xd& xB_in_world, bool use_margins) {
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  bool points_found =
      collision_model->ComputeMaximumDepthCollisionPoints(use_margins, points);

  xA_in_world = Matrix3Xd::Zero(3, points.size());
  xB_in_world = Matrix3Xd::Zero(3, points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    xA_in_world.col(i) = points[i].ptA;
    xB_in_world.col(i) = points[i].ptB;

    const DrakeCollision::Element* elementA = points[i].elementA;
    bodyA_idx.push_back(elementA->get_body()->body_index);
    const DrakeCollision::Element* elementB = points[i].elementB;
    bodyB_idx.push_back(elementB->get_body()->body_index);
  }
  return points_found;
}

template <typename DerivedQ>
KinematicsCache<typename DerivedQ::Scalar> RigidBodyTree::doKinematics(
    const Eigen::MatrixBase<DerivedQ>& q) const {
  KinematicsCache<typename DerivedQ::Scalar> ret(bodies);
  ret.initialize(q);
  doKinematics(ret);
  return ret;
}

template <typename DerivedQ, typename DerivedV>
KinematicsCache<typename DerivedQ::Scalar> RigidBodyTree::doKinematics(
    const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedV>& v,
    bool compute_JdotV) const {
  KinematicsCache<typename DerivedQ::Scalar> ret(bodies);
  ret.initialize(q, v);
  doKinematics(ret, compute_JdotV);
  return ret;
}

template <typename Scalar>
void RigidBodyTree::doKinematics(KinematicsCache<Scalar>& cache,
                                 bool compute_JdotV) const {
  const auto& q = cache.getQ();
  if (!initialized_)
    throw runtime_error("RigidBodyTree::doKinematics: call compile first.");

  // Only compute Jdot times V if V has been provided in the cache.
  compute_JdotV = compute_JdotV && cache.hasV();
  // Required in call to geometricJacobian below.
  cache.setPositionKinematicsCached();

  for (size_t i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    KinematicsCacheElement<Scalar>& element = cache.getElement(body);

    if (body.hasParent()) {
      const KinematicsCacheElement<Scalar>& parent_element =
          cache.getElement(*body.parent);
      const DrakeJoint& joint = body.getJoint();
      auto q_body =
          q.middleRows(body.position_num_start, joint.getNumPositions());

      // transform
      auto T_body_to_parent = joint.getTransformToParentBody().cast<Scalar>() *
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
        if (joint.getNumVelocities() == 0) {  // for fixed joints
          element.twist_in_world = parent_element.twist_in_world;
          if (compute_JdotV) {
            element.motion_subspace_in_world_dot_times_v =
                parent_element.motion_subspace_in_world_dot_times_v;
          }
        } else {
          // twist
          auto v_body =
              v.middleRows(body.velocity_num_start, joint.getNumVelocities());

          Eigen::Matrix<Scalar, TWIST_SIZE, 1> joint_twist =
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

template <typename Scalar>
void RigidBodyTree::updateCompositeRigidBodyInertias(
    KinematicsCache<Scalar>& cache) const {
  cache.checkCachedKinematicsSettings(false, false,
                                      "updateCompositeRigidBodyInertias");

  if (!cache.areInertiasCached()) {
    for (int i = 0; i < bodies.size(); i++) {
      auto& element = cache.getElement(*bodies[i]);
      element.inertia_in_world = transformSpatialInertia(
          element.transform_to_world, bodies[i]->I.cast<Scalar>());
      element.crb_in_world = element.inertia_in_world;
    }

    for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; i--) {
      if (bodies[i]->hasParent()) {
        const auto& element = cache.getElement(*bodies[i]);
        auto& parent_element = cache.getElement(*(bodies[i]->parent));
        parent_element.crb_in_world += element.crb_in_world;
      }
    }
  }
  cache.setInertiasCached();
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyTree::worldMomentumMatrix(
    KinematicsCache<Scalar>& cache, const std::set<int>& robotnum,
    bool in_terms_of_qdot) const {
  cache.checkCachedKinematicsSettings(false, false, "worldMomentumMatrix");
  updateCompositeRigidBodyInertias(cache);

  int nq = num_positions_;
  int nv = num_velocities_;
  int ncols = in_terms_of_qdot ? nq : nv;
  Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, ncols);
  ret.setZero();
  int gradient_row_start = 0;
  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];

    if (body.hasParent()) {
      const auto& element = cache.getElement(body);
      const DrakeJoint& joint = body.getJoint();
      int ncols_joint =
          in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
      if (isBodyPartOfRobot(body, robotnum)) {
        int start = in_terms_of_qdot ? body.position_num_start
                                     : body.velocity_num_start;

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
      gradient_row_start += TWIST_SIZE * ncols_joint;
    }
  }
  return ret;
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, 1> RigidBodyTree::worldMomentumMatrixDotTimesV(
    KinematicsCache<Scalar>& cache, const std::set<int>& robotnum) const {
  cache.checkCachedKinematicsSettings(true, true,
                                      "worldMomentumMatrixDotTimesV");
  updateCompositeRigidBodyInertias(cache);

  Matrix<Scalar, TWIST_SIZE, 1> ret;
  ret.setZero();
  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      if (isBodyPartOfRobot(body, robotnum)) {
        const auto& element = cache.getElement(body);
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

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic>
RigidBodyTree::centroidalMomentumMatrix(KinematicsCache<Scalar>& cache,
                                        const std::set<int>& robotnum,
                                        bool in_terms_of_qdot) const {
  // kinematics cache checks already being done in worldMomentumMatrix.
  auto ret = worldMomentumMatrix(cache, robotnum, in_terms_of_qdot);

  // transform from world frame to COM frame
  auto com = centerOfMass(cache, robotnum);
  auto angular_momentum_matrix = ret.template topRows<SPACE_DIMENSION>();
  auto linear_momentum_matrix = ret.template bottomRows<SPACE_DIMENSION>();
  angular_momentum_matrix += linear_momentum_matrix.colwise().cross(com);

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry>
  //  T(Translation<Scalar, SPACE_DIMENSION>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, 1> RigidBodyTree::centroidalMomentumMatrixDotTimesV(
    KinematicsCache<Scalar>& cache, const std::set<int>& robotnum) const {
  // kinematics cache checks already being done in worldMomentumMatrixDotTimesV
  auto ret = worldMomentumMatrixDotTimesV(cache, robotnum);

  // transform from world frame to COM frame:
  auto com = centerOfMass(cache, robotnum);
  auto angular_momentum_matrix_dot_times_v =
      ret.template topRows<SPACE_DIMENSION>();
  auto linear_momentum_matrix_dot_times_v =
      ret.template bottomRows<SPACE_DIMENSION>();
  angular_momentum_matrix_dot_times_v +=
      linear_momentum_matrix_dot_times_v.cross(com);

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry>
  //  T(Translation<Scalar, SPACE_DIMENSION>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

bool RigidBodyTree::isBodyPartOfRobot(const RigidBody& body,
                                      const std::set<int>& robotnum) const {
  for (std::set<int>::const_iterator it = robotnum.begin();
       it != robotnum.end(); ++it) {
    if (*it < -1) {
      return true;
    }
  }

  return robotnum.find(body.robotnum) != robotnum.end();
}

double RigidBodyTree::getMass(const std::set<int>& robotnum) const {
  double total_mass = 0.0;
  for (size_t i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (isBodyPartOfRobot(body, robotnum)) {
      total_mass += body.mass;
    }
  }
  return total_mass;
}

template <typename Scalar>
Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> RigidBodyTree::centerOfMass(
    KinematicsCache<Scalar>& cache, const std::set<int>& robotnum) const {
  cache.checkCachedKinematicsSettings(false, false, "centerOfMass");

  Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> com;
  com.setZero();
  double m = 0.0;

  for (size_t i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (isBodyPartOfRobot(body, robotnum)) {
      if (body.mass > 0) {
        com.noalias() +=
            body.mass * transformPoints(cache, body.com.cast<Scalar>(), i, 0);
      }
      m += body.mass;
    }
  }
  if (m > 0.0) com /= m;

  return com;
}

template <typename Scalar>
Matrix<Scalar, SPACE_DIMENSION, Eigen::Dynamic>
RigidBodyTree::centerOfMassJacobian(KinematicsCache<Scalar>& cache,
                                    const std::set<int>& robotnum,
                                    bool in_terms_of_qdot) const {
  cache.checkCachedKinematicsSettings(false, false, "centerOfMassJacobian");
  auto A = worldMomentumMatrix(cache, robotnum, in_terms_of_qdot);
  double total_mass = getMass(robotnum);
  return A.template bottomRows<SPACE_DIMENSION>() / total_mass;
}

template <typename Scalar>
Matrix<Scalar, SPACE_DIMENSION, 1> RigidBodyTree::centerOfMassJacobianDotTimesV(
    KinematicsCache<Scalar>& cache, const std::set<int>& robotnum) const {
  // kinematics cache checks are already being done in
  // centroidalMomentumMatrixDotTimesV
  auto cmm_dot_times_v = centroidalMomentumMatrixDotTimesV(cache, robotnum);
  double total_mass = getMass(robotnum);
  return cmm_dot_times_v.template bottomRows<SPACE_DIMENSION>() / total_mass;
}

template <typename DerivedNormal, typename DerivedPoint>
std::pair<Eigen::Vector3d, double> RigidBodyTree::resolveCenterOfPressure(
    const KinematicsCache<double>& cache,
    const std::vector<ForceTorqueMeasurement>& force_torque_measurements,
    const Eigen::MatrixBase<DerivedNormal>& normal,
    const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane) const {
  // kinematics cache checks are already being done in relativeTransform
  typedef typename DerivedNormal::Scalar Scalar;
  typedef Matrix<Scalar, 6, 1> Vector6;
  Vector6 total_wrench = Vector6::Zero();
  for (auto it = force_torque_measurements.begin();
       it != force_torque_measurements.end(); ++it) {
    auto transform_to_world = relativeTransform(cache, 0, it->frame_idx);
    total_wrench += transformSpatialForce(transform_to_world, it->wrench);
  }
  return ::resolveCenterOfPressure(total_wrench.template head<3>(),
                                   total_wrench.template tail<3>(), normal,
                                   point_on_contact_plane);
}

int RigidBodyTree::getNumContacts(const set<int>& body_idx) const {
  size_t n = 0, nb = body_idx.size(), bi;
  if (nb == 0) nb = bodies.size();
  set<int>::iterator iter = body_idx.begin();
  for (size_t i = 0; i < nb; i++) {
    if (body_idx.size() == 0)
      bi = i;
    else
      bi = *iter++;
    n += bodies[bi]->contact_pts.cols();
  }
  return static_cast<int>(n);
}

/* [body_ind, Tframe] = parseBodyOrFrameID(body_or_frame_id) */
template <typename Scalar>
int RigidBodyTree::parseBodyOrFrameID(
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
    if (frame_ind >= frames.size()) {
      std::ostringstream stream;
      stream << "Got a frame ind greater than available!\n";
      throw std::runtime_error(stream.str());
    }
    body_ind = frames[frame_ind]->body->body_index;

    if (Tframe) (*Tframe) = frames[frame_ind]->transform_to_body.cast<Scalar>();
  } else {
    body_ind = body_or_frame_id;
    if (Tframe) Tframe->setIdentity();
  }
  return body_ind;
}

int RigidBodyTree::parseBodyOrFrameID(const int body_or_frame_id) const {
  return parseBodyOrFrameID<double>(body_or_frame_id, nullptr);
}

void RigidBodyTree::findAncestorBodies(std::vector<int>& ancestor_bodies,
                                       int body_idx) const {
  const RigidBody* current_body = bodies[body_idx].get();
  while (current_body->hasParent()) {
    ancestor_bodies.push_back(current_body->parent->body_index);
    current_body = current_body->parent;
  }
}

KinematicPath RigidBodyTree::findKinematicPath(
    int start_body_or_frame_idx, int end_body_or_frame_idx) const {
  // find all ancestors of start_body and end_body
  int start_body = parseBodyOrFrameID(start_body_or_frame_idx);

  std::vector<int> start_body_ancestors;
  start_body_ancestors.push_back(start_body);
  findAncestorBodies(start_body_ancestors, start_body);

  int end_body = parseBodyOrFrameID(end_body_or_frame_idx);
  std::vector<int> end_body_ancestors;
  end_body_ancestors.push_back(end_body);
  findAncestorBodies(end_body_ancestors, end_body);

  // find least common ancestor
  size_t common_size =
      std::min(start_body_ancestors.size(), end_body_ancestors.size());
  bool least_common_ancestor_found = false;
  std::vector<int>::iterator start_body_lca_it =
      start_body_ancestors.end() - common_size;
  std::vector<int>::iterator end_body_lca_it =
      end_body_ancestors.end() - common_size;

  for (size_t i = 0; i < common_size; i++) {
    if (*start_body_lca_it == *end_body_lca_it) {
      least_common_ancestor_found = true;
      break;
    }
    start_body_lca_it++;
    end_body_lca_it++;
  }

  if (!least_common_ancestor_found) {
    std::ostringstream stream;
    stream << "There is no path between " << bodies[start_body]->name_
           << " and " << bodies[end_body]->name_ << ".";
    throw std::runtime_error(stream.str());
  }
  int least_common_ancestor = *start_body_lca_it;

  // compute path
  KinematicPath path;

  std::vector<int>::iterator it = start_body_ancestors.begin();
  for (; it != start_body_lca_it; it++) {
    path.joint_path.push_back(*it);
    path.joint_direction_signs.push_back(-1);
    path.body_path.push_back(*it);
  }

  path.body_path.push_back(least_common_ancestor);

  std::vector<int>::reverse_iterator reverse_it(end_body_lca_it);
  for (; reverse_it != end_body_ancestors.rend(); reverse_it++) {
    path.joint_path.push_back(*reverse_it);
    path.joint_direction_signs.push_back(1);
    path.body_path.push_back(*reverse_it);
  }
  return path;
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyTree::geometricJacobian(
    const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
    int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind,
    bool in_terms_of_qdot, std::vector<int>* v_or_qdot_indices) const {
  cache.checkCachedKinematicsSettings(false, false, "geometricJacobian");

  KinematicPath kinematic_path =
      findKinematicPath(base_body_or_frame_ind, end_effector_body_or_frame_ind);

  int cols = 0;
  int body_index;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); i++) {
    body_index = kinematic_path.joint_path[i];
    const RigidBody& body = *bodies[body_index];
    const DrakeJoint& joint = body.getJoint();
    cols +=
        in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
  }

  Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> J(TWIST_SIZE, cols);

  if (v_or_qdot_indices != nullptr) {
    v_or_qdot_indices->clear();
    v_or_qdot_indices->reserve(cols);
  }

  int col_start = 0;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); i++) {
    body_index = kinematic_path.joint_path[i];
    RigidBody& body = *bodies[body_index];
    const auto& element = cache.getElement(body);
    const DrakeJoint& joint = body.getJoint();
    int ncols_block =
        in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
    int sign = kinematic_path.joint_direction_signs[i];
    auto J_block = J.template block<TWIST_SIZE, Dynamic>(
        0, col_start, TWIST_SIZE, ncols_block);
    if (in_terms_of_qdot) {
      J_block.noalias() =
          sign * element.motion_subspace_in_world * element.qdot_to_v;
    } else {
      J_block.noalias() = sign * element.motion_subspace_in_world;
    }

    if (v_or_qdot_indices != nullptr) {
      int cols_block_start =
          in_terms_of_qdot ? body.position_num_start : body.velocity_num_start;
      for (int j = 0; j < ncols_block; j++) {
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

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, 1> RigidBodyTree::geometricJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
    int end_effector_body_or_frame_ind,
    int expressed_in_body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(true, true, "geometricJacobianDotTimesV");

  Matrix<Scalar, TWIST_SIZE, 1> ret(TWIST_SIZE, 1);

  int base_body_ind = parseBodyOrFrameID(base_body_or_frame_ind);
  int end_effector_body_ind =
      parseBodyOrFrameID(end_effector_body_or_frame_ind);

  const auto& base_element = cache.getElement(*bodies[base_body_ind]);
  const auto& end_effector_element =
      cache.getElement(*bodies[end_effector_body_ind]);

  ret = end_effector_element.motion_subspace_in_world_dot_times_v -
        base_element.motion_subspace_in_world_dot_times_v;

  int world_ind = 0;
  return transformSpatialAcceleration(cache, ret, base_body_ind,
                                      end_effector_body_ind, world_ind,
                                      expressed_in_body_or_frame_ind);
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, 1> RigidBodyTree::relativeTwist(
    const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
    int body_or_frame_ind, int expressed_in_body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(true, false, "relativeTwist");

  int base_ind = parseBodyOrFrameID(base_or_frame_ind);
  int body_ind = parseBodyOrFrameID(body_or_frame_ind);
  int world = 0;
  auto T = relativeTransform(cache, expressed_in_body_or_frame_ind, world);

  const auto& base_element = cache.getElement(*bodies[base_ind]);
  const auto& body_element = cache.getElement(*bodies[body_ind]);
  Matrix<Scalar, TWIST_SIZE, 1> relative_twist_in_world =
      body_element.twist_in_world - base_element.twist_in_world;
  return transformSpatialMotion(T, relative_twist_in_world);
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, 1> RigidBodyTree::transformSpatialAcceleration(
    const KinematicsCache<Scalar>& cache,
    const Matrix<Scalar, TWIST_SIZE, 1>& spatial_acceleration, int base_ind,
    int body_ind, int old_expressed_in_body_or_frame_ind,
    int new_expressed_in_body_or_frame_ind) const {
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

  Matrix<Scalar, TWIST_SIZE, 1> spatial_accel_temp =
      crossSpatialMotion(twist_of_old_wrt_new, twist_of_body_wrt_base);
  spatial_accel_temp += spatial_acceleration;
  return transformSpatialMotion(T_old_to_new, spatial_accel_temp);
}

template <typename Scalar>
Transform<Scalar, 3, Isometry> RigidBodyTree::relativeTransform(
    const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
    int body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(false, false, "relativeTransform");

  Transform<Scalar, 3, Isometry> Tbase_frame;
  int base_ind = parseBodyOrFrameID(base_or_frame_ind, &Tbase_frame);
  Transform<Scalar, 3, Isometry> Tbody_frame;
  int body_ind = parseBodyOrFrameID(body_or_frame_ind, &Tbody_frame);

  const auto& body_element = cache.getElement(*bodies[body_ind]);
  const auto& base_element = cache.getElement(*bodies[base_ind]);

  Transform<Scalar, 3, Isometry> Tbaseframe_to_world =
      base_element.transform_to_world * Tbase_frame;
  Transform<Scalar, 3, Isometry> Tworld_to_baseframe =
      Tbaseframe_to_world.inverse();
  Transform<Scalar, 3, Isometry> Tbodyframe_to_world =
      body_element.transform_to_world * Tbody_frame;
  return Tworld_to_baseframe * Tbodyframe_to_world;
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyTree::massMatrix(
    KinematicsCache<Scalar>& cache) const {
  cache.checkCachedKinematicsSettings(false, false, "massMatrix");

  int nv = num_velocities_;
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(nv, nv);
  ret.setZero();

  updateCompositeRigidBodyInertias(cache);

  for (size_t i = 0; i < bodies.size(); i++) {
    RigidBody& body_i = *bodies[i];
    if (body_i.hasParent()) {
      const auto& element_i = cache.getElement(body_i);
      int v_start_i = body_i.velocity_num_start;
      int nv_i = body_i.getJoint().getNumVelocities();
      auto F =
          (element_i.crb_in_world * element_i.motion_subspace_in_world).eval();

      // Hii
      ret.block(v_start_i, v_start_i, nv_i, nv_i).noalias() =
          (element_i.motion_subspace_in_world.transpose() * F).eval();

      // Hij
      RigidBody* body_j(body_i.parent);
      while (body_j->hasParent()) {
        const auto& element_j = cache.getElement(*body_j);
        int v_start_j = body_j->velocity_num_start;
        int nv_j = body_j->getJoint().getNumVelocities();
        auto Hji = (element_j.motion_subspace_in_world.transpose() * F).eval();
        ret.block(v_start_j, v_start_i, nv_j, nv_i) = Hji;
        ret.block(v_start_i, v_start_j, nv_i, nv_j) = Hji.transpose();

        body_j = body_j->parent;
      }
    }
  }

  return ret;
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree::dynamicsBiasTerm(
    KinematicsCache<Scalar>& cache,
    const eigen_aligned_unordered_map<RigidBody const*,
                                      Matrix<Scalar, TWIST_SIZE, 1>>& f_ext,
    bool include_velocity_terms) const {
  Matrix<Scalar, Eigen::Dynamic, 1> vd(num_velocities_, 1);
  vd.setZero();
  return inverseDynamics(cache, f_ext, vd, include_velocity_terms);
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree::inverseDynamics(
    KinematicsCache<Scalar>& cache,
    const eigen_aligned_unordered_map<RigidBody const*,
                                      Matrix<Scalar, TWIST_SIZE, 1>>& f_ext,
    const Matrix<Scalar, Eigen::Dynamic, 1>& vd,
    bool include_velocity_terms) const {
  cache.checkCachedKinematicsSettings(
      include_velocity_terms, include_velocity_terms, "inverseDynamics");

  updateCompositeRigidBodyInertias(cache);

  typedef typename Eigen::Matrix<Scalar, TWIST_SIZE, 1> Vector6;

  Vector6 root_accel = -a_grav.cast<Scalar>();
  Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> net_wrenches(TWIST_SIZE,
                                                          bodies.size());
  net_wrenches.col(0).setZero();

  for (size_t i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      const auto& element = cache.getElement(body);

      Vector6 spatial_accel = root_accel;
      if (include_velocity_terms)
        spatial_accel += element.motion_subspace_in_world_dot_times_v;

      int nv_joint = body.getJoint().getNumVelocities();
      auto vdJoint = vd.middleRows(body.velocity_num_start, nv_joint);
      spatial_accel.noalias() += element.motion_subspace_in_world * vdJoint;

      net_wrenches.col(i).noalias() = element.inertia_in_world * spatial_accel;
      if (include_velocity_terms) {
        auto I_times_twist =
            (element.inertia_in_world * element.twist_in_world).eval();
        net_wrenches.col(i).noalias() +=
            crossSpatialForce(element.twist_in_world, I_times_twist);
      }

      auto f_ext_iterator = f_ext.find(bodies[i].get());
      if (f_ext_iterator != f_ext.end()) {
        const auto& f_ext_i = f_ext_iterator->second;
        net_wrenches.col(i) -=
            transformSpatialForce(element.transform_to_world, f_ext_i);
      }
    }
  }

  Matrix<Scalar, Eigen::Dynamic, 1> ret(num_velocities_, 1);

  for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; i--) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      const auto& element = cache.getElement(body);
      const auto& net_wrenches_const = net_wrenches;  // eliminates the need for
                                                      // another explicit
                                                      // instantiation
      auto joint_wrench = net_wrenches_const.col(i);
      int nv_joint = body.getJoint().getNumVelocities();
      auto J_transpose = element.motion_subspace_in_world.transpose();
      ret.middleRows(body.velocity_num_start, nv_joint).noalias() =
          J_transpose * joint_wrench;
      auto parent_net_wrench = net_wrenches.col(body.parent->body_index);
      parent_net_wrench += joint_wrench;
    }
  }

  if (include_velocity_terms) ret += frictionTorques(cache.getV());

  return ret;
}

template <typename DerivedV>
Matrix<typename DerivedV::Scalar, Dynamic, 1> RigidBodyTree::frictionTorques(
    Eigen::MatrixBase<DerivedV> const& v) const {
  typedef typename DerivedV::Scalar Scalar;
  Matrix<Scalar, Dynamic, 1> ret(num_velocities_, 1);

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      const DrakeJoint& joint = body.getJoint();
      int nv_joint = joint.getNumVelocities();
      int v_start_joint = body.velocity_num_start;
      auto v_body = v.middleRows(v_start_joint, nv_joint);
      ret.middleRows(v_start_joint, nv_joint) = joint.frictionTorque(v_body);
    }
  }

  return ret;
}

template <typename Scalar, typename DerivedPoints>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree::transformPointsJacobian(
    const KinematicsCache<Scalar>& cache,
    const Eigen::MatrixBase<DerivedPoints>& points, int from_body_or_frame_ind,
    int to_body_or_frame_ind, bool in_terms_of_qdot) const {
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

  auto Jomega = J_geometric.template topRows<SPACE_DIMENSION>();
  auto Jv = J_geometric.template bottomRows<SPACE_DIMENSION>();

  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J(
      points_base.size(), cols);  // TODO(tkoolen): size at compile time
  J.setZero();

  int row_start = 0;
  for (int i = 0; i < npoints; i++) {
    // translation part
    int col = 0;
    for (std::vector<int>::iterator it = v_or_q_indices.begin();
         it != v_or_q_indices.end(); ++it) {
      J.template block<SPACE_DIMENSION, 1>(row_start, *it) = Jv.col(col);
      J.template block<SPACE_DIMENSION, 1>(row_start, *it).noalias() +=
          Jomega.col(col).cross(points_base.col(i));
      col++;
    }
    row_start += SPACE_DIMENSION;
  }

  return J;
}

template <typename Scalar>
Eigen::Matrix<Scalar, QUAT_SIZE, Eigen::Dynamic>
RigidBodyTree::relativeQuaternionJacobian(const KinematicsCache<Scalar>& cache,
                                          int from_body_or_frame_ind,
                                          int to_body_or_frame_ind,
                                          bool in_terms_of_qdot) const {
  int body_ind = parseBodyOrFrameID(from_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(to_body_or_frame_ind);
  KinematicPath kinematic_path = findKinematicPath(base_ind, body_ind);
  auto J_geometric = geometricJacobian(cache, base_ind, body_ind,
                                       to_body_or_frame_ind, in_terms_of_qdot);
  auto Jomega = J_geometric.template topRows<SPACE_DIMENSION>();
  auto quat =
      relativeQuaternion(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, QUAT_SIZE, SPACE_DIMENSION> Phi;
  angularvel2quatdotMatrix(
      quat, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr));
  return compactToFull((Phi * Jomega).eval(), kinematic_path.joint_path,
                       in_terms_of_qdot);
}

template <typename Scalar>
Eigen::Matrix<Scalar, RPY_SIZE, Eigen::Dynamic>
RigidBodyTree::relativeRollPitchYawJacobian(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind, bool in_terms_of_qdot) const {
  int body_ind = parseBodyOrFrameID(from_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(to_body_or_frame_ind);
  KinematicPath kinematic_path = findKinematicPath(base_ind, body_ind);
  auto J_geometric = geometricJacobian(cache, base_ind, body_ind,
                                       to_body_or_frame_ind, in_terms_of_qdot);
  auto Jomega = J_geometric.template topRows<SPACE_DIMENSION>();
  auto rpy =
      relativeRollPitchYaw(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, RPY_SIZE, SPACE_DIMENSION> Phi;
  angularvel2rpydotMatrix(
      rpy, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr),
      static_cast<typename Gradient<decltype(Phi), Dynamic, 2>::type*>(
          nullptr));
  return compactToFull((Phi * Jomega).eval(), kinematic_path.joint_path,
                       in_terms_of_qdot);
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree::forwardKinPositionGradient(const KinematicsCache<Scalar>& cache,
                                          int npoints,
                                          int from_body_or_frame_ind,
                                          int to_body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(false, false,
                                      "forwardKinPositionGradient");

  auto Tinv =
      relativeTransform(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(SPACE_DIMENSION * npoints,
                                                     SPACE_DIMENSION * npoints);
  ret.setZero();
  for (int i = 0; i < npoints; i++) {
    ret.template block<SPACE_DIMENSION, SPACE_DIMENSION>(
        SPACE_DIMENSION * i, SPACE_DIMENSION * i) = Tinv.linear();
  }
  return ret;
}

template <typename Scalar, typename DerivedPoints>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree::transformPointsJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache,
    const Eigen::MatrixBase<DerivedPoints>& points, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
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

  auto omega_twist = twist.template topRows<SPACE_DIMENSION>();
  auto v_twist = twist.template bottomRows<SPACE_DIMENSION>();

  auto rdots = (-r.colwise().cross(omega_twist)).eval();
  rdots.colwise() += v_twist;
  auto Jposdot_times_v_mat = (-rdots.colwise().cross(omega_twist)).eval();
  Jposdot_times_v_mat -=
      (r.colwise().cross(
           J_geometric_dot_times_v.template topRows<SPACE_DIMENSION>()))
          .eval();
  Jposdot_times_v_mat.colwise() +=
      J_geometric_dot_times_v.template bottomRows<SPACE_DIMENSION>();

  return Map<Matrix<Scalar, Dynamic, 1>>(Jposdot_times_v_mat.data(), r.size(),
                                         1);
}

template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree::relativeQuaternionJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(true, true,
                                      "relativeQuaternionJacobianDotTimesV");

  auto quat =
      relativeQuaternion(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, QUAT_SIZE, SPACE_DIMENSION> Phi;
  angularvel2quatdotMatrix(
      quat, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr));

  int expressed_in = to_body_or_frame_ind;
  const auto twist = relativeTwist(cache, to_body_or_frame_ind,
                                   from_body_or_frame_ind, expressed_in);
  auto omega_twist = twist.template topRows<SPACE_DIMENSION>();
  auto quatdot = (Phi * omega_twist).eval();

  using ADScalar = AutoDiffScalar<Matrix<Scalar, Dynamic,
                                         1>>;  // would prefer to use 1 instead
                                               // of Dynamic, but this causes
                                               // issues related to
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1006 on MSVC
  // 32 bit
  auto quat_autodiff = quat.template cast<ADScalar>().eval();
  gradientMatrixToAutoDiff(quatdot, quat_autodiff);
  Matrix<ADScalar, QUAT_SIZE, SPACE_DIMENSION> Phi_autodiff;
  angularvel2quatdotMatrix(
      quat_autodiff, Phi_autodiff,
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic>::type*>(
          nullptr));
  auto Phidot_vector = autoDiffToGradientMatrix(Phi_autodiff);
  Map<Matrix<Scalar, QUAT_SIZE, SPACE_DIMENSION>> Phid(Phidot_vector.data());

  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(
      cache, to_body_or_frame_ind, from_body_or_frame_ind, expressed_in);
  auto ret = (Phid * omega_twist).eval();
  ret.noalias() +=
      Phi * J_geometric_dot_times_v.template topRows<SPACE_DIMENSION>();
  return ret;
}

template <typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV(
    const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
    int to_body_or_frame_ind) const {
  cache.checkCachedKinematicsSettings(true, true,
                                      "relativeRollPitchYawJacobianDotTimesV");

  auto rpy =
      relativeRollPitchYaw(cache, from_body_or_frame_ind, to_body_or_frame_ind);
  Matrix<Scalar, RPY_SIZE, SPACE_DIMENSION> Phi;
  angularvel2rpydotMatrix(
      rpy, Phi,
      static_cast<typename Gradient<decltype(Phi), Dynamic>::type*>(nullptr),
      static_cast<typename Gradient<decltype(Phi), Dynamic, 2>::type*>(
          nullptr));

  int expressed_in = to_body_or_frame_ind;
  const auto twist = relativeTwist(cache, to_body_or_frame_ind,
                                   from_body_or_frame_ind, expressed_in);
  auto omega_twist = twist.template topRows<SPACE_DIMENSION>();
  auto rpydot = (Phi * omega_twist).eval();

  using ADScalar = AutoDiffScalar<Matrix<Scalar, Dynamic,
                                         1>>;  // would prefer to use 1 instead
                                               // of Dynamic, but this causes
                                               // issues related to
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1006 on MSVC
  // 32 bit
  auto rpy_autodiff = rpy.template cast<ADScalar>().eval();
  gradientMatrixToAutoDiff(rpydot, rpy_autodiff);
  Matrix<ADScalar, RPY_SIZE, SPACE_DIMENSION> Phi_autodiff;
  angularvel2rpydotMatrix(
      rpy_autodiff, Phi_autodiff,
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic>::type*>(
          nullptr),
      static_cast<typename Gradient<decltype(Phi_autodiff), Dynamic, 2>::type*>(
          nullptr));
  auto Phidot_vector = autoDiffToGradientMatrix(Phi_autodiff);
  Map<Matrix<Scalar, RPY_SIZE, SPACE_DIMENSION>> Phid(Phidot_vector.data());

  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(
      cache, to_body_or_frame_ind, from_body_or_frame_ind, expressed_in);
  auto ret = (Phid * omega_twist).eval();
  ret.noalias() +=
      Phi * J_geometric_dot_times_v.template topRows<SPACE_DIMENSION>();
  return ret;
}

RigidBody* RigidBodyTree::FindBody(const std::string& body_name,
                                   const std::string& model_name,
                                   int model_id) const {
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

  for (size_t ii = 0; ii < bodies.size(); ++ii) {
    // Skips the current body if model_id is not -1 and the body's robot ID is
    // not equal to the desired model ID.
    if (model_id != -1 && model_id != bodies[ii]->get_model_id()) continue;

    // Obtains a lower case version of the current body's model name.
    string current_model_name = bodies[ii]->model_name();
    std::transform(current_model_name.begin(), current_model_name.end(),
                   current_model_name.begin(), ::tolower);

    // Skips the current body if model_name is not empty and the body's model
    // name is not equal to the desired model name.
    if (!model_name_lower.empty() && model_name_lower != current_model_name)
      continue;

    // Obtains a lower case version of the current body's name.
    string current_body_name = bodies[ii]->name_;
    std::transform(current_body_name.begin(), current_body_name.end(),
                   current_body_name.begin(), ::tolower);

    // Checks if the body names match. If so, checks whether this is the first
    // match. If so, it saves the current body's index. Otherwise it throws
    // an exception indicating there are multiple matches.
    if (current_body_name == body_name_lower) {
      if (match_index < 0) {
        match_index = ii;
      } else {
        throw std::logic_error(
            "RigidBodyTree::FindBody: ERROR: found multiple bodys named \"" +
            body_name + "\", model name = \"" + model_name + "\", model id = " +
            std::to_string(model_id) + ".");
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
        body_name + "\", model name = \"" + model_name + "\", model id = " +
        std::to_string(model_id) + ".");
  }
}

RigidBody* RigidBodyTree::findLink(const std::string& link_name,
                    const std::string& model_name,
                    int model_id) const {
  return FindBody(link_name, model_name, model_id);
}

shared_ptr<RigidBodyFrame> RigidBodyTree::findFrame(
    const std::string& frame_name, int model_id) const {
  std::string frame_name_lower = frame_name;

  // Obtains a lower case version of frame_name.
  std::transform(frame_name_lower.begin(), frame_name_lower.end(),
                 frame_name_lower.begin(), ::tolower);

  // Instantiates a variable that keeps track of the index within the frames
  // array that contains the desired frame. It is initialized to be -1 to
  // indicate that no matching frame was found.
  int match_index = -1;

  for (size_t ii = 0; ii < frames.size(); ++ii) {
    // Skips the current frame if model_id is not -1 and the frame's robot ID is
    // not equal to the desired robot ID.
    if (model_id != -1 && model_id != frames[ii]->get_model_id()) continue;

    // Obtains a lower case version of the current frame.
    std::string current_frame_name = frames[ii]->name;
    std::transform(current_frame_name.begin(), current_frame_name.end(),
                   current_frame_name.begin(), ::tolower);

    // Checks if the frame names match. If so, checks whether this is the first
    // match. If so, it saves the current frame's index. Otherwise it throws
    // an exception indicating there are multiple matches.
    if (frame_name_lower == current_frame_name) {
      if (match_index < 0) {
        match_index = ii;
      } else {
        throw std::logic_error(
            "RigidBodyTree::findFrame: ERROR: Found multiple frames named \"" +
            frame_name + "\", model_id = " + std::to_string(model_id));
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
        frame_name + "\", model id = " + std::to_string(model_id) + ".");
  }
}

int RigidBodyTree::FindBodyIndex(const std::string& body_name,
                                 int model_id) const {
  RigidBody* body = FindBody(body_name, "", model_id);
  if (body == nullptr) {
    throw std::logic_error(
        "RigidBodyTree::FindBodyIndex: ERROR: Could not find index for rigid "
        "body \"" +
        body_name + "\", model_id = " + std::to_string(model_id) + ".");
  }
  return body->body_index;
}

int RigidBodyTree::findLinkId(const std::string& link_name, int model_id)
                              const {
  return FindBodyIndex(link_name, model_id);
}

RigidBody* RigidBodyTree::findJoint(const std::string& joint_name,
                                    int model_id) const {
  // Obtains a lower case version of joint_name.
  std::string joint_name_lower = joint_name;
  std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                 joint_name_lower.begin(), ::tolower);

  vector<bool> name_match;
  name_match.resize(this->bodies.size());

  for (size_t ii = 0; ii < this->bodies.size(); ii++) {
    if (bodies[ii]->hasParent()) {
      string current_joint_name = this->bodies[ii]->getJoint().getName();
      std::transform(current_joint_name.begin(), current_joint_name.end(),
                     current_joint_name.begin(),
                     ::tolower);  // convert to lower case
      if (current_joint_name == joint_name_lower) {
        name_match[ii] = true;
      } else {
        name_match[ii] = false;
      }
    }
  }

  if (model_id != -1) {
    for (size_t ii = 0; ii < this->bodies.size(); ii++) {
      if (name_match[ii]) {
        name_match[ii] = this->bodies[ii]->robotnum == model_id;
      }
    }
  }

  // Unlike the MATLAB implementation, I am not handling the fixed joints
  size_t ind_match = 0;
  bool match_found = false;
  for (size_t ii = 0; ii < this->bodies.size(); ++ii) {
    if (name_match[ii]) {
      if (match_found) {
        throw std::logic_error(
            "RigidBodyTree::findJoint: ERROR: Multiple "
            "joints found named \"" +
            joint_name + "\", model ID = " + std::to_string(model_id) + ".");
      }
      ind_match = ii;
      match_found = true;
    }
  }
  if (!match_found) {
    throw std::logic_error(
        "RigidBodyTree::findJoint: ERROR: Could not find unique joint " +
        std::string("named \"") + joint_name + "\", model_id = " +
        std::to_string(model_id));
  } else {
    return this->bodies[ind_match].get();
  }
}

int RigidBodyTree::findJointId(const std::string& joint_name, int robot) const {
  RigidBody* link = findJoint(joint_name, robot);
  if (link == nullptr)
    throw std::runtime_error("could not find joint id: " + joint_name);
  return link->body_index;
}

std::string RigidBodyTree::getBodyOrFrameName(int body_or_frame_id) const {
  if (body_or_frame_id >= 0) {
    return bodies[body_or_frame_id]->name_;
  } else if (body_or_frame_id < -1) {
    return frames[-body_or_frame_id - 2]->name;
  } else {
    return "COM";
  }
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1> RigidBodyTree::positionConstraints(
    const KinematicsCache<Scalar>& cache) const {
  Matrix<Scalar, Eigen::Dynamic, 1> ret(6 * loops.size(), 1);
  for (size_t i = 0; i < loops.size(); i++) {
    {  // position constraint
      auto ptA_in_B =
          transformPoints(cache, Vector3d::Zero(), loops[i].frameA->frame_index,
                          loops[i].frameB->frame_index);
      ret.template middleRows<3>(6 * i) = ptA_in_B;
    }
    {  // second position constraint (to constrain orientation)
      auto axis_A_end_in_B =
          transformPoints(cache, loops[i].axis, loops[i].frameA->frame_index,
                          loops[i].frameB->frame_index);
      ret.template middleRows<3>(6 * i + 3) = axis_A_end_in_B - loops[i].axis;
    }
  }
  return ret;
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
RigidBodyTree::positionConstraintsJacobian(const KinematicsCache<Scalar>& cache,
                                           bool in_terms_of_qdot) const {
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(
      6 * loops.size(), in_terms_of_qdot ? num_positions_ : num_velocities_);

  for (size_t i = 0; i < loops.size(); i++) {
    // position constraint
    ret.template middleRows<3>(6 * i) = transformPointsJacobian(
        cache, Vector3d::Zero(), loops[i].frameA->frame_index,
        loops[i].frameB->frame_index, in_terms_of_qdot);
    // second position constraint (to constrain orientation)
    ret.template middleRows<3>(6 * i + 3) = transformPointsJacobian(
        cache, loops[i].axis, loops[i].frameA->frame_index,
        loops[i].frameB->frame_index, in_terms_of_qdot);
  }
  return ret;
}

template <typename Scalar>
Matrix<Scalar, Eigen::Dynamic, 1>
RigidBodyTree::positionConstraintsJacDotTimesV(
    const KinematicsCache<Scalar>& cache) const {
  Matrix<Scalar, Eigen::Dynamic, 1> ret(6 * loops.size(), 1);

  for (size_t i = 0; i < loops.size(); i++) {
    // position constraint
    ret.template middleRows<3>(6 * i) = transformPointsJacobianDotTimesV(
        cache, Vector3d::Zero(), loops[i].frameA->frame_index,
        loops[i].frameB->frame_index);
    // second position constraint (to constrain orientation)
    ret.template middleRows<3>(6 * i + 3) = transformPointsJacobianDotTimesV(
        cache, loops[i].axis, loops[i].frameA->frame_index,
        loops[i].frameB->frame_index);
  }
  return ret;
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
void RigidBodyTree::jointLimitConstraints(MatrixBase<DerivedA> const& q,
                                          MatrixBase<DerivedB>& phi,
                                          MatrixBase<DerivedC>& J) const {
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  const size_t numFiniteMin = finite_min_index.size();
  const size_t numFiniteMax = finite_max_index.size();

  phi = VectorXd::Zero(numFiniteMin + numFiniteMax);
  J = MatrixXd::Zero(phi.size(), num_positions_);
  for (size_t i = 0; i < numFiniteMin; i++) {
    const int fi = finite_min_index[i];
    phi[i] = q[fi] - joint_limit_min[fi];
    J(i, fi) = 1.0;
  }

  for (size_t i = 0; i < numFiniteMax; i++) {
    const int fi = finite_max_index[i];
    phi[i + numFiniteMin] = joint_limit_max[fi] - q[fi];
    J(i + numFiniteMin, fi) = -1.0;
  }
}

size_t RigidBodyTree::getNumJointLimitConstraints() const {
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  return finite_min_index.size() + finite_max_index.size();
}

size_t RigidBodyTree::getNumPositionConstraints() const {
  return loops.size() * 6;
}

void RigidBodyTree::addFrame(std::shared_ptr<RigidBodyFrame> frame) {
  frames.push_back(frame);
  frame->frame_index = -(static_cast<int>(frames.size()) - 1) - 2;  // yuck!!
}

void RigidBodyTree::add_rigid_body(std::unique_ptr<RigidBody> body) {
  // TODO(amcastro-tri): body indexes should not be initialized here but on an
  // initialize call after all bodies and RigidBodySystem's are defined.
  // This initialize call will make sure that all global and local indexes are
  // properly computed taking into account a RigidBodySystem could be part of a
  // larger RigidBodySystem (a system within a tree of systems).
  body->body_index = static_cast<int>(bodies.size());

  // bodies will be sorted by SortTree by generation. Therefore bodies[0]
  // (world) will be at the top and subsequent generations of children will
  // follow.
  bodies.push_back(std::move(body));
}

int RigidBodyTree::AddFloatingJoint(
    const DrakeJoint::FloatingBaseType floating_base_type,
    const std::vector<int>& link_indices,
    const std::shared_ptr<RigidBodyFrame> weld_to_frame,
    const PoseMap* pose_map) {
  std::string floating_joint_name;
  RigidBody* weld_to_body{nullptr};
  Eigen::Isometry3d transform_to_world;

  if (weld_to_frame == nullptr) {
    // If weld_to_frame is not specified, weld the newly added model(s) to the
    // world with zero offset.
    weld_to_body = bodies[0].get();
    floating_joint_name = "base";
    transform_to_world = Eigen::Isometry3d::Identity();
  } else {
    // If weld_to_frame is specified and the model is being welded to the world,
    // ensure the "body" variable within weld_to_frame is nullptr. Then, only
    // use the transform_to_body variable within weld_to_frame to initialize
    // the robot at the desired location in the world.
    if (weld_to_frame->name == std::string(RigidBodyTree::kWorldLinkName)) {
      if (weld_to_frame->body != nullptr) {
        throw std::runtime_error(
            "RigidBodyTree::AddFloatingJoint: "
            "Attempted to weld robot to the world while specifying a body "
            "link!");
      }
      weld_to_body = bodies[0].get();  // the world's body
      floating_joint_name = "base";
    } else {
      weld_to_body = weld_to_frame->body;
      floating_joint_name = "weld";
    }
    transform_to_world = weld_to_frame->transform_to_body;
  }

  int num_floating_joints_added = 0;

  for (auto i : link_indices) {
    if (bodies[i]->parent == nullptr) {
      // The following code connects the parent-less link to the rigid body tree
      // using a floating joint.
      bodies[i]->parent = weld_to_body;

      Eigen::Isometry3d transform_to_model = Eigen::Isometry3d::Identity();
      if (pose_map != nullptr &&
          pose_map->find(bodies[i]->name_) != pose_map->end())
        transform_to_model = pose_map->at(bodies[i]->name_);

      switch (floating_base_type) {
        case DrakeJoint::FIXED: {
          std::unique_ptr<DrakeJoint> joint(new FixedJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case DrakeJoint::ROLLPITCHYAW: {
          std::unique_ptr<DrakeJoint> joint(new RollPitchYawFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        case DrakeJoint::QUATERNION: {
          std::unique_ptr<DrakeJoint> joint(new QuaternionFloatingJoint(
              floating_joint_name, transform_to_world * transform_to_model));
          bodies[i]->setJoint(move(joint));
          num_floating_joints_added++;
        } break;
        default:
          throw std::runtime_error("unknown floating base type");
      }
    }
  }

  if (num_floating_joints_added == 0) {
    throw std::runtime_error(
        "No root links found (every link in the rigid body model has a joint "
        "connecting it to some other joint).  You're about to loop "
        "indefinitely in the compile() method.  Still need to handle this "
        "case.");
    // could handle it by disconnecting one of the internal nodes, making that a
    // loop joint, and connecting the new free joint to the world
  }

  return num_floating_joints_added;
}

// Explicit template instantiations for massMatrix.
template DRAKERBM_EXPORT MatrixX<AutoDiffUpTo73d> RigidBodyTree::massMatrix<
    AutoDiffUpTo73d>(KinematicsCache<AutoDiffUpTo73d>&) const;
template DRAKERBM_EXPORT MatrixX<AutoDiffXd>
RigidBodyTree::massMatrix<AutoDiffXd>(KinematicsCache<AutoDiffXd>&) const;
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::massMatrix<double>(KinematicsCache<double>&) const;

// Explicit template instantiations for centerOfMass.
template DRAKERBM_EXPORT Vector3<AutoDiffUpTo73d> RigidBodyTree::centerOfMass<
    AutoDiffUpTo73d>(KinematicsCache<AutoDiffUpTo73d>&,
                     set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Vector3<AutoDiffXd> RigidBodyTree::centerOfMass<
    AutoDiffXd>(KinematicsCache<AutoDiffXd>&,
                set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Vector3d RigidBodyTree::centerOfMass<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for dynamicsBiasTerm.
template DRAKERBM_EXPORT VectorX<AutoDiffUpTo73d>
RigidBodyTree::dynamicsBiasTerm<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    unordered_map<
        RigidBody const*, TwistVector<AutoDiffUpTo73d>, hash<RigidBody const*>,
        equal_to<RigidBody const*>,
        Eigen::aligned_allocator<
            pair<RigidBody const* const, TwistVector<AutoDiffUpTo73d>>>> const&,
    bool) const;
template DRAKERBM_EXPORT VectorX<AutoDiffXd>
RigidBodyTree::dynamicsBiasTerm<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    unordered_map<RigidBody const*, TwistVector<AutoDiffXd>,
                  hash<RigidBody const*>, equal_to<RigidBody const*>,
                  Eigen::aligned_allocator<pair<
                      RigidBody const* const, TwistVector<AutoDiffXd>>>> const&,
    bool) const;
template DRAKERBM_EXPORT VectorXd RigidBodyTree::dynamicsBiasTerm<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody const*, TwistVector<double>, hash<RigidBody const*>,
                  equal_to<RigidBody const*>,
                  Eigen::aligned_allocator<pair<RigidBody const* const,
                                                TwistVector<double>>>> const&,
    bool) const;

// Explicit template instantiations for geometricJacobian.
template DRAKERBM_EXPORT TwistMatrix<AutoDiffUpTo73d>
RigidBodyTree::geometricJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int, bool,
    vector<int, allocator<int>>*) const;
template DRAKERBM_EXPORT TwistMatrix<AutoDiffXd>
RigidBodyTree::geometricJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int, bool,
    vector<int, allocator<int>>*) const;
template DRAKERBM_EXPORT TwistMatrix<double>
RigidBodyTree::geometricJacobian<double>(KinematicsCache<double> const&, int,
                                         int, int, bool,
                                         vector<int, allocator<int>>*) const;

// Explicit template instantiations for relativeTransform.
template DRAKERBM_EXPORT Eigen::Transform<AutoDiffUpTo73d, 3, 1, 0>
RigidBodyTree::relativeTransform<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;
template DRAKERBM_EXPORT Eigen::Transform<AutoDiffXd, 3, 1, 0>
RigidBodyTree::relativeTransform<AutoDiffXd>(KinematicsCache<AutoDiffXd> const&,
                                             int, int) const;
template DRAKERBM_EXPORT Eigen::Transform<double, 3, 1, 0>
RigidBodyTree::relativeTransform<double>(KinematicsCache<double> const&, int,
                                         int) const;

// Explicit template instantiations for centerOfMassJacobian.
template DRAKERBM_EXPORT Matrix3X<AutoDiffUpTo73d>
RigidBodyTree::centerOfMassJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT Matrix3X<AutoDiffXd>
RigidBodyTree::centerOfMassJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template DRAKERBM_EXPORT Matrix3Xd RigidBodyTree::centerOfMassJacobian<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;

// Explicit template instantiations for centroidalMomentumMatrix.
template DRAKERBM_EXPORT TwistMatrix<AutoDiffUpTo73d>
RigidBodyTree::centroidalMomentumMatrix<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT TwistMatrix<AutoDiffXd>
RigidBodyTree::centroidalMomentumMatrix<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template DRAKERBM_EXPORT TwistMatrix<double>
RigidBodyTree::centroidalMomentumMatrix<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;

// Explicit template instantiations for forwardKinPositionGradient.
template DRAKERBM_EXPORT MatrixX<AutoDiffUpTo73d>
RigidBodyTree::forwardKinPositionGradient<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int) const;
template DRAKERBM_EXPORT MatrixX<AutoDiffXd>
RigidBodyTree::forwardKinPositionGradient<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int) const;
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::forwardKinPositionGradient<double>(
    KinematicsCache<double> const&, int, int, int) const;

// Explicit template instantiations for geometricJacobianDotTimesV.
template DRAKERBM_EXPORT TwistVector<AutoDiffUpTo73d>
RigidBodyTree::geometricJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, int) const;
template DRAKERBM_EXPORT TwistVector<AutoDiffXd>
RigidBodyTree::geometricJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, int) const;
template DRAKERBM_EXPORT TwistVector<double>
RigidBodyTree::geometricJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int, int) const;

// Explicit template instantiations for centerOfMassJacobianDotTimesV.
template DRAKERBM_EXPORT Vector3<AutoDiffUpTo73d>
RigidBodyTree::centerOfMassJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Vector3<AutoDiffXd>
RigidBodyTree::centerOfMassJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Vector3d
RigidBodyTree::centerOfMassJacobianDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for centroidalMomentumMatrixDotTimesV.
template DRAKERBM_EXPORT TwistVector<AutoDiffUpTo73d>
RigidBodyTree::centroidalMomentumMatrixDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d>&,
    set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT TwistVector<AutoDiffXd>
RigidBodyTree::centroidalMomentumMatrixDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd>&,
    set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT TwistVector<double>
RigidBodyTree::centroidalMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT VectorXd RigidBodyTree::positionConstraints<double>(
    KinematicsCache<double> const&) const;

// Explicit template instantiations for positionConstraintsJacobian.
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::positionConstraintsJacobian<double>(
    KinematicsCache<double> const&, bool) const;

// Explicit template instantiations for positionConstraintsJacDotTimesV.
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::positionConstraintsJacDotTimesV<double>(
    KinematicsCache<double> const&) const;
template DRAKERBM_EXPORT void RigidBodyTree::jointLimitConstraints<
    VectorXd, VectorXd, MatrixXd>(Eigen::MatrixBase<VectorXd> const&,
                                  Eigen::MatrixBase<VectorXd>&,
                                  Eigen::MatrixBase<MatrixXd>&) const;

// Explicit template instantiations for relativeTwist.
template DRAKERBM_EXPORT TwistVector<double>
RigidBodyTree::relativeTwist<double>(KinematicsCache<double> const&, int, int,
                                     int) const;

// Explicit template instantiations for worldMomentumMatrix.
template DRAKERBM_EXPORT TwistMatrix<double> RigidBodyTree::worldMomentumMatrix<
    double>(KinematicsCache<double>&,
            set<int, less<int>, allocator<int>> const&, bool) const;

// Explicit template instantiations for worldMomentumMatrixDotTimesV.
template DRAKERBM_EXPORT TwistVector<double>
RigidBodyTree::worldMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;

// Explicit template instantiations for transformSpatialAcceleration.
template DRAKERBM_EXPORT TwistVector<double>
RigidBodyTree::transformSpatialAcceleration<double>(
    KinematicsCache<double> const&, TwistVector<double> const&, int, int, int,
    int) const;

// Explicit template instantiations for inverseDynamics.
template DRAKERBM_EXPORT VectorXd RigidBodyTree::inverseDynamics<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody const*, TwistVector<double>, hash<RigidBody const*>,
                  equal_to<RigidBody const*>,
                  Eigen::aligned_allocator<pair<RigidBody const* const,
                                                TwistVector<double>>>> const&,
    VectorXd const&, bool) const;

// Explicit template instantiations for jointLimitConstraints.
template DRAKERBM_EXPORT void RigidBodyTree::jointLimitConstraints<
    Eigen::Map<VectorXd>, Eigen::Map<VectorXd>, Eigen::Map<MatrixXd>>(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd>>&,
    Eigen::MatrixBase<Eigen::Map<MatrixXd>>&) const;

// Explicit template instantiations for resolveCenterOfPressure.
template DRAKERBM_EXPORT pair<Vector3d, double>
RigidBodyTree::resolveCenterOfPressure<Vector3d, Vector3d>(
    KinematicsCache<double> const&,
    vector<ForceTorqueMeasurement, allocator<ForceTorqueMeasurement>> const&,
    Eigen::MatrixBase<Vector3d> const&,
    Eigen::MatrixBase<Vector3d> const&) const;

// Explicit template instantiations for transformPointsJacobian.
template DRAKERBM_EXPORT MatrixX<AutoDiffUpTo73d>
RigidBodyTree::transformPointsJacobian<AutoDiffUpTo73d, Matrix3Xd>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Matrix3Xd> const&, int, int, bool) const;
template DRAKERBM_EXPORT MatrixX<AutoDiffXd>
RigidBodyTree::transformPointsJacobian<AutoDiffXd, Matrix3Xd>(
    KinematicsCache<AutoDiffXd> const&, Eigen::MatrixBase<Matrix3Xd> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::transformPointsJacobian<double, Matrix3Xd>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int,
    int, bool) const;
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::transformPointsJacobian<double, Vector3d>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Vector3d> const&, int,
    int, bool) const;
template DRAKERBM_EXPORT MatrixXd RigidBodyTree::transformPointsJacobian<
    double, Eigen::Block<Matrix3Xd, 3, 1, true>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Block<Matrix3Xd, 3, 1, true>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT MatrixX<AutoDiffUpTo73d>
RigidBodyTree::transformPointsJacobian<AutoDiffUpTo73d,
                                       Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT MatrixX<AutoDiffXd>
RigidBodyTree::transformPointsJacobian<AutoDiffXd, Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffXd> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT MatrixXd
RigidBodyTree::transformPointsJacobian<double, Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int,
    bool) const;

// Explicit template instantiations for transformPointsJacobianDotTimesV.
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::transformPointsJacobianDotTimesV<double, Matrix3Xd>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int,
    int) const;
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::transformPointsJacobianDotTimesV<double, Vector3d>(
    KinematicsCache<double> const&, Eigen::MatrixBase<Vector3d> const&, int,
    int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffUpTo73d>
RigidBodyTree::transformPointsJacobianDotTimesV<AutoDiffUpTo73d,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffUpTo73d> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffXd>
RigidBodyTree::transformPointsJacobianDotTimesV<AutoDiffXd,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<AutoDiffXd> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::transformPointsJacobianDotTimesV<double,
                                                Eigen::Map<Matrix3Xd const>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Matrix3Xd const>> const&, int, int) const;

// Explicit template instantiations for relativeQuaternionJacobian.
template DRAKERBM_EXPORT Matrix4Xd
RigidBodyTree::relativeQuaternionJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template DRAKERBM_EXPORT Matrix4X<AutoDiffUpTo73d>
RigidBodyTree::relativeQuaternionJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, bool) const;
template DRAKERBM_EXPORT Matrix4X<AutoDiffXd>
RigidBodyTree::relativeQuaternionJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, bool) const;

// Explicit template instantiations for relativeRollPitchYawJacobian.
template DRAKERBM_EXPORT Matrix3Xd
RigidBodyTree::relativeRollPitchYawJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template DRAKERBM_EXPORT Matrix3X<AutoDiffUpTo73d>
RigidBodyTree::relativeRollPitchYawJacobian<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int, bool) const;
template DRAKERBM_EXPORT Matrix3X<AutoDiffXd>
RigidBodyTree::relativeRollPitchYawJacobian<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int, bool) const;

// Explicit template instantiations for relativeRollPitchYawJacobianDotTimesV.
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffXd>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffUpTo73d>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;

// Explicit template instantiations for relativeQuaternionJacobianDotTimesV.
template DRAKERBM_EXPORT VectorXd
RigidBodyTree::relativeQuaternionJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffUpTo73d>
RigidBodyTree::relativeQuaternionJacobianDotTimesV<AutoDiffUpTo73d>(
    KinematicsCache<AutoDiffUpTo73d> const&, int, int) const;
template DRAKERBM_EXPORT VectorX<AutoDiffXd>
RigidBodyTree::relativeQuaternionJacobianDotTimesV<AutoDiffXd>(
    KinematicsCache<AutoDiffXd> const&, int, int) const;

// Explicit template instantiations for doKinematics(cache).
template DRAKERBM_EXPORT void RigidBodyTree::doKinematics(
    KinematicsCache<double>&, bool) const;
template DRAKERBM_EXPORT void RigidBodyTree::doKinematics(
    KinematicsCache<AutoDiffXd>&, bool) const;
template DRAKERBM_EXPORT void RigidBodyTree::doKinematics(
    KinematicsCache<AutoDiffUpTo73d>&, bool) const;

// Explicit template instantiations for doKinematics(q).
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<VectorXd> const&) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Block<MatrixXd const, -1, 1, true>> const&) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Block<MatrixXd, -1, 1, true>> const&) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&) const;
template DRAKERBM_EXPORT KinematicsCache<AutoDiffXd>
RigidBodyTree::doKinematics(
    Eigen::MatrixBase<VectorX<AutoDiffXd>> const&) const;

// Explicit template instantiations for doKinematics(q, v).
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<VectorXd> const&, Eigen::MatrixBase<VectorXd> const&,
    bool) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&,
    Eigen::MatrixBase<Eigen::Block<VectorXd const, -1, 1, false>> const&,
    bool) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Block<VectorXd, -1, 1, false>> const&,
    Eigen::MatrixBase<Eigen::Block<VectorXd, -1, 1, false>> const&,
    bool) const;
template DRAKERBM_EXPORT KinematicsCache<AutoDiffXd>
RigidBodyTree::doKinematics(Eigen::MatrixBase<VectorX<AutoDiffXd>> const&,
                            Eigen::MatrixBase<VectorX<AutoDiffXd>> const&,
                            bool) const;
template DRAKERBM_EXPORT KinematicsCache<AutoDiffUpTo73d>
RigidBodyTree::doKinematics(Eigen::MatrixBase<VectorX<AutoDiffUpTo73d>> const&,
                            Eigen::MatrixBase<VectorX<AutoDiffUpTo73d>> const&,
                            bool) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd>> const&, bool) const;
template DRAKERBM_EXPORT KinematicsCache<double> RigidBodyTree::doKinematics(
    Eigen::MatrixBase<Eigen::Map<VectorXd const>> const&,
    Eigen::MatrixBase<Eigen::Map<VectorXd const>> const&, bool) const;
