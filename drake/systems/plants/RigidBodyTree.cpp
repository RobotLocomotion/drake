#include <iostream>

#include "drake/util/drakeGeometryUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/joints/FixedJoint.h"
#include "drake/util/drakeUtil.h"

#include <algorithm>
#include <string>
#include <regex>
#include <limits>
#include "KinematicsCache.h"

#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

const set<int> RigidBodyTree::default_robot_num_set = {0};

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
     << " on " << obj.frameA->body->linkname << " to pt "
     << obj.frameB->transform_to_body.matrix().topRightCorner(3, 1).transpose()
     << " on " << obj.frameB->body->linkname << std::endl;
  return os;
}

RigidBodyTree::RigidBodyTree(
    const std::string& urdf_filename,
    const DrakeJoint::FloatingBaseType floating_base_type)
    : collision_model(DrakeCollision::newModel()) {
  a_grav << 0, 0, 0, 0, 0, -9.81;

  shared_ptr<RigidBody> b(new RigidBody());
  b->linkname = "world";
  b->robotnum = 0;
  b->body_index = 0;
  bodies.push_back(b);

  initialized = false;

  addRobotFromURDF(urdf_filename, floating_base_type);
}

RigidBodyTree::RigidBodyTree(void)
    : collision_model(DrakeCollision::newModel()) {
  a_grav << 0, 0, 0, 0, 0, -9.81;

  shared_ptr<RigidBody> b(new RigidBody());
  b->linkname = "world";
  b->robotnum = 0;
  b->body_index = 0;
  bodies.push_back(b);

  initialized = false;
}

RigidBodyTree::~RigidBodyTree(void) {}

void RigidBodyTree::compile(void) {
  // reorder body list to make sure that parents before children in the list
  size_t i = 0;
  while (i < bodies.size() - 1) {
    if (bodies[i]->hasParent()) {
      auto iter = find(bodies.begin() + i + 1, bodies.end(), bodies[i]->parent);
      if (iter != bodies.end()) {
        bodies.erase(iter);
        bodies.insert(bodies.begin() + i, bodies[i]->parent);
        i--;
      }
    }
    i++;
  }

  // weld joints for links that have zero inertia and no children (as seen in
  // pr2.urdf)
  for (size_t i = 0; i < bodies.size(); i++) {
    if (bodies[i]->hasParent() && bodies[i]->I.isConstant(0)) {
      bool hasChild = false;
      for (size_t j = i + 1; j < bodies.size(); j++) {
        if (bodies[j]->parent == bodies[i]) {
          hasChild = true;
          break;
        }
      }
      if (!hasChild) {
        // now check if this body is attached by a loop joint
        for (const auto& loop : loops) {
          if ((loop.frameA->body == bodies[i]) ||
              (loop.frameB->body == bodies[i])) {
            hasChild = true;
            break;
          }
        }
      }
      if (!hasChild) {
        cout << "welding " << bodies[i]->getJoint().getName()
             << " because it has no inertia beneath it" << endl;
        unique_ptr<DrakeJoint> joint_unique_ptr(
            new FixedJoint(bodies[i]->getJoint().getName(),
                           bodies[i]->getJoint().getTransformToParentBody()));
        bodies[i]->setJoint(move(joint_unique_ptr));
      }
    }
  }

  num_positions = 0;
  num_velocities = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      body.position_num_start = num_positions;
      num_positions += body.getJoint().getNumPositions();
      body.velocity_num_start = num_velocities;
      num_velocities += body.getJoint().getNumVelocities();
    } else {
      body.position_num_start = 0;
      body.velocity_num_start = 0;
    }
  }

  for (size_t i = 0; i < bodies.size(); i++) {
    bodies[i]->body_index = static_cast<int>(i);
  }

  B.resize(num_velocities, actuators.size());
  B = MatrixXd::Zero(num_velocities, actuators.size());
  for (size_t ia = 0; ia < actuators.size(); ia++)
    for (int i = 0; i < actuators[ia].body->getJoint().getNumVelocities(); i++)
      B(actuators[ia].body->velocity_num_start + i, ia) =
          actuators[ia].reduction;

  // gather joint limits in RBM vector
  joint_limit_min = VectorXd::Constant(
      num_positions, -std::numeric_limits<double>::infinity());
  joint_limit_max = VectorXd::Constant(num_positions,
                                       std::numeric_limits<double>::infinity());
  for (int i = 0; i < bodies.size(); i++) {
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

  updateStaticCollisionElements();

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    getTerrainContactPoints(body, body.contact_pts);
  }

  initialized = true;
}

Eigen::VectorXd RigidBodyTree::getZeroConfiguration() const {
  Eigen::VectorXd q(num_positions);
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
  Eigen::VectorXd q(num_positions);
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
  if (position_num < 0 || position_num >= num_positions)
    throw std::runtime_error("position_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
         bodies[body_index + 1]->position_num_start <= position_num)
    body_index++;

  return bodies[body_index]->getJoint().getPositionName(
      position_num - bodies[body_index]->position_num_start);
}

string RigidBodyTree::getVelocityName(int velocity_num) const {
  if (velocity_num < 0 || velocity_num >= num_velocities)
    throw std::runtime_error("velocity_num is out of range");

  size_t body_index = 0;
  while (body_index + 1 < bodies.size() &&
         bodies[body_index + 1]->velocity_num_start <= velocity_num)
    body_index++;

  return bodies[body_index]->getJoint().getVelocityName(
      velocity_num - bodies[body_index]->velocity_num_start);
}

string RigidBodyTree::getStateName(int state_num) const {
  if (state_num < num_positions)
    return getPositionName(state_num);
  else
    return getVelocityName(state_num - num_positions);
}

void RigidBodyTree::drawKinematicTree(std::string graphviz_dotfile_filename) {
  ofstream dotfile;
  dotfile.open(graphviz_dotfile_filename);
  dotfile << "digraph {" << endl;
  for (const auto& body : bodies) {
    dotfile << "  " << body->linkname << " [label=\"" << body->linkname << endl;
    dotfile << "mass=" << body->mass << ", com=" << body->com.transpose()
            << endl;
    dotfile << "inertia=" << endl
            << body->I << endl;
    dotfile << "\"];" << endl;
    if (body->hasParent()) {
      const auto& joint = body->getJoint();
      dotfile << "  " << body->linkname << " -> " << body->parent->linkname
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
    dotfile << "  " << frame->name << " -> " << frame->body->linkname
            << " [label=\"";
    dotfile << "transform_to_body=" << endl
            << frame->transform_to_body.matrix() << endl;
    dotfile << "\"];" << endl;
  }

  for (const auto& loop : loops) {
    dotfile << "  " << loop.frameA->body->linkname << " -> "
            << loop.frameB->body->linkname << " [label=\"loop " << endl;
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

  for (int i = 0; i < this->num_positions; ++i) {
    name_to_index_map[getPositionName(i)] = i;
  }
  return name_to_index_map;
}

DrakeCollision::ElementId RigidBodyTree::addCollisionElement(
    const RigidBody::CollisionElement& element,
    RigidBody& body, const string& group_name) {
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

  Vector3d ptA, ptB, n;
  double distance;
  for (int i = 0; i < closest_points.size(); ++i) {
    closest_points[i].getResults(ptA, ptB, n, distance);
    x.col(i) = ptB;
    body_x.col(i) = ptA;
    normal.col(i) = n;
    phi[i] = distance;
    const RigidBody::CollisionElement* elementB =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(closest_points[i].getIdB()));
    body_idx.push_back(elementB->getBody()->body_index);
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
  
  //Original code assuming convex shapes
#if 0
  bool points_found =
      collision_model->closestPointsAllToAll(ids_to_check, use_margins, points);
#endif

  points = collision_model->potentialCollisionPoints(use_margins);
  bool points_found = points.size() > 0;

  xA = MatrixXd::Zero(3, points.size());
  xB = MatrixXd::Zero(3, points.size());
  normal = MatrixXd::Zero(3, points.size());
  phi = VectorXd::Zero(points.size());

  Vector3d ptA, ptB, n;
  double distance;
  for (int i = 0; i < points.size(); ++i) {
    points[i].getResults(ptA, ptB, n, distance);
    xA.col(i) = ptA;
    xB.col(i) = ptB;
    normal.col(i) = n;
    phi[i] = distance;
    const RigidBody::CollisionElement* elementA =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(points[i].getIdA()));
    // DEBUG
    // cout << "RigidBodyTree::collisionDetect: points[i].getIdA() = " <<
    // points[i].getIdA() << endl;
    // cout << "RigidBodyTree::collisionDetect:
    // collision_model->readElement(points[i].getIdA()) = " <<
    // collision_model->readElement(points[i].getIdA()) << endl;
    // cout << "RigidBodyTree::collisionDetect:
    // collision_model->readElement(points[i].getIdA())->getId() = " <<
    // collision_model->readElement(points[i].getIdA())->getId() << endl;
    // cout << "RigidBodyTree::collisionDetect: elementA = " << elementA <<
    // endl;
    // END_DEBUG
    bodyA_idx.push_back(elementA->getBody()->body_index);
    const RigidBody::CollisionElement* elementB =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(points[i].getIdB()));
    bodyB_idx.push_back(elementB->getBody()->body_index);
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
                                    vector<int>& bodyB_idx, bool use_margins) { //use_margins = true by default
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

  Vector3d ptA, ptB, n;
  double distance;

  for (size_t i = 0; i < num_potential_collisions; i++) {
    const RigidBody::CollisionElement* elementA =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(potential_collisions[i].getIdA()));
    const RigidBody::CollisionElement* elementB =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(potential_collisions[i].getIdB()));
    potential_collisions[i].getResults(ptA, ptB, n, distance);
    xA.col(i) = ptA;
    xB.col(i) = ptB;
    normal.col(i) = n;
    phi[i] = distance;
    bodyA_idx.push_back(elementA->getBody()->body_index);
    bodyB_idx.push_back(elementB->getBody()->body_index);
  }
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
      collision_model->collisionPointsAllToAll(use_margins, points);

  xA_in_world = Matrix3Xd::Zero(3, points.size());
  xB_in_world = Matrix3Xd::Zero(3, points.size());

  Vector3d ptA, ptB, n;
  double distance;
  for (int i = 0; i < points.size(); ++i) {
    points[i].getResults(ptA, ptB, n, distance);
    xA_in_world.col(i) = ptA;
    xB_in_world.col(i) = ptB;

    const RigidBody::CollisionElement* elementA =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(points[i].getIdA()));
    bodyA_idx.push_back(elementA->getBody()->body_index);
    const RigidBody::CollisionElement* elementB =
        dynamic_cast<const RigidBody::CollisionElement*>(
            collision_model->readElement(points[i].getIdB()));
    bodyB_idx.push_back(elementB->getBody()->body_index);
  }
  return points_found;
}

void RigidBodyTree::warnOnce(const string& id, const string& msg) {
  auto print_warning_iter = already_printed_warnings.find(id);
  if (print_warning_iter == already_printed_warnings.end()) {
    cout << msg << endl;
    already_printed_warnings.insert(id);
  }
}

// bool RigidBodyTree::closestDistanceAllBodies(VectorXd& distance,
// MatrixXd& Jd)
//{
// MatrixXd ptsA, ptsB, normal, JA, JB;
// vector<int> bodyA_idx, bodyB_idx;
// bool return_val =
// closestPointsAllBodies(bodyA_idx, bodyB_idx, ptsA, ptsB, normal, distance,
// JA, JB, Jd);
// DEBUG
// cout << "RigidBodyTree::closestDistanceAllBodies: distance.size() = " <<
// distance.size() << endl;
// END_DEBUG
// return return_val;
//};

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

  int nq = num_positions;
  int nv = num_velocities;
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
  for (int i = 0; i < bodies.size(); i++) {
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

  for (int i = 0; i < bodies.size(); i++) {
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

template <typename Derived>
void RigidBodyTree::getContactPositions(
    const KinematicsCache<typename Derived::Scalar>& cache,
    MatrixBase<Derived>& pos, const set<int>& body_idx) const {
  // kinematics cache checks are already being done in forwardKin
  int n = 0, nc, nb = static_cast<int>(body_idx.size()), bi;
  if (nb == 0) nb = static_cast<int>(bodies.size());
  set<int>::iterator iter = body_idx.begin();
  for (int i = 0; i < nb; i++) {
    if (body_idx.size() == 0)
      bi = i;
    else
      bi = *iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc > 0) {
      pos.block(0, n, 3, nc) =
          forwardKin(cache, bodies[bi]->contact_pts, bi, 0, 0);
      n += nc;
    }
  }
}

template <typename Derived>
void RigidBodyTree::getContactPositionsJac(
    const KinematicsCache<typename Derived::Scalar>& cache,
    MatrixBase<Derived>& J, const set<int>& body_idx) const {
  // kinematics cache checks are already being done in forwardKinJacobian
  int n = 0, nc, nb = static_cast<int>(body_idx.size()), bi;
  if (nb == 0) nb = static_cast<int>(bodies.size());
  set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i = 0; i < nb; i++) {
    if (body_idx.size() == 0)
      bi = i;
    else
      bi = *iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc > 0) {
      p.resize(3 * nc, num_positions);
      J.block(3 * n, 0, 3 * nc, num_positions) =
          forwardKinJacobian(cache, bodies[bi]->contact_pts, bi, 0, 0, true, 0);
      n += nc;
    }
  }
}

/* [body_ind, Tframe] = parseBodyOrFrameID(body_or_frame_id) */
template <typename Scalar>
int RigidBodyTree::parseBodyOrFrameID(
    const int body_or_frame_id,
    Eigen::Transform<Scalar, 3, Isometry>* Tframe) const {
  int body_ind = 0;
  if (body_or_frame_id == -1) {
    cerr << "parseBodyOrFrameID got a -1, which should have been reserved for "
            "COM.  Shouldn't have gotten here." << endl;
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
    current_body = current_body->parent.get();
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
    stream << "There is no path between " << bodies[start_body]->linkname
           << " and " << bodies[end_body]->linkname << ".";
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

  int nv = num_velocities;
  Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(nv, nv);
  ret.setZero();

  updateCompositeRigidBodyInertias(cache);

  for (int i = 0; i < bodies.size(); i++) {
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
      shared_ptr<RigidBody> body_j(body_i.parent);
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
  Matrix<Scalar, Eigen::Dynamic, 1> vd(num_velocities, 1);
  vd.setZero();
  return inverseDynamics(cache, f_ext, vd, include_velocity_terms);
};

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

  for (int i = 0; i < bodies.size(); i++) {
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

  Matrix<Scalar, Eigen::Dynamic, 1> ret(num_velocities, 1);

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
  Matrix<Scalar, Dynamic, 1> ret(num_velocities, 1);

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
  int cols = in_terms_of_qdot ? num_positions : num_velocities;
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
      points_base.size(), cols);  // TODO: size at compile time
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
};

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
};

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
};

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
           J_geometric_dot_times_v.template topRows<SPACE_DIMENSION>())).eval();
  Jposdot_times_v_mat.colwise() +=
      J_geometric_dot_times_v.template bottomRows<SPACE_DIMENSION>();

  return Map<Matrix<Scalar, Dynamic, 1>>(Jposdot_times_v_mat.data(), r.size(),
                                         1);
};

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
};

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
};

shared_ptr<RigidBody> RigidBodyTree::findLink(std::string linkname,
                                              int robot) const {
  std::transform(linkname.begin(), linkname.end(), linkname.begin(),
                 ::tolower);  // convert to lower case

  // std::regex linkname_connector("[abc]");
  // cout<<"get linkname_connector"<<endl;
  // linkname = std::regex_replace(linkname, linkname_connector, string("_"));
  int match = -1;
  for (int i = 0; i < bodies.size(); i++) {
    // Note: unlike the MATLAB implementation, I don't have to handle the fixed
    // joint names
    string lower_linkname = bodies[i]->linkname;
    std::transform(lower_linkname.begin(), lower_linkname.end(),
                   lower_linkname.begin(),
                   ::tolower);                    // convert to lower case
    if (lower_linkname.compare(linkname) == 0) {  // the names match
      if (robot == -1 ||
          bodies[i]->robotnum == robot) {  // it's the right robot
        if (match < 0) {                   // it's the first match
          match = i;
        } else {
          cerr << "found multiple links named " << linkname << endl;
          return nullptr;
        }
      }
    }
  }
  if (match >= 0) return bodies[match];
  cerr << "could not find any links named " << linkname << endl;
  return nullptr;
}

shared_ptr<RigidBody> RigidBodyTree::findLink(std::string linkname,
                                              std::string model_name) const {
  std::transform(linkname.begin(), linkname.end(), linkname.begin(),
                 ::tolower);  // convert to lower case
  std::transform(model_name.begin(), model_name.end(), model_name.begin(),
                 ::tolower);  // convert to lower case

  // std::regex linkname_connector("[abc]");
  // cout<<"get linkname_connector"<<endl;
  // linkname = std::regex_replace(linkname, linkname_connector, string("_"));
  int match = -1;
  for (int i = 0; i < bodies.size(); i++) {
    // Note: unlike the MATLAB implementation, I don't have to handle the fixed
    // joint names
    string lower_linkname = bodies[i]->linkname;
    std::transform(lower_linkname.begin(), lower_linkname.end(),
                   lower_linkname.begin(),
                   ::tolower);                    // convert to lower case
    if (lower_linkname.compare(linkname) == 0) {  // the names match
      string lower_model_name = bodies[i]->model_name;
      std::transform(lower_model_name.begin(), lower_model_name.end(),
                     lower_model_name.begin(), ::tolower);
      if (model_name.empty() ||
          lower_model_name.compare(model_name) == 0) {  // it's the right robot
        if (match < 0) {                                // it's the first match
          match = i;
        } else {
          cerr << "found multiple links named " << linkname << endl;
          return nullptr;
        }
      }
    }
  }
  if (match >= 0) return bodies[match];
  cerr << "could not find any links named " << linkname << endl;
  return nullptr;
}

shared_ptr<RigidBodyFrame> RigidBodyTree::findFrame(
    std::string frame_name, std::string model_name) const {
  std::transform(frame_name.begin(), frame_name.end(), frame_name.begin(),
                 ::tolower);  // convert to lower case
  std::transform(model_name.begin(), model_name.end(), model_name.begin(),
                 ::tolower);  // convert to lower case

  int match = -1;
  for (int i = 0; i < frames.size(); i++) {
    string frame_name_lower = frames[i]->name;
    std::transform(frame_name_lower.begin(), frame_name_lower.end(),
                   frame_name_lower.begin(),
                   ::tolower);                        // convert to lower case
    if (frame_name_lower.compare(frame_name) == 0) {  // the names match
      string frame_model_name_lower = frames[i]->body->model_name;
      std::transform(frame_model_name_lower.begin(),
                     frame_model_name_lower.end(),
                     frame_model_name_lower.begin(), ::tolower);
      if (model_name.empty() ||
          frame_model_name_lower == model_name) {  // it's the right robot
        if (match < 0) {                           // it's the first match
          match = i;
        } else {
          cerr << "Error: found multiple frames named " << frame_name << endl;
          return nullptr;
        }
      }
    }
  }
  if (match >= 0) return frames[match];
  cerr << "Error: could not find a frame named " << frame_name << endl;
  return nullptr;
}

int RigidBodyTree::findLinkId(const std::string& name, int robot) const {
  shared_ptr<RigidBody> link = findLink(name, robot);
  if (link == nullptr)
    throw std::runtime_error("could not find link id: " + name);
  return link->body_index;
}

shared_ptr<RigidBody> RigidBodyTree::findJoint(std::string jointname,
                                               int robot) const {
  std::transform(jointname.begin(), jointname.end(), jointname.begin(),
                 ::tolower);  // convert to lower case

  vector<bool> name_match;
  name_match.resize(this->bodies.size());
  for (int i = 0; i < this->bodies.size(); i++) {
    if (bodies[i]->hasParent()) {
      string lower_jointname = this->bodies[i]->getJoint().getName();
      std::transform(lower_jointname.begin(), lower_jointname.end(),
                     lower_jointname.begin(),
                     ::tolower);  // convert to lower case
      if (lower_jointname.compare(jointname) == 0) {
        name_match[i] = true;
      } else {
        name_match[i] = false;
      }
    }
  }
  if (robot != -1) {
    for (int i = 0; i < this->bodies.size(); i++) {
      if (name_match[i]) {
        name_match[i] = this->bodies[i]->robotnum == robot;
      }
    }
  }
  // Unlike the MATLAB implementation, I am not handling the fixed joints
  int num_match = 0;
  int ind_match = -1;
  for (int i = 0; i < this->bodies.size(); i++) {
    if (name_match[i]) {
      num_match++;
      ind_match = i;
    }
  }
  if (num_match != 1) {
    cerr << "couldn't find unique joint " << jointname << endl;
    return (nullptr);
  } else {
    return this->bodies[ind_match];
  }
}

int RigidBodyTree::findJointId(const std::string& name, int robot) const {
  shared_ptr<RigidBody> link = findJoint(name, robot);
  if (link == nullptr)
    throw std::runtime_error("could not find joint id: " + name);
  return link->body_index;
}

std::string RigidBodyTree::getBodyOrFrameName(int body_or_frame_id) const {
  if (body_or_frame_id >= 0) {
    return bodies[body_or_frame_id]->linkname;
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
      6 * loops.size(), in_terms_of_qdot ? num_positions : num_velocities);

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
  J = MatrixXd::Zero(phi.size(), num_positions);
  for (int i = 0; i < numFiniteMin; i++) {
    const int fi = finite_min_index[i];
    phi[i] = q[fi] - joint_limit_min[fi];
    J(i, fi) = 1.0;
  }

  for (int i = 0; i < numFiniteMax; i++) {
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

template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::massMatrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::massMatrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::massMatrix<double>(KinematicsCache<double>&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  3, 1, 0, 3, 1>
    RigidBodyTree::centerOfMass<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  3, 1, 0, 3, 1>
    RigidBodyTree::centerOfMass<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 3, 1, 0, 3, 1>
RigidBodyTree::centerOfMass<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  -1, 1, 0, -1, 1>
    RigidBodyTree::dynamicsBiasTerm<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        unordered_map<
            RigidBody const*,
            Eigen::Matrix<
                Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                6, 1, 0, 6, 1>,
            hash<RigidBody const*>, equal_to<RigidBody const*>,
            Eigen::aligned_allocator<
                pair<RigidBody const* const,
                     Eigen::Matrix<Eigen::AutoDiffScalar<
                                       Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                                   6, 1, 0, 6, 1>>>> const&,
        bool) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  -1, 1, 0, -1, 1>
    RigidBodyTree::dynamicsBiasTerm<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        unordered_map<
            RigidBody const*,
            Eigen::Matrix<
                Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                6, 1, 0, 6, 1>,
            hash<RigidBody const*>, equal_to<RigidBody const*>,
            Eigen::aligned_allocator<
                pair<RigidBody const* const,
                     Eigen::Matrix<Eigen::AutoDiffScalar<
                                       Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                                   6, 1, 0, 6, 1>>>> const&,
        bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::dynamicsBiasTerm<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody const*, Eigen::Matrix<double, 6, 1, 0, 6, 1>,
                  hash<RigidBody const*>, equal_to<RigidBody const*>,
                  Eigen::aligned_allocator<
                      pair<RigidBody const* const,
                           Eigen::Matrix<double, 6, 1, 0, 6, 1>>>> const&,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, 6, -1, 0, 6,
    -1>
RigidBodyTree::geometricJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int, int, bool, vector<int, allocator<int>>*) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 6, -1, 0, 6,
    -1>
RigidBodyTree::geometricJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int, int, bool, vector<int, allocator<int>>*) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, -1, 0, 6, -1>
RigidBodyTree::geometricJacobian<double>(KinematicsCache<double> const&, int,
                                         int, int, bool,
                                         vector<int, allocator<int>>*) const;
template DRAKERBM_EXPORT Eigen::Transform<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, 3, 1, 0>
RigidBodyTree::relativeTransform<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Transform<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 3, 1, 0>
RigidBodyTree::relativeTransform<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Transform<double, 3, 1, 0>
RigidBodyTree::relativeTransform<double>(KinematicsCache<double> const&, int,
                                         int) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  3, -1, 0, 3, -1>
    RigidBodyTree::centerOfMassJacobian<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  3, -1, 0, 3, -1>
    RigidBodyTree::centerOfMassJacobian<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 3, -1, 0, 3, -1>
RigidBodyTree::centerOfMassJacobian<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  6, -1, 0, 6, -1>
    RigidBodyTree::centroidalMomentumMatrix<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  6, -1, 0, 6, -1>
    RigidBodyTree::centroidalMomentumMatrix<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        set<int, less<int>, allocator<int>> const&, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, -1, 0, 6, -1>
RigidBodyTree::centroidalMomentumMatrix<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::forwardKinPositionGradient<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::forwardKinPositionGradient<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::forwardKinPositionGradient<double>(
    KinematicsCache<double> const&, int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, 6, 1, 0, 6,
    1>
RigidBodyTree::geometricJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 6, 1, 0, 6,
    1>
RigidBodyTree::geometricJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, 1, 0, 6, 1>
RigidBodyTree::geometricJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int, int) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  3, 1, 0, 3, 1>
    RigidBodyTree::centerOfMassJacobianDotTimesV<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  3, 1, 0, 3, 1>
    RigidBodyTree::centerOfMassJacobianDotTimesV<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 3, 1, 0, 3, 1>
RigidBodyTree::centerOfMassJacobianDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
                  6, 1, 0, 6, 1>
    RigidBodyTree::centroidalMomentumMatrixDotTimesV<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT
    Eigen::Matrix<Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
                  6, 1, 0, 6, 1>
    RigidBodyTree::centroidalMomentumMatrixDotTimesV<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
        KinematicsCache<
            Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>&,
        set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, 1, 0, 6, 1>
RigidBodyTree::centroidalMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::positionConstraints<double>(
    KinematicsCache<double> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::positionConstraintsJacobian<double>(
    KinematicsCache<double> const&, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::positionConstraintsJacDotTimesV<double>(
    KinematicsCache<double> const&) const;
template DRAKERBM_EXPORT void
RigidBodyTree::jointLimitConstraints<Eigen::Matrix<double, -1, 1, 0, -1, 1>,
                                     Eigen::Matrix<double, -1, 1, 0, -1, 1>,
                                     Eigen::Matrix<double, -1, -1, 0, -1, -1>>(
    Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1>> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1>>&,
    Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1>>&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, 1, 0, 6, 1>
RigidBodyTree::relativeTwist<double>(KinematicsCache<double> const&, int, int,
                                     int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, -1, 0, 6, -1>
RigidBodyTree::worldMomentumMatrix<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, 1, 0, 6, 1>
RigidBodyTree::transformSpatialAcceleration<double>(
    KinematicsCache<double> const&, Eigen::Matrix<double, 6, 1, 0, 6, 1> const&,
    int, int, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 6, 1, 0, 6, 1>
RigidBodyTree::worldMomentumMatrixDotTimesV<double>(
    KinematicsCache<double>&, set<int, less<int>, allocator<int>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::inverseDynamics<double>(
    KinematicsCache<double>&,
    unordered_map<RigidBody const*, Eigen::Matrix<double, 6, 1, 0, 6, 1>,
                  hash<RigidBody const*>, equal_to<RigidBody const*>,
                  Eigen::aligned_allocator<
                      pair<RigidBody const* const,
                           Eigen::Matrix<double, 6, 1, 0, 6, 1>>>> const&,
    Eigen::Matrix<double, -1, 1, 0, -1, 1> const&, bool) const;
template DRAKERBM_EXPORT void RigidBodyTree::jointLimitConstraints<
    Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0, Eigen::Stride<0, 0>>,
    Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0, Eigen::Stride<0, 0>>,
    Eigen::Map<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
               Eigen::Stride<0, 0>>>(
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                                 Eigen::Stride<0, 0>>> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0,
                                 Eigen::Stride<0, 0>>>&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, -1, -1, 0, -1, -1>, 0,
                                 Eigen::Stride<0, 0>>>&) const;
template DRAKERBM_EXPORT pair<Eigen::Matrix<double, 3, 1, 0, 3, 1>, double>
RigidBodyTree::resolveCenterOfPressure<Eigen::Matrix<double, 3, 1, 0, 3, 1>,
                                       Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
    KinematicsCache<double> const&,
    vector<ForceTorqueMeasurement, allocator<ForceTorqueMeasurement>> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>> const&) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::transformPointsJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
    Eigen::Matrix<double, 3, -1, 0, 3, -1>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, -1, 0, 3, -1>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::transformPointsJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
    Eigen::Matrix<double, 3, -1, 0, 3, -1>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, -1, 0, 3, -1>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::transformPointsJacobian<double,
                                       Eigen::Matrix<double, 3, -1, 0, 3, -1>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, -1, 0, 3, -1>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::transformPointsJacobianDotTimesV<
    double, Eigen::Matrix<double, 3, -1, 0, 3, -1>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, -1, 0, 3, -1>> const&, int,
    int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 4, -1, 0, 4, -1>
RigidBodyTree::relativeQuaternionJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, 3, -1, 0, 3, -1>
RigidBodyTree::relativeRollPitchYawJacobian<double>(
    KinematicsCache<double> const&, int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::relativeQuaternionJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<double>(
    KinematicsCache<double> const&, int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::transformPointsJacobian<double,
                                       Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>> const&, int, int,
    bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::transformPointsJacobianDotTimesV<
    double, Eigen::Matrix<double, 3, 1, 0, 3, 1>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Matrix<double, 3, 1, 0, 3, 1>> const&, int,
    int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::transformPointsJacobian<
    double, Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1, true>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 3, -1, 0, 3, -1>, 3, 1,
                                   true>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::transformPointsJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
    Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::Stride<0, 0>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, -1, 0,
    -1, -1>
RigidBodyTree::transformPointsJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
    Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::Stride<0, 0>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, -1, 0, -1, -1>
RigidBodyTree::transformPointsJacobian<
    double, Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
                       Eigen::Stride<0, 0>>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, 4, -1, 0, 4,
    -1>
RigidBodyTree::relativeQuaternionJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 4, -1, 0, 4,
    -1>
RigidBodyTree::relativeQuaternionJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, 3, -1, 0, 3,
    -1>
RigidBodyTree::relativeRollPitchYawJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, 3, -1, 0, 3,
    -1>
RigidBodyTree::relativeRollPitchYawJacobian<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int, bool) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::transformPointsJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>,
    Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::Stride<0, 0>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::transformPointsJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>,
    Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::Stride<0, 0>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<double, -1, 1, 0, -1, 1>
RigidBodyTree::transformPointsJacobianDotTimesV<
    double, Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
                       Eigen::Stride<0, 0>>>(
    KinematicsCache<double> const&,
    Eigen::MatrixBase<Eigen::Map<Eigen::Matrix<double, 3, -1, 0, 3, -1> const,
                                 0, Eigen::Stride<0, 0>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::relativeQuaternionJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::relativeQuaternionJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1>>> const&,
    int, int) const;
template DRAKERBM_EXPORT Eigen::Matrix<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>, -1, 1, 0, -1,
    1>
RigidBodyTree::relativeRollPitchYawJacobianDotTimesV<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>>(
    KinematicsCache<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1>>> const&,
    int, int) const;
