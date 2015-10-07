#include <iostream>
#include <map>

#include "drakeGeometryUtil.h"
#include "RigidBodyManipulator.h"
#include "DrakeJoint.h"
#include "FixedJoint.h"
#include "drakeUtil.h"

#include <algorithm>
#include <string>
#include <regex>
#include <limits>
#include "RigidBodyConstraint.h"
#include "KinematicsCache.h"

using namespace std;
using namespace Eigen;

std::set<int> emptyIntSet;


template <typename T>
void getFiniteIndexes(T const & v, std::vector<int> &finite_indexes)
{
  finite_indexes.clear();
  const size_t n = v.size();
  for (int x = 0; x < n; x++)
  {
    if (std::isfinite(static_cast<double>(v[x]))) {
      finite_indexes.push_back(x);
    }
  }
}

std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj)
{
  os << "loop connects pt " << obj.frameA->transform_to_body.topRightCorner(3,1).transpose() << " on " << obj.frameA->body->linkname << " to pt " << obj.frameB->transform_to_body.topRightCorner(3,1).transpose() << " on " << obj.frameB->body->linkname << std::endl;
  return os;
}

RigidBodyManipulator::RigidBodyManipulator(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type)
  :  collision_model(DrakeCollision::newModel())
{
  a_grav << 0, 0, 0, 0, 0, -9.81;

  shared_ptr<RigidBody> b = make_shared<RigidBody>();
  b->linkname = "world";
  b->robotnum = 0;
  b->body_index = 0;
  bodies.push_back(b);

  initialized = false;

  addRobotFromURDF(urdf_filename,floating_base_type);
}

RigidBodyManipulator::RigidBodyManipulator(void) :
    collision_model(DrakeCollision::newModel())
{
  a_grav << 0, 0, 0, 0, 0, -9.81;

  shared_ptr<RigidBody> b = make_shared<RigidBody>();
  b->linkname = "world";
  b->robotnum = 0;
  b->body_index = 0;
  bodies.push_back(b);

  initialized = false;
}

RigidBodyManipulator::~RigidBodyManipulator(void)
{
}


void RigidBodyManipulator::compile(void)
{
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

  // weld joints for links that have zero inertia and no children (as seen in pr2.urdf)
  for (size_t i=0; i<bodies.size(); i++) {
    if (bodies[i]->hasParent() && bodies[i]->I.isConstant(0)) {
      bool hasChild = false;
      for (size_t j=i+1; j<bodies.size(); j++) {
        if (bodies[j]->parent == bodies[i]) {
          hasChild = true;
          break;
        }
      }
      if (!hasChild) {
        // now check if this body is attached by a loop joint
        for (const auto &loop : loops) {
          if ((loop.frameA->body == bodies[i]) || (loop.frameB->body == bodies[i])) {
            hasChild = true;
            break;
          }
        }
      }
      if (!hasChild) {
      	cout << "welding " << bodies[i]->getJoint().getName() << " because it has no inertia beneath it" << endl;
        unique_ptr<DrakeJoint> joint_unique_ptr(new FixedJoint(bodies[i]->getJoint().getName(), bodies[i]->getJoint().getTransformToParentBody()));
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
    }
    else {
      body.position_num_start = 0;
      body.velocity_num_start = 0;
    }
  }

  for (size_t i=0; i<bodies.size(); i++) {
    bodies[i]->body_index = static_cast<int>(i);
  }

  B.resize(num_velocities,actuators.size());
  B = MatrixXd::Zero(num_velocities,actuators.size());
  for (size_t ia=0; ia<actuators.size(); ia++)
    for (int i=0; i<actuators[ia].body->getJoint().getNumVelocities(); i++)
      B(actuators[ia].body->velocity_num_start+i,ia) = actuators[ia].reduction;

  // gather joint limits in RBM vector
  joint_limit_min = VectorXd::Constant(num_positions, -std::numeric_limits<double>::infinity());
  joint_limit_max = VectorXd::Constant(num_positions, std::numeric_limits<double>::infinity());
  for (int i = 0; i < bodies.size(); i++) {
    auto& body = bodies[i];
    if (body->hasParent()) {
      const DrakeJoint& joint = body->getJoint();
      joint_limit_min.segment(body->position_num_start, joint.getNumPositions()) = joint.getJointLimitMin();
      joint_limit_max.segment(body->position_num_start, joint.getNumPositions()) = joint.getJointLimitMax();
    }
  }

  updateStaticCollisionElements();

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    getTerrainContactPoints(body, body.contact_pts);
  }

  initialized = true;
}

void RigidBodyManipulator::getRandomConfiguration(Eigen::VectorXd& q, std::default_random_engine& generator) const
{
	for (int i=0; i<bodies.size(); i++) {
		if (bodies[i]->hasParent()) {
			const DrakeJoint& joint = bodies[i]->getJoint();
			q.middleRows(bodies[i]->position_num_start,joint.getNumPositions()) = joint.randomConfiguration(generator);
		}
	}
}

string RigidBodyManipulator::getPositionName(int position_num) const
{
	if (position_num<0 || position_num>=num_positions)
		throw std::runtime_error("position_num is out of range");

	size_t body_index = 0;
	while (body_index+1<bodies.size() && bodies[body_index+1]->position_num_start<=position_num) body_index++;

	return bodies[body_index]->getJoint().getPositionName(position_num-bodies[body_index]->position_num_start);
}

string RigidBodyManipulator::getVelocityName(int velocity_num) const
{
	if (velocity_num<0 || velocity_num>=num_velocities)
		throw std::runtime_error("velocity_num is out of range");

	size_t body_index = 0;
	while (body_index+1<bodies.size() && bodies[body_index+1]->velocity_num_start<=velocity_num) body_index++;

	return bodies[body_index]->getJoint().getVelocityName(velocity_num-bodies[body_index]->velocity_num_start);
}

string RigidBodyManipulator::getStateName(int state_num) const
{
	if (state_num<num_positions)
		return getPositionName(state_num);
	else
		return getVelocityName(state_num - num_positions);
}

map<string, int> RigidBodyManipulator::computePositionNameToIndexMap() const
{
  map<string, int> name_to_index_map;

  for (int i = 0; i < this->num_positions; ++i) {
    name_to_index_map[getPositionName(i)] = i;
  }
  return name_to_index_map;
}

DrakeCollision::ElementId RigidBodyManipulator::addCollisionElement(const RigidBody::CollisionElement& element, const shared_ptr<RigidBody>& body, string group_name)
{
  DrakeCollision::ElementId id(collision_model->addElement(element));
  if (id != 0) {
    body->collision_element_ids.push_back(id);
    body->collision_element_groups[group_name].push_back(id);
  }
  return id;
}

void RigidBodyManipulator::updateCollisionElements(const RigidBody& body, const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world)
{
  for (auto id_iter = body.collision_element_ids.begin(); id_iter != body.collision_element_ids.end(); ++id_iter) {
    collision_model->updateElementWorldTransform(*id_iter, transform_to_world.matrix());
  }
}

void RigidBodyManipulator::updateStaticCollisionElements()
{
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (!body.hasParent()) {
      updateCollisionElements(body, Isometry3d::Identity());
    }
  }
}

void RigidBodyManipulator::updateDynamicCollisionElements(const KinematicsCache<double>& cache)
{
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    const RigidBody& body = **it;
    if (body.hasParent()) {
      updateCollisionElements(body, cache.getElement(body).transform_to_world);
    }
  }
  collision_model->updateModel();
}

void RigidBodyManipulator::getTerrainContactPoints(const RigidBody& body, Eigen::Matrix3Xd &terrain_points) const
{
  // clear matrix before filling it again
  size_t num_points = 0;
  terrain_points.resize(Eigen::NoChange,0);

  for (auto id_iter = body.collision_element_ids.begin(); id_iter != body.collision_element_ids.end();++id_iter) {

    Matrix3Xd element_points;
    collision_model->getTerrainContactPoints(*id_iter, element_points);
    terrain_points.conservativeResize(Eigen::NoChange, terrain_points.cols() + element_points.cols());
    terrain_points.block(0, num_points, terrain_points.rows(), element_points.cols()) = element_points;
    num_points += element_points.cols();
  }
}

bool RigidBodyManipulator::collisionRaycast(const KinematicsCache<double>& cache,
                                            const Matrix3Xd &origins,
                                            const Matrix3Xd &ray_endpoints,
                                            VectorXd &distances,
                                            bool use_margins )
{
  updateDynamicCollisionElements(cache);
  return collision_model->collisionRaycast(origins, ray_endpoints, use_margins, distances);
}


bool RigidBodyManipulator::collisionDetect(const KinematicsCache<double>& cache,
                                           VectorXd& phi,
                                           Matrix3Xd& normal,
                                           Matrix3Xd& xA,
                                           Matrix3Xd& xB,
                                           vector<int>& bodyA_idx,
                                           vector<int>& bodyB_idx,
                                           const vector<DrakeCollision::ElementId>& ids_to_check,
                                           bool use_margins)
{
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  //DEBUG
  //cout << "RigidBodyManipulator::collisionDetect: calling collision_model->closestPointsAllToAll" << endl;
  //END_DEBUG
  bool points_found = collision_model->closestPointsAllToAll(ids_to_check, use_margins, points);
  //DEBUG
  //cout << "RigidBodyManipulator::collisionDetect: points.size() = " << points.size() << endl;
  //END_DEBUG

  xA = MatrixXd::Zero(3,points.size());
  xB = MatrixXd::Zero(3,points.size());
  normal = MatrixXd::Zero(3,points.size());
  phi = VectorXd::Zero(points.size());

  Vector3d ptA, ptB, n;
  double distance;
  for (int i=0; i<points.size(); ++i) {
    points[i].getResults(ptA, ptB, n, distance);
    xA.col(i) = ptA;
    xB.col(i) = ptB;
    normal.col(i) = n;
    phi[i] = distance;
    const RigidBody::CollisionElement* elementA = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(points[i].getIdA()));
    //DEBUG
    //cout << "RigidBodyManipulator::collisionDetect: points[i].getIdA() = " << points[i].getIdA() << endl;
    //cout << "RigidBodyManipulator::collisionDetect: collision_model->readElement(points[i].getIdA()) = " << collision_model->readElement(points[i].getIdA()) << endl;
    //cout << "RigidBodyManipulator::collisionDetect: collision_model->readElement(points[i].getIdA())->getId() = " << collision_model->readElement(points[i].getIdA())->getId() << endl;
    //cout << "RigidBodyManipulator::collisionDetect: elementA = " << elementA << endl;
    //END_DEBUG
    bodyA_idx.push_back(elementA->getBody()->body_index);
    const RigidBody::CollisionElement* elementB = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(points[i].getIdB()));
    bodyB_idx.push_back(elementB->getBody()->body_index);
  }
  return points_found;
}

bool RigidBodyManipulator::collisionDetect(const KinematicsCache<double>& cache,
                                           VectorXd& phi,
                                           Matrix3Xd& normal,
                                           Matrix3Xd& xA,
                                           Matrix3Xd& xB,
                                           vector<int>& bodyA_idx,
                                           vector<int>& bodyB_idx,
                                           const vector<int>& bodies_idx,
                                           const set<string>& active_element_groups,
                                           bool use_margins)
{
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_idx_iter = bodies_idx.begin();
       body_idx_iter !=bodies_idx.end();
       ++body_idx_iter) {
    if (*body_idx_iter >= 0 && *body_idx_iter < bodies.size()) {
      for (auto group_iter = active_element_groups.begin();
          group_iter != active_element_groups.end();
          ++group_iter) {
        bodies[*body_idx_iter]->appendCollisionElementIdsFromThisBody(*group_iter, ids_to_check);
      }
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx, ids_to_check, use_margins);
}

bool RigidBodyManipulator::collisionDetect(const KinematicsCache<double>& cache,
                                           VectorXd& phi,
                                           Matrix3Xd& normal,
                                           Matrix3Xd& xA,
                                           Matrix3Xd& xB,
                                           vector<int>& bodyA_idx,
                                           vector<int>& bodyB_idx,
                                           const vector<int>& bodies_idx,
                                           bool use_margins)
{
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_idx_iter = bodies_idx.begin();
       body_idx_iter !=bodies_idx.end();
       ++body_idx_iter) {
    if (*body_idx_iter >= 0 && *body_idx_iter < bodies.size()) {
      bodies[*body_idx_iter]->appendCollisionElementIdsFromThisBody(ids_to_check);
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx, ids_to_check, use_margins);
}

bool RigidBodyManipulator::collisionDetect(const KinematicsCache<double>& cache,
                                           VectorXd& phi,
                                           Matrix3Xd& normal,
                                           Matrix3Xd& xA,
                                           Matrix3Xd& xB,
                                           vector<int>& bodyA_idx,
                                           vector<int>& bodyB_idx,
                                           const set<string>& active_element_groups,
                                           bool use_margins)
{
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_iter = bodies.begin();
       body_iter !=bodies.end();
       ++body_iter) {
    for (auto group_iter = active_element_groups.begin();
        group_iter != active_element_groups.end();
        ++group_iter) {
      (*body_iter)->appendCollisionElementIdsFromThisBody(*group_iter, ids_to_check);
    }
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx, ids_to_check, use_margins);
}

bool RigidBodyManipulator::collisionDetect(const KinematicsCache<double>& cache,
                                           VectorXd& phi,
                                           Matrix3Xd& normal,
                                           Matrix3Xd& xA,
                                           Matrix3Xd& xB,
                                           vector<int>& bodyA_idx,
                                           vector<int>& bodyB_idx,
                                           bool use_margins)
{
  vector<DrakeCollision::ElementId> ids_to_check;
  for (auto body_iter = bodies.begin();
       body_iter !=bodies.end();
       ++body_iter) {
    (*body_iter)->appendCollisionElementIdsFromThisBody(ids_to_check);
  }
  return collisionDetect(cache, phi, normal, xA, xB, bodyA_idx, bodyB_idx, ids_to_check, use_margins);
}

void RigidBodyManipulator::potentialCollisions(const KinematicsCache<double>& cache, VectorXd& phi,
                                               Matrix3Xd& normal,
                                               Matrix3Xd& xA,
                                               Matrix3Xd& xB,
                                               vector<int>& bodyA_idx,
                                               vector<int>& bodyB_idx,
                                               bool use_margins)
{
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
    const RigidBody::CollisionElement* elementA = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(potential_collisions[i].getIdA()));
    const RigidBody::CollisionElement* elementB = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(potential_collisions[i].getIdB()));
    potential_collisions[i].getResults(ptA, ptB, n, distance);
    xA.col(i) = ptA;
    xB.col(i) = ptB;
    normal.col(i) = n;
    phi[i] = distance;
    bodyA_idx.push_back(elementA->getBody()->body_index);
    bodyB_idx.push_back(elementB->getBody()->body_index);
  }
}

vector<size_t> RigidBodyManipulator::collidingPoints(const KinematicsCache<double>& cache,
    const vector<Vector3d>& points,
    double collision_threshold)
{
  updateDynamicCollisionElements(cache);
  return collision_model->collidingPoints(points, collision_threshold);
}

bool RigidBodyManipulator::allCollisions(const KinematicsCache<double>& cache, vector<int>& bodyA_idx,
                                         vector<int>& bodyB_idx,
                                         Matrix3Xd& xA_in_world, Matrix3Xd& xB_in_world,
                                         bool use_margins)
{
  updateDynamicCollisionElements(cache);

  vector<DrakeCollision::PointPair> points;
  bool points_found = collision_model->collisionPointsAllToAll(use_margins, points);

  xA_in_world = Matrix3Xd::Zero(3,points.size());
  xB_in_world = Matrix3Xd::Zero(3,points.size());

  Vector3d ptA, ptB, n;
  double distance;
  for (int i=0; i<points.size(); ++i) {
    points[i].getResults(ptA, ptB, n, distance);
    xA_in_world.col(i) = ptA;
    xB_in_world.col(i) = ptB;

    const RigidBody::CollisionElement* elementA = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(points[i].getIdA()));
    bodyA_idx.push_back(elementA->getBody()->body_index);
    const RigidBody::CollisionElement* elementB = dynamic_cast<const RigidBody::CollisionElement*>(collision_model->readElement(points[i].getIdB()));
    bodyB_idx.push_back(elementB->getBody()->body_index);
  }
  return points_found;
}

void RigidBodyManipulator::warnOnce(const string& id, const string& msg)
{
  auto print_warning_iter = already_printed_warnings.find(id);
  if (print_warning_iter == already_printed_warnings.end()) {
    cout << msg << endl;
    already_printed_warnings.insert(id);
  }
}

//bool RigidBodyManipulator::closestDistanceAllBodies(VectorXd& distance,
                                                        //MatrixXd& Jd)
//{
  //MatrixXd ptsA, ptsB, normal, JA, JB;
  //vector<int> bodyA_idx, bodyB_idx;
  //bool return_val = closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normal,distance, JA,JB,Jd);
  //DEBUG
  //cout << "RigidBodyManipulator::closestDistanceAllBodies: distance.size() = " << distance.size() << endl;
  //END_DEBUG
  //return return_val;
//};

template <typename Scalar>
void RigidBodyManipulator::updateCompositeRigidBodyInertias(KinematicsCache<Scalar>& cache, int gradient_order) const {
  if (gradient_order > 1) {
    throw std::runtime_error("only first order gradients are available");
  }
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "updateCompositeRigidBodyInertias");

  if (gradient_order > cache.getCachedInertiaGradientsOrder()) {
    for (int i = 0; i < bodies.size(); i++) {
      auto& element = cache.getElement(*bodies[i]);
      typename Gradient<typename Transform<Scalar, 3, Isometry>::MatrixType, Dynamic>::type* dTdq = nullptr;
      if (gradient_order > 0)
        dTdq = &element.dtransform_to_world_dq;

      auto inertia_in_world = transformSpatialInertia(element.transform_to_world, dTdq, bodies[i]->I.cast<Scalar>());
      element.inertia_in_world.value() = inertia_in_world.value();
      element.crb_in_world.value() = inertia_in_world.value();
      if (gradient_order > 0) {
        element.inertia_in_world.gradient().value() = inertia_in_world.gradient().value();
        element.crb_in_world.gradient().value() = inertia_in_world.gradient().value();
      }
    }

    for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; i--) {
      if (bodies[i]->hasParent()) {
        const auto& element = cache.getElement(*bodies[i]);
        auto& parent_element = cache.getElement(*(bodies[i]->parent));
        parent_element.crb_in_world.value() += element.crb_in_world.value();
        if (gradient_order > 0) {
          parent_element.crb_in_world.gradient().value() += element.crb_in_world.gradient().value();
        }
      }
    }
  }
  cache.setCachedInertiaGradientsOrder(gradient_order);
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::worldMomentumMatrix(KinematicsCache<Scalar>& cache,
    int gradient_order, const std::set<int>& robotnum, bool in_terms_of_qdot) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradient is available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "worldMomentumMatrix");

  updateCompositeRigidBodyInertias(cache, cache.getGradientOrder());

  int nq = num_positions;
  int nv = num_velocities;
  int ncols = in_terms_of_qdot ? nq : nv;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, ncols, nq, gradient_order);
  ret.value().setZero();
  if (gradient_order > 0)
    ret.gradient().value().setZero();
  int gradient_row_start = 0;
  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];

    if (body.hasParent()) {
      const auto& element = cache.getElement(body);
      const DrakeJoint& joint = body.getJoint();
      int ncols_joint = in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
      if (isBodyPartOfRobot(body, robotnum))
      {
        int start = in_terms_of_qdot ? body.position_num_start : body.velocity_num_start;

        if (in_terms_of_qdot) {
          auto crb = (element.crb_in_world.value() * element.motion_subspace_in_world.value()).eval();
          ret.value().middleCols(start, ncols_joint).noalias() = crb * element.qdot_to_v.value();
          if (gradient_order > 0) {
            auto dcrb = matGradMultMat(element.crb_in_world.value(), element.motion_subspace_in_world.value(), element.crb_in_world.gradient().value(), element.motion_subspace_in_world.gradient().value());
            ret.gradient().value().middleRows(gradient_row_start, TWIST_SIZE * ncols_joint) = matGradMultMat(crb, element.qdot_to_v.value(), dcrb, element.qdot_to_v.gradient().value());
          }
        }
        else {
          ret.value().middleCols(start, ncols_joint).noalias() = element.crb_in_world.value() * element.motion_subspace_in_world.value();
          if (gradient_order > 0) {
            ret.gradient().value().middleRows(gradient_row_start, TWIST_SIZE * ncols_joint) = matGradMultMat(element.crb_in_world.value(), element.motion_subspace_in_world.value(), element.crb_in_world.gradient().value(), element.motion_subspace_in_world.gradient().value());
          }
        }
      }
      gradient_row_start += TWIST_SIZE * ncols_joint;
    }
  }
  return ret;
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, 1> RigidBodyManipulator::worldMomentumMatrixDotTimesV(KinematicsCache<Scalar>& cache,
    int gradient_order, const std::set<int>& robotnum) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradient is available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, true, "worldMomentumMatrixDotTimesV");

  updateCompositeRigidBodyInertias(cache, cache.getGradientOrder());

  GradientVar<Scalar, TWIST_SIZE, 1> ret(TWIST_SIZE, 1, num_positions, gradient_order);
  ret.value().setZero();
  if (gradient_order > 0)
    ret.gradient().value().setZero();

  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      if (isBodyPartOfRobot(body, robotnum)) {
        const auto& element = cache.getElement(body);
        ret.value().noalias() += element.inertia_in_world.value() * element.motion_subspace_in_world_dot_times_v.value();
        auto inertia_times_twist = (element.inertia_in_world.value() * element.twist_in_world.value()).eval();
        ret.value().noalias() += crossSpatialForce(element.twist_in_world.value(), inertia_times_twist);

        if (gradient_order > 0) {
          auto dJdotVdq = element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions);
          ret.gradient().value() += matGradMultMat(element.inertia_in_world.value(), element.motion_subspace_in_world_dot_times_v.value(), element.inertia_in_world.gradient().value(), dJdotVdq);
          auto dinertia_times_twist = (element.inertia_in_world.value() * element.twist_in_world.gradient().value()).eval();
          dinertia_times_twist.noalias() += matGradMult(element.inertia_in_world.gradient().value(), element.twist_in_world.value());
          ret.gradient().value() += dCrossSpatialForce(element.twist_in_world.value(), inertia_times_twist, element.twist_in_world.gradient().value(), dinertia_times_twist);
        }
      }
    }
  }

  return ret;
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::centroidalMomentumMatrix(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum, bool in_terms_of_qdot) const
{
  // kinematics cache checks already being done in worldMomentumMatrix.
  auto ret = worldMomentumMatrix(cache, gradient_order, robotnum, in_terms_of_qdot);

  // transform from world frame to COM frame
  Matrix<Scalar, SPACE_DIMENSION, 1> com = centerOfMass(cache, 0).value();
  auto angular_momentum_matrix = ret.value().template topRows<SPACE_DIMENSION>();
  auto linear_momentum_matrix = ret.value().template bottomRows<SPACE_DIMENSION>();
  if (gradient_order > 0) {
    // gradient of CoM is linear momentum matrix in terms of qdot divided by mass
    typename Gradient<decltype(com), Eigen::Dynamic, 1>::type dcom(SPACE_DIMENSION, num_positions);
    if (in_terms_of_qdot) {
      dcom = linear_momentum_matrix;
    }
    else {
      // transform in terms of v -> in terms of qdot
      for (int i = 0; i < bodies.size(); i++) {
        RigidBody& body = *bodies[i];
        if (body.hasParent()) {
          const auto& element = cache.getElement(body);
          const DrakeJoint& joint = body.getJoint();
          int nv_joint = joint.getNumVelocities();
          int nq_joint = joint.getNumPositions();
          dcom.middleCols(body.position_num_start, nq_joint).noalias() = linear_momentum_matrix.middleCols(body.velocity_num_start, nv_joint) * element.qdot_to_v.value();
        }
      }
    }
    dcom /= getMass(robotnum);

    // unfortunately we don't yet have anything more convenient for taking the gradient of a.colwise().cross(b)
    for (int col = 0; col < ret.value().cols(); col++) {
      auto linear_momentum_matrix_col = linear_momentum_matrix.col(col);
      auto dangular_momentum_matrix_col = ret.gradient().value().template middleRows<SPACE_DIMENSION>(col * TWIST_SIZE);
      auto dlinear_momentum_matrix_col = ret.gradient().value().template middleRows<SPACE_DIMENSION>(col * TWIST_SIZE + SPACE_DIMENSION);
      dangular_momentum_matrix_col += dcrossProduct(linear_momentum_matrix_col, com, dlinear_momentum_matrix_col, dcom);
    }
  }
  angular_momentum_matrix += linear_momentum_matrix.colwise().cross(com);

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> T(Translation<Scalar, SPACE_DIMENSION>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, 1> RigidBodyManipulator::centroidalMomentumMatrixDotTimesV(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum) const
{
  // kinematics cache checks already being done in worldMomentumMatrixDotTimesV
  auto ret = worldMomentumMatrixDotTimesV(cache, gradient_order, robotnum);

  // transform from world frame to COM frame:
  auto com = centerOfMass(cache, gradient_order);
  auto angular_momentum_matrix_dot_times_v = ret.value().template topRows<SPACE_DIMENSION>();
  auto linear_momentum_matrix_dot_times_v = ret.value().template bottomRows<SPACE_DIMENSION>();

  if (gradient_order > 0) {
    auto dangular_momentum_matrix_dot_times_v = ret.gradient().value().template middleRows<SPACE_DIMENSION>(0);
    auto dlinear_momentum_matrix_dot_times_v = ret.gradient().value().template middleRows<SPACE_DIMENSION>(SPACE_DIMENSION);
    dangular_momentum_matrix_dot_times_v += dcrossProduct(linear_momentum_matrix_dot_times_v, com.value(), dlinear_momentum_matrix_dot_times_v, com.gradient().value());
  }
  angular_momentum_matrix_dot_times_v += linear_momentum_matrix_dot_times_v.cross(com.value());

  //  Valid for more general frame transformations but slower:
  //  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> T(Translation<Scalar, SPACE_DIMENSION>(-com.value()));
  //  ret.value() = transformSpatialForce(T, ret.value());

  return ret;
}

bool RigidBodyManipulator::isBodyPartOfRobot(const RigidBody& body, const std::set<int>& robotnum) const
{
  for (std::set<int>::const_iterator it = robotnum.begin(); it != robotnum.end(); ++it) {
    if (*it < -1) {
      return true;
    }
  }

  return robotnum.find(body.robotnum) != robotnum.end();
}

double RigidBodyManipulator::getMass(const std::set<int>& robotnum) const
{
  double total_mass = 0.0;
  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (isBodyPartOfRobot(body, robotnum))
    {
      total_mass += body.mass;
    }
  }
  return total_mass;
}

template <typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMass(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum) const
{
  cache.checkCachedKinematicsSettings(false, false, false, "centerOfMass"); // don't check for gradients, that will be done in centerOfMassJacobian below.

  int nq = num_positions;
  GradientVar<Scalar, SPACE_DIMENSION, 1> com(SPACE_DIMENSION, 1, nq, gradient_order);
  double m = 0.0;
  double body_mass;
  com.value().setZero();

  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (isBodyPartOfRobot(body, robotnum))
    {
      body_mass = body.mass;
      if (body_mass > 0) {
        Vector3d body_com_body_frame = (body.com.topRows<SPACE_DIMENSION>());
        auto body_com = forwardKin(cache, body_com_body_frame, i, 0, 0, gradient_order);
        com.value() *= m;
        com.value().noalias() += body_mass * body_com.value();
        com.value() /= (m + body_mass);
      }
      m += body_mass;
    }
  }

  if (gradient_order > 0) {
    auto J_com = centerOfMassJacobian(cache, gradient_order - 1, robotnum, true);
    com.gradient().value() = J_com.value();
    if (gradient_order > 1)
      com.gradient().gradient().value() = J_com.gradient().value();
  }

  return com;
}

template <typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> RigidBodyManipulator::centerOfMassJacobian(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum, bool in_terms_of_qdot) const
{
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "centerOfMassJacobian");

  auto A = worldMomentumMatrix(cache, gradient_order, robotnum, in_terms_of_qdot);
  GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> ret(SPACE_DIMENSION, A.value().cols(), num_positions, gradient_order);
  double total_mass = getMass(robotnum);
  ret.value() = A.value().template bottomRows<SPACE_DIMENSION>() / total_mass;
  if (gradient_order > 0) {
    for (int col = 0; col < A.value().cols(); col++) {
      ret.gradient().value().template middleRows<SPACE_DIMENSION>(col * SPACE_DIMENSION) = A.gradient().value().template middleRows<SPACE_DIMENSION>(col * A.value().rows() + SPACE_DIMENSION) / total_mass;
    }
  }
  return ret;
}

template <typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMassJacobianDotTimesV(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum) const
{
  // kinematics cache checks are already being done in centroidalMomentumMatrixDotTimesV
  auto cmm_dot_times_v = centroidalMomentumMatrixDotTimesV(cache, gradient_order, robotnum);
  GradientVar<Scalar, SPACE_DIMENSION, 1> ret(SPACE_DIMENSION, 1, num_positions, gradient_order);
  double total_mass = getMass(robotnum);
  ret.value().noalias() = cmm_dot_times_v.value().template bottomRows<SPACE_DIMENSION>() / total_mass;
  if (gradient_order > 0) {
    ret.gradient().value().noalias() = cmm_dot_times_v.gradient().value().template bottomRows<SPACE_DIMENSION>() / total_mass;
  }
  return ret;
}

template <typename DerivedNormal, typename DerivedPoint>
std::pair<Eigen::Vector3d, double> RigidBodyManipulator::resolveCenterOfPressure(const KinematicsCache<double>& cache, const std::vector<ForceTorqueMeasurement> & force_torque_measurements, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane) const
{
  // kinematics cache checks are already being done in relativeTransform
  typedef typename DerivedNormal::Scalar Scalar;
  typedef Matrix<Scalar, 6, 1> Vector6;
  Vector6 total_wrench = Vector6::Zero();
  for (auto it = force_torque_measurements.begin(); it != force_torque_measurements.end(); ++it) {
    Isometry3d transform_to_world(relativeTransform(cache, 0, it->frame_idx, 0).value());
    total_wrench += transformSpatialForce(transform_to_world, it->wrench);
  }
  return ::resolveCenterOfPressure(total_wrench.template head<3>(), total_wrench.template tail<3>(), normal, point_on_contact_plane);
}

int RigidBodyManipulator::getNumContacts(const set<int> &body_idx) const
{
  size_t n=0,nb=body_idx.size(),bi;
  if (nb==0) nb=bodies.size();
  set<int>::iterator iter = body_idx.begin();
  for (size_t i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    n += bodies[bi]->contact_pts.cols();
  }
  return static_cast<int>(n);
}


template<typename Derived>
void RigidBodyManipulator::getContactPositions(const KinematicsCache<typename Derived::Scalar>& cache, MatrixBase<Derived> &pos, const set<int> &body_idx) const {
  // kinematics cache checks are already being done in forwardKin
  int n = 0, nc, nb = static_cast<int>(body_idx.size()), bi;
  if (nb == 0) nb = static_cast<int>(bodies.size());
  set<int>::iterator iter = body_idx.begin();
  for (int i = 0; i < nb; i++) {
    if (body_idx.size() == 0) bi = i;
    else bi = *iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc > 0) {
      pos.block(0, n, 3, nc) = forwardKin(cache, bodies[bi]->contact_pts, bi, 0, 0, 1).value();
      n += nc;
    }
  }
}

template<typename Derived>
void RigidBodyManipulator::getContactPositionsJac(const KinematicsCache<typename Derived::Scalar>& cache, MatrixBase<Derived> &J, const set<int> &body_idx) const {
  // kinematics cache checks are already being done in forwardKinJacobian
  int n = 0, nc, nb = static_cast<int>(body_idx.size()), bi;
  if (nb == 0) nb = static_cast<int>(bodies.size());
  set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i = 0; i < nb; i++) {
    if (body_idx.size() == 0) bi = i;
    else bi = *iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc > 0) {
      p.resize(3 * nc, num_positions);
      J.block(3 * n, 0, 3 * nc, num_positions) = forwardKinJacobian(cache, bodies[bi]->contact_pts, bi, 0, 0, true, 0).value();
      n += nc;
    }
  }
}

/* [body_ind,Tframe] = parseBodyOrFrameID(body_or_frame_id) */
template <typename Scalar>
int RigidBodyManipulator::parseBodyOrFrameID(const int body_or_frame_id, Eigen::Transform<Scalar, 3, Isometry>* Tframe) const {

  int body_ind=0;
  if (body_or_frame_id == -1) {
    cerr << "parseBodyOrFrameID got a -1, which should have been reserved for COM.  Shouldn't have gotten here." << endl;
  } else if (body_or_frame_id<0) {
    int frame_ind = -body_or_frame_id-2;
    // check that this is in range
    if (frame_ind >= frames.size()){
      std::ostringstream stream;
      stream << "Got a frame ind greater than available!\n";
      throw std::runtime_error(stream.str());
    }
    body_ind = frames[frame_ind]->body->body_index;

    if (Tframe)
      (*Tframe) = frames[frame_ind]->transform_to_body.cast<Scalar>();
  } else {
    body_ind = body_or_frame_id;
    if (Tframe)
      Tframe->setIdentity();
  }
  return body_ind;
}

int RigidBodyManipulator::parseBodyOrFrameID(const int body_or_frame_id) const
{
  return parseBodyOrFrameID<double>(body_or_frame_id, nullptr);
}

void RigidBodyManipulator::findAncestorBodies(std::vector<int>& ancestor_bodies, int body_idx) const
{
  const RigidBody* current_body = bodies[body_idx].get();
  while (current_body->hasParent())
  {
    ancestor_bodies.push_back(current_body->parent->body_index);
    current_body = current_body->parent.get();
  }
}

KinematicPath RigidBodyManipulator::findKinematicPath(int start_body_or_frame_idx, int end_body_or_frame_idx) const
{
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
  size_t common_size = std::min(start_body_ancestors.size(), end_body_ancestors.size());
  bool least_common_ancestor_found = false;
  std::vector<int>::iterator start_body_lca_it = start_body_ancestors.end() - common_size;
  std::vector<int>::iterator end_body_lca_it = end_body_ancestors.end() - common_size;

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
    stream << "There is no path between " << bodies[start_body]->linkname << " and " << bodies[end_body]->linkname << ".";
    throw std::runtime_error(stream.str());
  }
  int least_common_ancestor = *start_body_lca_it;

  // compute path
  KinematicPath path;

  std::vector<int>::iterator it = start_body_ancestors.begin();
  for ( ; it != start_body_lca_it; it++) {
    path.joint_path.push_back(*it);
    path.joint_direction_signs.push_back(-1);
    path.body_path.push_back(*it);
  }

  path.body_path.push_back(least_common_ancestor);

  std::vector<int>::reverse_iterator reverse_it(end_body_lca_it);
  for ( ; reverse_it != end_body_ancestors.rend(); reverse_it++) {
    path.joint_path.push_back(*reverse_it);
    path.joint_direction_signs.push_back(1);
    path.body_path.push_back(*reverse_it);
  }
  return path;
}

template<typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::geometricJacobian(const KinematicsCache<Scalar>& cache,
    int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot, std::vector<int>* v_or_qdot_indices) const
{
  if (gradient_order > 1) {
    throw std::runtime_error("gradient order not supported");
  }
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "geometricJacobian");

  KinematicPath kinematic_path = findKinematicPath(base_body_or_frame_ind, end_effector_body_or_frame_ind);
  int nq = num_positions;

  int cols = 0;
  int body_index;
  for (size_t i = 0; i < kinematic_path.joint_path.size(); i++) {
    body_index = kinematic_path.joint_path[i];
    const std::shared_ptr<RigidBody>& body = bodies[body_index];
    const DrakeJoint& joint = body->getJoint();
    cols += in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
  }

  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, cols, nq, gradient_order);
  auto& J = ret.value();

  Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> motion_subspace;

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
    int ncols_block = in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
    int sign = kinematic_path.joint_direction_signs[i];
    auto J_block = J.template block<TWIST_SIZE, Dynamic>(0, col_start, TWIST_SIZE, ncols_block);
    if (in_terms_of_qdot) {
      J_block.noalias() = sign * element.motion_subspace_in_world.value() * element.qdot_to_v.value();
    }
    else {
      J_block.noalias() = sign * element.motion_subspace_in_world.value();
    }

    if (v_or_qdot_indices != nullptr) {
      int cols_block_start = in_terms_of_qdot ? body.position_num_start : body.velocity_num_start;
      for (int j = 0; j < ncols_block; j++) {
        v_or_qdot_indices->push_back(cols_block_start + j);
      }
    }
    col_start += ncols_block;
  }

  auto T_world_to_frame = relativeTransform(cache, expressed_in_body_or_frame_ind, 0, 0);
  Transform<Scalar, 3, Isometry> H0(T_world_to_frame.value());
  if (expressed_in_body_or_frame_ind != 0) {
    J = transformSpatialMotion(H0, J);
  }

  if (gradient_order > 0) {
    auto& dJdq = ret.gradient().value();
    dJdq.setZero();
    int col = 0;
    std::vector<int> qdot_ind_ij;
    auto rows = intRange<TWIST_SIZE>(0);
    for (size_t i = 0; i < kinematic_path.joint_path.size(); i++) {
      int j = kinematic_path.joint_path[i];
      int sign = kinematic_path.joint_direction_signs[i];
      RigidBody& body_j = *bodies[j];
      const auto& element_j = cache.getElement(body_j);
      Matrix<Scalar, TWIST_SIZE, Dynamic, 0, TWIST_SIZE, DrakeJoint::MAX_NUM_POSITIONS> Jj = (sign * element_j.motion_subspace_in_world.value()).eval();
      if (in_terms_of_qdot) {
        Jj *= element_j.qdot_to_v.value();
      }

      auto dSjdqj = (sign * element_j.motion_subspace_in_body.gradient().value()).eval();
      if (in_terms_of_qdot) {
        dSjdqj = matGradMultMat((sign * element_j.motion_subspace_in_body.value()).eval(), element_j.qdot_to_v.value(), dSjdqj, element_j.qdot_to_v.gradient().value().middleCols(body_j.position_num_start, body_j.getJoint().getNumPositions()));
      }
      auto Hj_gradientvar = relativeTransform(cache, expressed_in_body_or_frame_ind, j, 0);
      Transform<Scalar, 3, Isometry> Hj(Hj_gradientvar.value());
      auto Jj0_gradientvar = geometricJacobian(cache, expressed_in_body_or_frame_ind, j, 0, 0, true, &qdot_ind_ij);

      for (int Jj_col = 0; Jj_col < Jj.cols(); Jj_col++) {
        auto dSjdqj_block = transformSpatialMotion(Hj, dSjdqj.template middleRows<TWIST_SIZE>(Jj_col * TWIST_SIZE)).eval();
        setSubMatrixGradient<Eigen::Dynamic>(dJdq, dSjdqj_block, rows, intRange<1>(col), TWIST_SIZE, body_j.position_num_start, body_j.getJoint().getNumPositions());
        auto crm_block = transformSpatialMotion(H0, crossSpatialMotion((-Jj.col(Jj_col)).eval(), Jj0_gradientvar.value()));
        int crm_block_col = 0;
        for (std::vector<int>::iterator it = qdot_ind_ij.begin(); it != qdot_ind_ij.end(); ++it) {
          dJdq.template block<TWIST_SIZE, 1>(TWIST_SIZE * col, *it) += crm_block.col(crm_block_col);
          crm_block_col++;
        }
        col++;
      }
    }
  }
  return ret;
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, 1> RigidBodyManipulator::geometricJacobianDotTimesV(const KinematicsCache<Scalar>& cache,
    int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradient available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, true, "geometricJacobianDotTimesV");

  GradientVar<Scalar, TWIST_SIZE, 1> ret(TWIST_SIZE, 1, num_positions, gradient_order);

  int base_body_ind = parseBodyOrFrameID(base_body_or_frame_ind);
  int end_effector_body_ind = parseBodyOrFrameID(end_effector_body_or_frame_ind);

  const auto& base_element = cache.getElement(*bodies[base_body_ind]);
  const auto& end_effector_element = cache.getElement(*bodies[end_effector_body_ind]);

  ret.value() = end_effector_element.motion_subspace_in_world_dot_times_v.value() - base_element.motion_subspace_in_world_dot_times_v.value();
  if (gradient_order > 0) {
    ret.gradient().value() = end_effector_element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions) - base_element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions);
  }

  int world_ind = 0;
  return transformSpatialAcceleration(cache, ret, base_body_ind, end_effector_body_ind, world_ind, expressed_in_body_or_frame_ind);
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, 1> RigidBodyManipulator::relativeTwist(const KinematicsCache<Scalar>& cache,
    int base_or_frame_ind, int body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order) const
{
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, false, "relativeTwist");

  GradientVar<Scalar, TWIST_SIZE, 1> ret(TWIST_SIZE, 1, num_positions, gradient_order);

  int base_ind = parseBodyOrFrameID(base_or_frame_ind);
  int body_ind = parseBodyOrFrameID(body_or_frame_ind);
  int world = 0;
  auto T = relativeTransform(cache, expressed_in_body_or_frame_ind, world, gradient_order);
  auto T_isometry = Transform<Scalar, SPACE_DIMENSION, Isometry>(T.value());

  const auto& base_element = cache.getElement(*bodies[base_ind]);
  const auto& body_element = cache.getElement(*bodies[body_ind]);
  Matrix<Scalar, TWIST_SIZE, 1> relative_twist_in_world = body_element.twist_in_world.value() - base_element.twist_in_world.value();
  ret.value() = transformSpatialMotion(T_isometry, relative_twist_in_world);

  if (gradient_order > 0) {
    auto drelative_twist_in_world = (body_element.twist_in_world.gradient().value() - base_element.twist_in_world.gradient().value()).eval();
    ret.gradient().value() = dTransformSpatialMotion(T_isometry, relative_twist_in_world, T.gradient().value(), drelative_twist_in_world);
  }
  return ret;
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, 1> RigidBodyManipulator::transformSpatialAcceleration(const KinematicsCache<Scalar>& cache,
    const GradientVar<Scalar, TWIST_SIZE, 1>& spatial_acceleration, int base_ind, int body_ind, int old_expressed_in_body_or_frame_ind, int new_expressed_in_body_or_frame_ind) const
{
  int gradient_order = spatial_acceleration.maxOrder();
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, true, "transformSpatialAcceleration");

  if (old_expressed_in_body_or_frame_ind == new_expressed_in_body_or_frame_ind) {
    return spatial_acceleration;
  }

  GradientVar<Scalar, TWIST_SIZE, 1> ret(TWIST_SIZE, 1, num_positions, gradient_order);

  auto twist_of_body_wrt_base = relativeTwist(cache, base_ind, body_ind, old_expressed_in_body_or_frame_ind, gradient_order);
  auto twist_of_old_wrt_new = relativeTwist(cache, new_expressed_in_body_or_frame_ind, old_expressed_in_body_or_frame_ind, old_expressed_in_body_or_frame_ind, gradient_order);
  auto T_old_to_new = relativeTransform(cache, new_expressed_in_body_or_frame_ind, old_expressed_in_body_or_frame_ind, gradient_order);
  auto T_old_to_new_isometry = Transform<Scalar, SPACE_DIMENSION, Isometry>(T_old_to_new.value());

  Matrix<double, TWIST_SIZE, 1> spatial_accel_temp = crossSpatialMotion(twist_of_old_wrt_new.value(), twist_of_body_wrt_base.value());
  spatial_accel_temp += spatial_acceleration.value();
  ret.value() = transformSpatialMotion(T_old_to_new_isometry, spatial_accel_temp);

  if (gradient_order > 0) {
    auto dspatial_accel_temp = dCrossSpatialMotion(twist_of_old_wrt_new.value(), twist_of_body_wrt_base.value(), twist_of_old_wrt_new.gradient().value(), twist_of_body_wrt_base.gradient().value());
    dspatial_accel_temp += spatial_acceleration.gradient().value();
    ret.gradient().value() = dTransformSpatialMotion(T_old_to_new_isometry, spatial_accel_temp, T_old_to_new.gradient().value(), dspatial_accel_temp);
  }
  return ret;
}

template<typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> RigidBodyManipulator::relativeTransform(const KinematicsCache<Scalar>& cache,
    int base_or_frame_ind, int body_or_frame_ind, int gradient_order) const
{
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "relativeTransform");

  int nq = num_positions;
  GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> ret(SPACE_DIMENSION + 1, SPACE_DIMENSION + 1, nq, gradient_order);

  Transform<Scalar, 3, Isometry> Tbase_frame;
  int base_ind = parseBodyOrFrameID(base_or_frame_ind, &Tbase_frame);
  Transform<Scalar, 3, Isometry> Tbody_frame;
  int body_ind = parseBodyOrFrameID(body_or_frame_ind, &Tbody_frame);

  const auto& body_element = cache.getElement(*bodies[body_ind]);
  const auto& base_element = cache.getElement(*bodies[base_ind]);

  Transform<Scalar, 3, Isometry> Tbaseframe_to_world = base_element.transform_to_world * Tbase_frame;
  Transform<Scalar, 3, Isometry> Tworld_to_baseframe = Tbaseframe_to_world.inverse();
  Transform<Scalar, 3, Isometry> Tbodyframe_to_world = body_element.transform_to_world * Tbody_frame;
  ret.value() = (Tworld_to_baseframe * Tbodyframe_to_world).matrix();

  if (gradient_order > 0) {
    auto dTbaseframe_to_world = matGradMult(base_element.dtransform_to_world_dq, Tbase_frame.matrix());
    auto dTworld_to_baseframe = dHomogTransInv(Tbaseframe_to_world, dTbaseframe_to_world);
    auto dTbodyframe_to_world = matGradMult(body_element.dtransform_to_world_dq, Tbody_frame.matrix());
    ret.gradient().value() = matGradMultMat(Tworld_to_baseframe.matrix(), Tbodyframe_to_world.matrix(), dTworld_to_baseframe, dTbodyframe_to_world);
  }
  if (gradient_order > 1) {
    throw std::runtime_error("gradient order > 1 not supported");
  }
  return ret;
}

template<typename Scalar>
GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::massMatrix(KinematicsCache<Scalar>& cache, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradients are available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "massMatrix");

  int nv = num_velocities;
  int nq = num_positions;
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(nv, nv, nq, gradient_order);
  ret.value().setZero();
  if (gradient_order > 0)
    ret.gradient().value().setZero();

  updateCompositeRigidBodyInertias(cache, cache.getGradientOrder());

  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body_i = *bodies[i];
    if (body_i.hasParent()) {
      const auto& element_i = cache.getElement(body_i);
      int v_start_i = body_i.velocity_num_start;
      int nv_i = body_i.getJoint().getNumVelocities();
      GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> F(TWIST_SIZE, nv_i, nq, gradient_order);
      F.value().noalias() = (element_i.crb_in_world.value() * element_i.motion_subspace_in_world.value()).eval();

      // Hii
      ret.value().block(v_start_i, v_start_i, nv_i, nv_i).noalias() = (element_i.motion_subspace_in_world.value().transpose() * F.value()).eval();
      if (gradient_order > 0) {
        F.gradient().value() = matGradMultMat(element_i.crb_in_world.value(), element_i.motion_subspace_in_world.value(), element_i.crb_in_world.gradient().value(), element_i.motion_subspace_in_world.gradient().value());
        auto dHii = matGradMultMat(element_i.motion_subspace_in_world.value().transpose(), F.value(), transposeGrad(element_i.motion_subspace_in_world.gradient().value(), TWIST_SIZE), F.gradient().value());
        for (int row = 0; row < nv_i; row++) {
          for (int col = 0; col < nv_i; col++) {
            setSubMatrixGradient<Eigen::Dynamic>(ret.gradient().value(), getSubMatrixGradient<Eigen::Dynamic>(dHii, row, col, nv_i), v_start_i + row, v_start_i + col, nv);
          }
        }

      }

      // Hij
      shared_ptr<RigidBody> body_j(body_i.parent);
      while (body_j->hasParent()) {
        const auto& element_j = cache.getElement(*body_j);
        int v_start_j = body_j->velocity_num_start;
        int nv_j = body_j->getJoint().getNumVelocities();
        auto Hji = (element_j.motion_subspace_in_world.value().transpose() * F.value()).eval();
        ret.value().block(v_start_j, v_start_i, nv_j, nv_i) = Hji;
        ret.value().block(v_start_i, v_start_j, nv_i, nv_j) = Hji.transpose();

        if (gradient_order > 0) {
          auto dHji = matGradMultMat(element_j.motion_subspace_in_world.value().transpose(), F.value(), transposeGrad(element_j.motion_subspace_in_world.gradient().value(), TWIST_SIZE), F.gradient().value());
          for (int row = 0; row < Hji.rows(); row++) {
            for (int col = 0; col < Hji.cols(); col++) {
              auto dHji_element = getSubMatrixGradient<Eigen::Dynamic>(dHji, row, col, Hji.rows());
              setSubMatrixGradient<Eigen::Dynamic>(ret.gradient().value(), dHji_element, row + v_start_j, col + v_start_i, nv);
              setSubMatrixGradient<Eigen::Dynamic>(ret.gradient().value(), dHji_element, col + v_start_i, row + v_start_j, nv);
            }
          }
        }
        body_j = body_j->parent;
      }

    }
  }

  return ret;
}

/**
 * Note that this inverse dynamics algorithm can be used to compute the 'C' dynamics bias term by setting
 * the joint acceleration vector to zero (or just passing in a nullptr for vd).
 * This algorithm can also be used to compute the gravitational term only by passing in nullptr for vd and
 * additionally calling doKinematics with a zero joint velocity vector.
 * To compute only the Coriolis term, pass in nullptr for vd and set gravity to zero.
 *
 * Note that the wrenches in f_ext are expressed in body frame.
 */
template <typename Scalar>
GradientVar<Scalar, Eigen::Dynamic, 1> RigidBodyManipulator::inverseDynamics(KinematicsCache<Scalar>& cache,
    std::map<int, std::unique_ptr<GradientVar<Scalar, TWIST_SIZE, 1> > >& f_ext,
    GradientVar<Scalar, Eigen::Dynamic, 1>* vd, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradients are available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, true, "inverseDynamics");

  updateCompositeRigidBodyInertias(cache, cache.getGradientOrder());

  int nq = num_positions;
  int nv = num_velocities;

  typedef typename Eigen::Matrix<Scalar, TWIST_SIZE, 1> Vector6;

  Vector6 root_accel = -a_grav;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> net_wrenches(TWIST_SIZE, bodies.size(), nq + nv, gradient_order); // gradient w.r.t q and v
  net_wrenches.value().col(0).setZero();
  if (gradient_order > 0) {
    net_wrenches.gradient().value().template topRows<TWIST_SIZE>().setZero();
  }

  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      const auto& element = cache.getElement(body);
      Vector6 spatial_accel = root_accel + element.motion_subspace_in_world_dot_times_v.value();
      int nv_joint = body.getJoint().getNumVelocities();
      if (vd != nullptr) {
        auto vdJoint = vd->value().middleRows(body.velocity_num_start, nv_joint);
        spatial_accel.noalias() += element.motion_subspace_in_world.value() * vdJoint;
      }

      auto I_times_twist = (element.inertia_in_world.value() * element.twist_in_world.value()).eval();
      net_wrenches.value().col(i).noalias() = element.inertia_in_world.value() * spatial_accel;
      net_wrenches.value().col(i).noalias() += crossSpatialForce(element.twist_in_world.value(), I_times_twist);

      if (gradient_order > 0) {
        typename Gradient<Vector6, Eigen::Dynamic>::type dspatial_acceldq = element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions);
        typename Gradient<Vector6, Eigen::Dynamic>::type dspatial_acceldv = element.motion_subspace_in_world_dot_times_v.gradient().value().rightCols(num_velocities);

        if (vd != nullptr && nv_joint > 0) {
          const auto& vd_const = *vd; // eliminates the need for an additional explicit instantiation
          auto vdJoint = vd_const.value().middleRows(body.velocity_num_start, nv_joint);
          auto dvdJoint = vd_const.gradient().value().middleRows(body.velocity_num_start, nv_joint);
          dspatial_acceldq.noalias() += element.motion_subspace_in_world.value() * dvdJoint.middleCols(0, nq);
          dspatial_acceldq += matGradMult(element.motion_subspace_in_world.gradient().value(), vdJoint);

          dspatial_acceldv.noalias() += element.motion_subspace_in_world.value() * dvdJoint.middleCols(nq, nv);
        }

        typename Gradient<Vector6, Eigen::Dynamic>::type dI_times_twistdq = element.inertia_in_world.value() * element.twist_in_world.gradient().value();
        dI_times_twistdq += matGradMult(element.inertia_in_world.gradient().value(), element.twist_in_world.value());

        std::vector<int> v_indices;
        auto dtwist_dvsubvector = geometricJacobian(cache, 0, i, 0, 0, false, &v_indices);

        typename Gradient<Vector6, Eigen::Dynamic>::type dtwistdv(TWIST_SIZE, nv);
        dtwistdv.setZero();
        typename Gradient<Vector6, Eigen::Dynamic>::type dI_times_twistdv(TWIST_SIZE, nv);
        dI_times_twistdv.setZero();
        auto dI_times_twist_dvsubvector = element.inertia_in_world.value() * dtwist_dvsubvector.value();
        for (size_t col = 0; col < v_indices.size(); col++) {
          dtwistdv.col(v_indices[col]) = dtwist_dvsubvector.value().col(col);
          dI_times_twistdv.col(v_indices[col]) = dI_times_twist_dvsubvector.col(col);
        }

        auto net_wrenches_q_gradient_block = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, 0, TWIST_SIZE, nq);
        net_wrenches_q_gradient_block.noalias() = element.inertia_in_world.value() * dspatial_acceldq;
        net_wrenches_q_gradient_block += matGradMult(element.inertia_in_world.gradient().value(), spatial_accel);
        net_wrenches_q_gradient_block += dCrossSpatialForce(element.twist_in_world.value(), I_times_twist, element.twist_in_world.gradient().value(), dI_times_twistdq);

        auto net_wrenches_v_gradient_block = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, nq, TWIST_SIZE, nv);
        net_wrenches_v_gradient_block.noalias() = element.inertia_in_world.value() * dspatial_acceldv;
        net_wrenches_v_gradient_block += dCrossSpatialForce(element.twist_in_world.value(), I_times_twist, dtwistdv, dI_times_twistdv);
      }

      if (f_ext[i] != nullptr) {
        net_wrenches.value().col(i) -= transformSpatialForce(element.transform_to_world, f_ext[i]->value());

        if (gradient_order > 0) {
          auto net_wrenches_q_gradient_block = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, 0, TWIST_SIZE, nq);
          auto df_extdq = f_ext[i]->gradient().value().middleCols(0, nq);
          net_wrenches_q_gradient_block -= dTransformSpatialForce(element.transform_to_world, f_ext[i]->value(), element.dtransform_to_world_dq, df_extdq);

          auto net_wrenches_v_gradient_block = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, nq, TWIST_SIZE, nv);
          auto df_extdv = f_ext[i]->gradient().value().middleCols(nq, nv);
          net_wrenches_v_gradient_block -= transformSpatialForce(element.transform_to_world, df_extdv);
        }
      }
    }
  }

  GradientVar<Scalar, Eigen::Dynamic, 1> ret(num_velocities, 1, nq + nv, gradient_order);

  for (int i = static_cast<int>(bodies.size()) - 1; i >= 0; i--) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      const auto& element = cache.getElement(body);
      const auto& net_wrenches_const = net_wrenches; // eliminates the need for another explicit instantiation
      auto joint_wrench = net_wrenches_const.value().col(i);
      int nv_joint = body.getJoint().getNumVelocities();
      auto J_transpose = element.motion_subspace_in_world.value().transpose();
      ret.value().middleRows(body.velocity_num_start, nv_joint).noalias() = J_transpose * joint_wrench;
      auto parent_net_wrench = net_wrenches.value().col(body.parent->body_index);
      parent_net_wrench += joint_wrench;

      if (gradient_order > 0) {
        auto djoint_wrenchdq = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, 0, TWIST_SIZE, nq);
        auto dCdq_block = ret.gradient().value().block(body.velocity_num_start, 0, nv_joint, nq);
        dCdq_block = J_transpose * djoint_wrenchdq;
        dCdq_block += matGradMult(transposeGrad(element.motion_subspace_in_world.gradient().value(), TWIST_SIZE), joint_wrench);

        auto djoint_wrenchdv = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * i, nq, TWIST_SIZE, nv);
        auto dCdv_block = ret.gradient().value().block(body.velocity_num_start, nq, nv_joint, nv);
        dCdv_block.noalias() = J_transpose * djoint_wrenchdv;

        auto dparent_net_wrenchdq = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * body.parent->body_index, 0, TWIST_SIZE, nq);
        dparent_net_wrenchdq += djoint_wrenchdq;

        auto dparent_net_wrenchdv = net_wrenches.gradient().value().template block<TWIST_SIZE, Eigen::Dynamic>(TWIST_SIZE * body.parent->body_index, nq, TWIST_SIZE, nv);
        dparent_net_wrenchdv += djoint_wrenchdv;
      }
    }
  }

  auto friction_torques = frictionTorques(cache.getV(), gradient_order);
  ret.value() += friction_torques.value();
  if (gradient_order > 0)
    ret.gradient().value().block(0, nq, nv, nv) += friction_torques.gradient().value();

  return ret;
}

template <typename DerivedV>
GradientVar<typename DerivedV::Scalar, Dynamic, 1> RigidBodyManipulator::frictionTorques(Eigen::MatrixBase<DerivedV> const & v, int gradient_order) const {
  typedef typename DerivedV::Scalar Scalar;
  GradientVar<Scalar, Dynamic, 1> ret(num_velocities, 1, num_velocities, gradient_order);
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradients are available");

  if (gradient_order > 0)
    ret.gradient().value().setZero();

  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      const DrakeJoint& joint = body.getJoint();
      int nv_joint = joint.getNumVelocities();
      int v_start_joint = body.velocity_num_start;
      auto v_body = v.middleRows(v_start_joint, nv_joint);
      auto friction_torque_body = joint.frictionTorque(v_body, gradient_order);
      ret.value().middleRows(v_start_joint, nv_joint) = friction_torque_body.value();
      if (gradient_order > 0) {
        ret.gradient().value().block(v_start_joint, v_start_joint, nv_joint, nv_joint) = friction_torque_body.gradient().value();
      }
    }
  }

  return ret;
}

/*
 * rotation_type  0, no rotation
 *      1, output Euler angles
 *      2, output quaternion [w,x,y,z], with w>=0
 */
template <typename DerivedPoints>
GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> RigidBodyManipulator::forwardKin(const KinematicsCache<typename DerivedPoints::Scalar>& cache,
    const MatrixBase<DerivedPoints> &points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order) const
{
  if (gradient_order > 2)
    throw std::runtime_error("only first and second order gradients are available");
  cache.checkCachedKinematicsSettings(false, false, false, "forwardKin"); // rely on forwardJac for gradient cache check

  int nq = num_positions;
  int npoints = static_cast<int>(points.cols());
  typedef typename DerivedPoints::Scalar Scalar;

  // compute rotation and translation
  auto T = relativeTransform(cache, new_body_or_frame_ind, current_body_or_frame_ind, 0).value();
  auto R = T.template topLeftCorner<SPACE_DIMENSION, SPACE_DIMENSION>();
  auto p = T.template topRightCorner<SPACE_DIMENSION, 1>();

  // transform points to new frame
  GradientVar<Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> x(SPACE_DIMENSION + rotationRepresentationSize(rotation_type), npoints, nq, gradient_order);
  x.value().template topRows<SPACE_DIMENSION>().colwise() = p;
  x.value().template topRows<SPACE_DIMENSION>().noalias() += R * points;

  // convert rotation representation
  auto qrot = rotmat2Representation(R, rotation_type);
  x.value().bottomRows(qrot.rows()).colwise() = qrot;

  if (gradient_order > 0) {
    int J_gradient_order = gradient_order - 1;
    auto J = forwardKinJacobian(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type, true, J_gradient_order);
    x.gradient().value() = J.value();
    if (J_gradient_order > 0)
      x.gradient().gradient().value() = J.gradient().value();
  }

  return x;
}

template <typename DerivedPoints>
GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian(const KinematicsCache<typename DerivedPoints::Scalar>& cache,
    const MatrixBase<DerivedPoints> &points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, bool in_terms_of_qdot, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradient is available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "forwardKinJacobian");
  typedef typename DerivedPoints::Scalar Scalar;

  // possibly slightly wasteful if we needed x anyway, but not terrible
  auto x = forwardKin(cache, points, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type, 0).value();

  int nq = num_positions;
  int nv = num_velocities;
  int cols = in_terms_of_qdot ? nq : nv;
  int npoints = static_cast<int>(x.cols());

  // compute geometric Jacobian
  int body_ind = parseBodyOrFrameID(current_body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(new_body_or_frame_ind);
  std::vector<int> v_or_q_indices;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> J_geometric = geometricJacobian(cache, base_ind, body_ind, new_body_or_frame_ind, gradient_order, in_terms_of_qdot, &v_or_q_indices);

  // split up into rotational and translational parts
  auto Jomega = J_geometric.value().template topRows<SPACE_DIMENSION>();
  auto Jv = J_geometric.value().template bottomRows<SPACE_DIMENSION>();

  // compute rotation Jacobian
  DenseIndex rotation_representation_size = x.rows() - SPACE_DIMENSION;
  auto qrot = x.template block<Eigen::Dynamic, 1>(SPACE_DIMENSION, 0, rotation_representation_size, 1);
  auto Phi = angularvel2RepresentationDotMatrix(rotation_type, qrot, gradient_order);
  auto Jrot = (Phi.value() * Jomega).eval();

  // compute J
  GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> J(x.size(), cols, nq, gradient_order);
  J.value().setZero();
  int row_start = 0;
  for (int i = 0; i < npoints; i++) {
    // translation part
    int col = 0;
    const auto& point = x.template block<SPACE_DIMENSION, 1>(0, i);
    for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
      J.value().template block<SPACE_DIMENSION, 1>(row_start, *it) = Jv.col(col);
      const auto& Jomega_col = Jomega.col(col);
      J.value().template block<SPACE_DIMENSION, 1>(row_start, *it).noalias() += Jomega_col.cross(point);
      col++;
    }
    row_start += SPACE_DIMENSION;

    // rotation part
    if (Jrot.rows() > 0) {
      col = 0;
      for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
        J.value().template block<Eigen::Dynamic, 1>(row_start, *it, Jrot.rows(), 1) = Jrot.col(col);
        col++;
      }
      row_start += static_cast<int>(qrot.rows());
    }
  }

  if (gradient_order > 0) {
    typename Gradient<decltype(Jomega), Eigen::Dynamic>::type dJomega(Jomega.size(), nq);
    typename Gradient<decltype(Jv), Eigen::Dynamic>::type dJv(Jv.size(), nq);
    for (int col = 0; col < J_geometric.value().cols(); col++) {
      for (int row = 0; row < SPACE_DIMENSION; row++) {
        dJomega.row(row + col * Jomega.rows()) = getSubMatrixGradient<Eigen::Dynamic>(J_geometric.gradient().value(), row, col, J_geometric.value().rows());
        dJv.row(row + col * Jv.rows()) = getSubMatrixGradient<Eigen::Dynamic>(J_geometric.gradient().value(), row + SPACE_DIMENSION, col, J_geometric.value().rows());
      }
    }

    KinematicPath path = findKinematicPath(base_ind, body_ind);
    MatrixXd dqrotdq = in_terms_of_qdot ? compactToFull(Jrot, path.joint_path, true) : transformVelocityMappingToPositionDotMapping(cache, compactToFull(Jrot, path.joint_path, false));

    auto dPhidq = (Phi.gradient().value() * dqrotdq).eval();
    auto dJrot = matGradMultMat(Phi.value(), Jomega, dPhidq, dJomega);

    J.gradient().value().setZero();

    row_start = 0;
    auto rows = intRange<SPACE_DIMENSION>(0);
    for (int i = 0; i < npoints; i++) {
      // translation part
      int col = 0;
      const auto& point = x.template block<SPACE_DIMENSION, 1>(0, i);
      Matrix<Scalar, SPACE_DIMENSION, Dynamic> dpoint;
      if (in_terms_of_qdot)
        dpoint = getSubMatrixGradient<Eigen::Dynamic>(J.value(), rows, intRange<1>(i), x.rows());
      else
        dpoint = transformVelocityMappingToPositionDotMapping(cache, getSubMatrixGradient<Eigen::Dynamic>(J.value(), rows, intRange<1>(i), x.rows()));

      for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
        const auto& Jomega_col = Jomega.col(col);
        auto dJpos_col = getSubMatrixGradient<Eigen::Dynamic>(dJv, rows, intRange<1>(col), Jv.rows());
        auto dJomega_col = getSubMatrixGradient<Eigen::Dynamic>(dJomega, rows, intRange<1>(col), Jomega.rows());
        dJpos_col.noalias() += dcrossProduct(Jomega_col, point, dJomega_col, dpoint);
        setSubMatrixGradient<Eigen::Dynamic>(J.gradient().value(), dJpos_col, intRange<SPACE_DIMENSION>(row_start), intRange<1>(*it), J.value().rows());
        col++;
      }
      row_start += SPACE_DIMENSION;

      // rotation part
      if (Jrot.rows() > 0) {
        col = 0;
        for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
          for (int row = 0; row < qrot.rows(); row++) {
            auto dJrot_element = getSubMatrixGradient<Eigen::Dynamic>(dJrot, row, col, Jrot.rows());
            setSubMatrixGradient<Eigen::Dynamic>(J.gradient().value(), dJrot_element, row_start + row, *it, J.value().rows());
          }
          col++;
        }
        row_start += static_cast<int>(qrot.rows());
      }
    }
  }

  return J;
}

template <typename Scalar>
GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinPositionGradient(const KinematicsCache<Scalar>& cache,
  int npoints, int current_body_or_frame_ind, int new_body_or_frame_ind, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("Only first order gradients supported");
  cache.checkCachedKinematicsSettings(gradient_order > 0, false, false, "forwardKinPositionGradient");

  int nq = num_positions;
  auto Tinv = relativeTransform(cache, current_body_or_frame_ind, new_body_or_frame_ind, gradient_order);
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(SPACE_DIMENSION * npoints, SPACE_DIMENSION * npoints, nq, gradient_order);
  ret.value().setZero();
  for (int i = 0; i < npoints; i++) {
    ret.value().template block<SPACE_DIMENSION, SPACE_DIMENSION>(SPACE_DIMENSION * i, SPACE_DIMENSION * i) = Tinv.value().template topLeftCorner<SPACE_DIMENSION, SPACE_DIMENSION>();
  }

  if (gradient_order > 0) {
    ret.gradient().value().setZero();
    std::vector<int> rows_cols;
    rows_cols.reserve(SPACE_DIMENSION);
    auto Rinv_gradient = getSubMatrixGradient<Eigen::Dynamic>(Tinv.gradient().value(), intRange<SPACE_DIMENSION>(0), intRange<SPACE_DIMENSION>(0), Tinv.value().rows());

    for (int i = 0; i < npoints; i++) {
      rows_cols.clear();
      for (int j = 0; j < SPACE_DIMENSION; j++)
        rows_cols.push_back(SPACE_DIMENSION * i + j);
      setSubMatrixGradient(ret.gradient().value(), Rinv_gradient, rows_cols, rows_cols, ret.value().rows());
    }
  }
  return ret;
}

template <typename DerivedPoints>
GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, 1> RigidBodyManipulator::forwardJacDotTimesV(const KinematicsCache<typename DerivedPoints::Scalar>& cache, const MatrixBase<DerivedPoints>& points,
    int body_or_frame_ind, int base_or_frame_ind, int rotation_type, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradients are available");
  cache.checkCachedKinematicsSettings(gradient_order > 0, true, true, "forwardJacDotTimesV");

  typedef typename DerivedPoints::Scalar Scalar;
  int npoints = static_cast<int>(points.cols());

  auto x = forwardKin(cache, points, body_or_frame_ind, base_or_frame_ind, rotation_type, gradient_order);
  auto r = x.value().template topRows<SPACE_DIMENSION>();
  auto qrot = x.value().template bottomLeftCorner<Eigen::Dynamic, 1>(rotationRepresentationSize(rotation_type), 1);
  GradientVar<Scalar, Eigen::Dynamic, SPACE_DIMENSION> Phi = angularvel2RepresentationDotMatrix(rotation_type, qrot, gradient_order + 1);

  int expressed_in = base_or_frame_ind;
  const auto twist = relativeTwist(cache, base_or_frame_ind, body_or_frame_ind, expressed_in, gradient_order);
  const auto J_geometric_dot_times_v = geometricJacobianDotTimesV(cache, base_or_frame_ind, body_or_frame_ind, expressed_in, gradient_order);

  auto omega_twist = twist.value().template topRows<SPACE_DIMENSION>();
  auto v_twist = twist.value().template bottomRows<SPACE_DIMENSION>();
  auto qrotdot = (Phi.value() * omega_twist).eval();
  Matrix<Scalar, Dynamic, 1> Phid_vector = Phi.gradient().value() * qrotdot;
  Map<Matrix<Scalar, Eigen::Dynamic, SPACE_DIMENSION>> Phid(Phid_vector.data(), Phi.value().rows(), Phi.value().cols());

  GradientVar<Scalar, Eigen::Dynamic, 1> Jrotdot_times_v(Phid.rows(), 1, num_positions, gradient_order);
  Jrotdot_times_v.value().noalias() = (Phid * omega_twist).eval();
  Jrotdot_times_v.value().noalias() += Phi.value() * J_geometric_dot_times_v.value().template topRows<SPACE_DIMENSION>();

  auto rdots = (-r.colwise().cross(omega_twist)).eval();
  rdots.colwise() += v_twist;
  auto Jposdot_times_v_mat = (-rdots.colwise().cross(omega_twist)).eval();
  Jposdot_times_v_mat -= (r.colwise().cross(J_geometric_dot_times_v.value().template topRows<SPACE_DIMENSION>())).eval();
  Jposdot_times_v_mat.colwise() += J_geometric_dot_times_v.value().template bottomRows<SPACE_DIMENSION>();

  GradientVar<Scalar, Dynamic, 1> ret(x.value().size(), 1, num_positions, gradient_order);
  typename MatrixXd::Index row_start = 0;
  for (int i = 0; i < npoints; i++) {
    ret.value().template middleRows<SPACE_DIMENSION>(row_start) = Jposdot_times_v_mat.col(i);
    row_start += SPACE_DIMENSION;

    ret.value().middleRows(row_start, Jrotdot_times_v.value().rows()) = Jrotdot_times_v.value();
    row_start += Jrotdot_times_v.value().rows();
  }

  if (gradient_order > 0) {
    auto dqrot = x.gradient().value().middleRows(SPACE_DIMENSION, qrot.rows());
    auto domega_twist = twist.gradient().value().template topRows<SPACE_DIMENSION>();
    auto dv_twist = twist.gradient().value().template bottomRows<SPACE_DIMENSION>();
    auto dPhi = (Phi.gradient().value() * dqrot).eval();
    auto dqrotdot = matGradMult(dPhi, omega_twist);
    dqrotdot.noalias() += Phi.value() * domega_twist;
    auto ddPhidqrotdq = (Phi.gradient().gradient().value() * dqrot).eval();
    auto dPhid = matGradMult(ddPhidqrotdq, qrotdot);
    dPhid.noalias() += (Phi.gradient().value() * dqrotdot).eval();

    auto dJrotdot_times_v = (matGradMult(dPhid, omega_twist)).eval();
    dJrotdot_times_v.noalias() += Phid * domega_twist;
    dJrotdot_times_v.noalias() += matGradMult(dPhi, J_geometric_dot_times_v.value().template topRows<SPACE_DIMENSION>());
    dJrotdot_times_v.noalias() += Phi.value() * J_geometric_dot_times_v.gradient().value().template topRows<SPACE_DIMENSION>();

    row_start = 0;
    auto pos_rows = intRange<SPACE_DIMENSION>(0);
    for (int i = 0; i < npoints; i++) {
      auto r_col = r.col(i);
      auto rdot = rdots.col(i);
      auto dr_col = getSubMatrixGradient<Eigen::Dynamic>(x.gradient().value(), pos_rows, intRange<1>(i), x.value().rows());
      auto drdot = dcrossProduct(omega_twist, r_col, domega_twist, dr_col);
      drdot += dv_twist;
      auto dJpos_dot_times_v = ret.gradient().value().template middleRows<SPACE_DIMENSION>(row_start);
      dJpos_dot_times_v.noalias() = dcrossProduct(omega_twist, rdot, domega_twist, drdot);
      dJpos_dot_times_v.noalias() += dcrossProduct(J_geometric_dot_times_v.value().template topRows<SPACE_DIMENSION>(), r_col, J_geometric_dot_times_v.gradient().value().template topRows<SPACE_DIMENSION>(), dr_col);
      dJpos_dot_times_v += J_geometric_dot_times_v.gradient().value().template bottomRows<SPACE_DIMENSION>();
      row_start += SPACE_DIMENSION;

      ret.gradient().value().middleRows(row_start, Jrotdot_times_v.value().rows()) = dJrotdot_times_v;
      row_start += Jrotdot_times_v.value().rows();
    }
  }

  return ret;
}

shared_ptr<RigidBody> RigidBodyManipulator::findLink(std::string linkname, int robot) const
{
  std::transform(linkname.begin(), linkname.end(), linkname.begin(), ::tolower); // convert to lower case

  //std::regex linkname_connector("[abc]");
  //cout<<"get linkname_connector"<<endl;
  //linkname = std::regex_replace(linkname,linkname_connector,string("_"));
  int match = -1;
  for(int i = 0;i<bodies.size();i++) {
    // Note: unlike the MATLAB implementation, I don't have to handle the fixed joint names
    string lower_linkname = bodies[i]->linkname;
    std::transform(lower_linkname.begin(), lower_linkname.end(), lower_linkname.begin(),
                   ::tolower); // convert to lower case
    if (lower_linkname.compare(linkname) == 0) { // the names match
      if (robot == -1 || bodies[i]->robotnum == robot) { // it's the right robot
        if (match < 0) { // it's the first match
          match = i;
        } else {
          cerr << "found multiple links named " << linkname << endl;
          return nullptr;
        }
      }
    }
  }
  if (match>=0) return bodies[match];
  cerr << "could not find any links named " << linkname << endl;
  return nullptr;
}

int RigidBodyManipulator::findLinkId(const std::string& name, int robot) const
{
	shared_ptr<RigidBody> link = findLink(name,robot);
	if (link == nullptr)
	  throw std::runtime_error("could not find link id: " + name);
	return link->body_index;
}

shared_ptr<RigidBody> RigidBodyManipulator::findJoint(std::string jointname, int robot) const
{
  std::transform(jointname.begin(), jointname.end(), jointname.begin(), ::tolower); // convert to lower case

  vector<bool> name_match;
  name_match.resize(this->bodies.size());
  for(int i = 0;i<this->bodies.size();i++)
  {
  	if (bodies[i]->hasParent()) {
  		string lower_jointname = this->bodies[i]->getJoint().getName();
  		std::transform(lower_jointname.begin(), lower_jointname.end(), lower_jointname.begin(), ::tolower); // convert to lower case
			if(lower_jointname.compare(jointname) == 0)
			{
				name_match[i] = true;
			}
			else
			{
				name_match[i] = false;
			}
  	}
  }
  if(robot != -1)
  {
    for(int i = 0;i<this->bodies.size();i++)
    {
      if(name_match[i])
      {
        name_match[i] = this->bodies[i]->robotnum == robot;
      }
    }
  }
  // Unlike the MATLAB implementation, I am not handling the fixed joints
  int num_match = 0;
  int ind_match = -1;
  for(int i = 0;i<this->bodies.size();i++)
  {
    if(name_match[i])
    {
      num_match++;
      ind_match = i;
    }
  }
  if(num_match != 1)
  {
    cerr<<"couldn't find unique joint "<<jointname<<endl;
    return(nullptr);
  }
  else
  {
    return this->bodies[ind_match];
  }
}

int RigidBodyManipulator::findJointId(const std::string& name, int robot) const
{
	shared_ptr<RigidBody> link = findJoint(name,robot);
	if (link == nullptr)
    throw std::runtime_error("could not find joint id: " + name);
	return link->body_index;
}

std::string RigidBodyManipulator::getBodyOrFrameName(int body_or_frame_id) const
{
  if (body_or_frame_id>=0) {
    return bodies[body_or_frame_id]->linkname;
  } else if (body_or_frame_id < -1) {
    return frames[-body_or_frame_id-2]->name;
  } else {
    return "COM";
  }
}

template <typename Scalar>
GradientVar<Scalar, Eigen::Dynamic, 1> RigidBodyManipulator::positionConstraints(const KinematicsCache<Scalar>& cache, int gradient_order) const
{
  if (gradient_order > 1)
    throw std::runtime_error("only first order gradients are implemented so far (it's trivial to add more)");

  GradientVar<Scalar, Eigen::Dynamic, 1> ret(6*loops.size(), 1, num_positions, gradient_order);
  for (size_t i = 0; i < loops.size(); i++) {
    { // position constraint
      auto ptA_in_B = forwardKin(cache, Vector3d::Zero(), loops[i].frameA->frame_index, loops[i].frameB->frame_index, 0,
                                 gradient_order);
      ret.value().middleRows(6 * i, 3) = ptA_in_B.value();
      if (gradient_order > 0) {
        ret.gradient().value().middleRows(6 * i, 3) = ptA_in_B.gradient().value();
      }
    }
    { // second position constraint (to constrain orientation)
      auto ptA_in_B = forwardKin(cache, loops[i].axis, loops[i].frameA->frame_index, loops[i].frameB->frame_index, 0,
                                 gradient_order);
      ret.value().middleRows(6 * i + 3, 3) = ptA_in_B.value() - loops[i].axis;
      if (gradient_order > 0) {
        ret.gradient().value().middleRows(6 * i + 3, 3) = ptA_in_B.gradient().value();
      }
    }
  }
  return ret;
}

template <typename DerivedA, typename DerivedB, typename DerivedC>
void RigidBodyManipulator::jointLimitConstraints(MatrixBase<DerivedA> const & q, MatrixBase<DerivedB> &phi, MatrixBase<DerivedC> &J) const
{
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  const size_t numFiniteMin = finite_min_index.size();
  const size_t numFiniteMax = finite_max_index.size();

  phi = VectorXd::Zero(numFiniteMin + numFiniteMax);
  J = MatrixXd::Zero(phi.size(), num_positions);
  for (int i = 0; i < numFiniteMin; i++)
  {
    const int fi = finite_min_index[i];
    phi[i] = q[fi] - joint_limit_min[fi];
    J(i, fi) = 1.0;
  }

  for (int i = 0; i < numFiniteMax; i++)
  {
    const int fi = finite_max_index[i];
    phi[i + numFiniteMin] = joint_limit_max[fi] - q[fi];
    J(i + numFiniteMin, fi) = -1.0;
  }
}

size_t RigidBodyManipulator::getNumJointLimitConstraints() const
{
  std::vector<int> finite_min_index;
  std::vector<int> finite_max_index;

  getFiniteIndexes(joint_limit_min, finite_min_index);
  getFiniteIndexes(joint_limit_max, finite_max_index);

  return finite_min_index.size() + finite_max_index.size();
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> RigidBodyManipulator::transformVelocityMappingToPositionDotMapping(
    const KinematicsCache<typename Derived::Scalar>& cache, const Eigen::MatrixBase<Derived>& mat) const
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> ret(mat.rows(), num_positions);
  int ret_col_start = 0;
  int mat_col_start = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      const DrakeJoint& joint = body.getJoint();
      const auto& element = cache.getElement(body);
      ret.middleCols(ret_col_start, joint.getNumPositions()).noalias() = mat.middleCols(mat_col_start, joint.getNumVelocities()) * element.qdot_to_v.value();
      ret_col_start += joint.getNumPositions();
      mat_col_start += joint.getNumVelocities();
    }
  }
  return ret;
}
size_t RigidBodyManipulator::getNumPositionConstraints() const
{
  return loops.size()*6;
}


void RigidBodyManipulator::addFrame(const std::shared_ptr<RigidBodyFrame>& frame)
{
  frames.push_back(frame);
  frame->frame_index=-(static_cast<int>(frames.size())-1)-2; // yuck!!
}

template DLLEXPORT_RBM void RigidBodyManipulator::getContactPositions(const KinematicsCache<double>& cache, MatrixBase <MatrixXd > &, const set<int> &) const;
template DLLEXPORT_RBM void RigidBodyManipulator::getContactPositionsJac(const KinematicsCache<double>& cache, MatrixBase <MatrixXd > &,const set<int> &) const;

template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::centroidalMomentumMatrix(KinematicsCache<double>&, int, const std::set<int>&, bool) const;
template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, 1> RigidBodyManipulator::centroidalMomentumMatrixDotTimesV(KinematicsCache<double>&, int, const std::set<int>&) const;
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMass(KinematicsCache<double>&, int, const std::set<int>&) const;
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION, Eigen::Dynamic> RigidBodyManipulator::centerOfMassJacobian(KinematicsCache<double>&, int, const std::set<int>&, bool) const;
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMassJacobianDotTimesV(KinematicsCache<double>&, int, const std::set<int>&) const;
template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::geometricJacobian<double>(const KinematicsCache<double>&, int, int, int, int, bool, std::vector<int>*) const;
template DLLEXPORT_RBM GradientVar<AutoDiffScalar<VectorXd>, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::geometricJacobian< AutoDiffScalar<VectorXd> >(KinematicsCache< AutoDiffScalar<VectorXd> > const&, int, int, int, int, bool, vector<int>*) const;
template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, 1> RigidBodyManipulator::geometricJacobianDotTimesV(const KinematicsCache<double>&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> RigidBodyManipulator::relativeTransform(const KinematicsCache<double>&, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKin<Matrix3Xd >(const KinematicsCache<double>&, const MatrixBase<Matrix3Xd> &, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKin<Eigen::MatrixXd >(const KinematicsCache<double>&, Eigen::MatrixBase<Eigen::MatrixXd> const &, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, 1> RigidBodyManipulator::forwardKin<Vector3d>(const KinematicsCache<double>&, const MatrixBase<Vector3d> &, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian(const KinematicsCache<double>&, const MatrixBase<Matrix3Xd> &, int, int, int, bool, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, 1, true> >(const KinematicsCache<double>&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, 1, true> > const &, int, int, int, bool, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian<Eigen::MatrixXd >(const KinematicsCache<double>&, Eigen::MatrixBase<Eigen::MatrixXd> const &, int, int, int, bool, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian<Eigen::Vector3d >(const KinematicsCache<double>&, Eigen::MatrixBase<Eigen::Vector3d> const &, int, int, int, bool, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinJacobian<Eigen::Block<Eigen::Matrix3Xd, 3, 1, true> >(
    const KinematicsCache<double>&, Eigen::MatrixBase<Eigen::Block<Eigen::Matrix3Xd, 3, 1, true> > const &, int, int, int, bool, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinPositionGradient(const KinematicsCache<double>&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, 1> RigidBodyManipulator::forwardJacDotTimesV(const KinematicsCache<double>&, const MatrixBase< Matrix<double, 3, Eigen::Dynamic> >&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, 1> RigidBodyManipulator::forwardJacDotTimesV(const KinematicsCache<double>&, const MatrixBase< Matrix<double, 3, 1> >&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::massMatrix(KinematicsCache<double>&, int) const;
template DLLEXPORT_RBM GradientVar<AutoDiffScalar<VectorXd>, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::massMatrix<AutoDiffScalar<VectorXd> >(KinematicsCache<AutoDiffScalar<VectorXd> >&, int) const;
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, 1> RigidBodyManipulator::inverseDynamics(KinematicsCache<double>&, std::map<int, std::unique_ptr<GradientVar<double, TWIST_SIZE, 1> > >& f_ext, GradientVar<double, Eigen::Dynamic, 1>* vd, int) const;
template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, 1> RigidBodyManipulator::relativeTwist<double>(const KinematicsCache<double>&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, 6, Dynamic> RigidBodyManipulator::worldMomentumMatrix<double>(KinematicsCache<double>&, int, const std::set<int>&, bool) const;
template DLLEXPORT_RBM GradientVar<double, 6, 1> RigidBodyManipulator::transformSpatialAcceleration<double>(const KinematicsCache<double>&, GradientVar<double, 6, 1> const&, int, int, int, int) const;
template DLLEXPORT_RBM GradientVar<double, 6, 1> RigidBodyManipulator::worldMomentumMatrixDotTimesV<double>(KinematicsCache<double>&, int, const std::set<int>&) const;

template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, 1> RigidBodyManipulator::positionConstraints(const KinematicsCache<double>&, int) const;

template DLLEXPORT_RBM void RigidBodyManipulator::jointLimitConstraints(MatrixBase<VectorXd> const &, MatrixBase<VectorXd> &, MatrixBase<MatrixXd> &) const;
template DLLEXPORT_RBM void RigidBodyManipulator::jointLimitConstraints(MatrixBase< Map<VectorXd> > const &, MatrixBase<VectorXd> &, MatrixBase<MatrixXd> &) const;
template DLLEXPORT_RBM void RigidBodyManipulator::jointLimitConstraints(MatrixBase< Map<VectorXd> > const &, MatrixBase< Map<VectorXd> > &, MatrixBase< Map<MatrixXd> > &) const;

template DLLEXPORT_RBM std::pair<Eigen::Vector3d, double> RigidBodyManipulator::resolveCenterOfPressure(const KinematicsCache<double>&, const std::vector<ForceTorqueMeasurement> &, const Eigen::MatrixBase<Eigen::Vector3d> &, const Eigen::MatrixBase<Eigen::Vector3d> &) const;
