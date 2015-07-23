
#include "RigidBodyManipulator.h"
#include <stdexcept>

using namespace std;

const int defaultRobotNum[1] = {0};
const set<int> RigidBody::defaultRobotNumSet(defaultRobotNum,defaultRobotNum+1);

RigidBody::RigidBody(void) :
    parent(nullptr),
    S(TWIST_SIZE, 0),
    dSdqi(0, 0),
    J(TWIST_SIZE, 0),
    dJdq(0, 0),
    qdot_to_v(0, 0),
    dqdot_to_v_dqi(0, 0),
    dqdot_to_v_dq(0, 0),
    v_to_qdot(0, 0),
    dv_to_qdot_dqi(0, 0),
    dv_to_qdot_dq(0, 0),
    T_new(Isometry3d::Identity()),
    dTdq_new(HOMOGENEOUS_TRANSFORM_SIZE, 0),
    twist(TWIST_SIZE, 1),
    dtwistdq(TWIST_SIZE, 0),
    SdotV(TWIST_SIZE, 1),
    dSdotVdqi(TWIST_SIZE, 0),
    dSdotVdvi(TWIST_SIZE, 0),
    JdotV(TWIST_SIZE, 1),
    dJdotVdq(TWIST_SIZE, 0),
    dJdotVdv(TWIST_SIZE, 0),
    collision_filter_group(DrakeCollision::DEFAULT_GROUP),
    collision_filter_ignores(DrakeCollision::NONE_MASK)
{
  robotnum = 0;
	position_num_start = 0;
	velocity_num_start = 0;
	body_index = 0;
	mass = 0.0;
	floating = 0;
	pitch = 0;
	com = Vector3d::Zero();
	I << Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
	T = Matrix4d::Identity();
	Tdot = Matrix4d::Zero();
	Ttree = Matrix4d::Identity();
	T_body_to_joint = Matrix4d::Identity();
}

void RigidBody::setN(int nq, int nv) {
  dTdq = MatrixXd::Zero(3*nq,4);
  dTdqdot = MatrixXd::Zero(3*nq,4);
  ddTdqdq = MatrixXd::Zero(3*nq*nq,4);

  dJdq.resize(J.size(), nq);
  dTdq_new.resize(T.size(), nq);
  dtwistdq.resize(twist.size(), nq);

  dJdotVdq.resize(TWIST_SIZE, nq);
  dJdotVdv.resize(TWIST_SIZE, nv);

  dqdot_to_v_dq.resize(Eigen::NoChange, nq);
  dv_to_qdot_dq.resize(Eigen::NoChange, nq);
}


void RigidBody::setJoint(std::unique_ptr<DrakeJoint> new_joint)
{
  this->joint = move(new_joint);

  S.resize(TWIST_SIZE, joint->getNumVelocities());
  dSdqi.resize(S.size(), joint->getNumPositions());
  J.resize(TWIST_SIZE, joint->getNumVelocities());
  qdot_to_v.resize(joint->getNumVelocities(), joint->getNumPositions()),
  dqdot_to_v_dqi.resize(qdot_to_v.size(), joint->getNumPositions()),
  dqdot_to_v_dq.resize(qdot_to_v.size(), Eigen::NoChange);
  v_to_qdot.resize(joint->getNumPositions(), joint->getNumVelocities()),
  dv_to_qdot_dqi.resize(v_to_qdot.size(), joint->getNumPositions());
  dv_to_qdot_dq.resize(v_to_qdot.size(), Eigen::NoChange);
  dSdotVdqi.resize(TWIST_SIZE, joint->getNumPositions());
  dSdotVdvi.resize(TWIST_SIZE, joint->getNumVelocities());
}

const DrakeJoint& RigidBody::getJoint() const
{
  if (joint) {
    return (*joint);
  }
  else {
    throw runtime_error("Joint is not initialized");
  }
}

void RigidBody::setupOldKinematicTree(RigidBodyManipulator* model)
{
  // note: could also setup the featherstone data structures (in RBM) to use the old dynamics; but that doesn't add any additional functionality and is about to be removed

  if (hasParent()) {
    const DrakeJoint& joint = getJoint();
    joint.setupOldKinematicTree(model,body_index,position_num_start,velocity_num_start);
  }
}

void RigidBody::computeAncestorDOFs(RigidBodyManipulator* model)
{
  if (position_num_start>=0) {
    int i,j;
    if (parent!=nullptr) {
      ancestor_dofs = parent->ancestor_dofs;
      ddTdqdq_nonzero_rows = parent->ddTdqdq_nonzero_rows;
    }

    if (floating==1) {
    	for (j=0; j<6; j++) {
    		ancestor_dofs.insert(position_num_start+j);
    		for (i=0; i<3*model->num_positions; i++) {
    			ddTdqdq_nonzero_rows.insert(i*model->num_positions + position_num_start + j);
    			ddTdqdq_nonzero_rows.insert(3*model->num_positions*position_num_start + i + j);
    		}
    	}
    } else if (floating==2) {
    	for (j=0; j<7; j++) {
    		ancestor_dofs.insert(position_num_start+j);
    		for (i=0; i<3*model->num_positions; i++) {
    			ddTdqdq_nonzero_rows.insert(i*model->num_positions + position_num_start + j);
    			ddTdqdq_nonzero_rows.insert(3*model->num_positions*position_num_start + i + j);
    		}
    	}
    }
    else {
    	ancestor_dofs.insert(position_num_start);
    	for (i=0; i<3*model->num_positions; i++) {
    		ddTdqdq_nonzero_rows.insert(i*model->num_positions + position_num_start);
    		ddTdqdq_nonzero_rows.insert(3*model->num_positions*position_num_start + i);
    	}
    }


    // compute matrix blocks
    IndexRange ind;  ind.start=-1; ind.length=0;
    for (i=0; i<3*model->num_positions*model->num_positions; i++) {
      if (ddTdqdq_nonzero_rows.find(i)!=ddTdqdq_nonzero_rows.end()) {
        if (ind.start<0) ind.start=i;
      } else {
        if (ind.start>=0) {
          ind.length = i-ind.start;
          ddTdqdq_nonzero_rows_grouped.insert(ind);
          ind.start = -1;
        }
      }
    }
    if (ind.start>=0) {
      ind.length = i-ind.start;
      ddTdqdq_nonzero_rows_grouped.insert(ind);
    }
  }
}

bool RigidBody::hasParent() const {
  return parent !=nullptr;
}


void RigidBody::addVisualElement(const DrakeShapes::VisualElement& element)
{
  visual_elements.push_back(element);
}

const DrakeShapes::VectorOfVisualElements& RigidBody::getVisualElements() const
{
  return visual_elements;
}

void RigidBody::setCollisionFilter(const DrakeCollision::bitmask& group, 
                                   const DrakeCollision::bitmask& ignores)
{
  setCollisionFilterGroup(group);
  setCollisionFilterIgnores(ignores);
}

bool RigidBody::appendCollisionElementIdsFromThisBody(const string& group_name, vector<DrakeCollision::ElementId>& ids) const
{
  auto group_ids_iter = collision_element_groups.find(group_name);
  if (group_ids_iter != collision_element_groups.end()) {
    ids.reserve(ids.size() + distance(group_ids_iter->second.begin(), group_ids_iter->second.end()));
    ids.insert(ids.end(), group_ids_iter->second.begin(), group_ids_iter->second.end());
    return true;
  } else {
    return false;
  }
}

bool RigidBody::appendCollisionElementIdsFromThisBody(vector<DrakeCollision::ElementId>& ids) const
{
  ids.reserve(ids.size() + collision_element_ids.size());
  ids.insert(ids.end(), collision_element_ids.begin(), collision_element_ids.end());
  return true;
}

RigidBody::CollisionElement::
CollisionElement( const CollisionElement& other)
  : DrakeCollision::Element(other), body(other.getBody()) {}

  RigidBody::CollisionElement::
CollisionElement( const Matrix4d& T_element_to_link, std::shared_ptr<RigidBody> body)
  : DrakeCollision::Element(T_element_to_link), body(body) {}

  RigidBody::CollisionElement::
CollisionElement(const DrakeShapes::Geometry& geometry,
    const Matrix4d& T_element_to_link, std::shared_ptr<RigidBody> body)
  : DrakeCollision::Element(geometry, T_element_to_link), body(body) {}

RigidBody::CollisionElement* RigidBody::CollisionElement::clone() const
{
  return new CollisionElement(*this);
}

const std::shared_ptr<RigidBody>& RigidBody::CollisionElement:: getBody() const
{
  return this->body;
}

bool RigidBody::CollisionElement::collidesWith( const DrakeCollision::Element* other) const
{
  //DEBUG
  //cout << "RigidBody::CollisionElement::collidesWith: START" << endl;
  //END_DEBUG
  auto other_rb = dynamic_cast<const RigidBody::CollisionElement*>(other);
  bool collides = true;
  if (other_rb != nullptr) {
    collides = this->body->collidesWith(other_rb->body);
    //DEBUG
    //cout << "RigidBody::CollisionElement::collidesWith:" << endl;
    //cout << "  " << this->body->linkname << " & " << other_rb->body->linkname;
    //cout << ": collides = " << collides << endl;
    //END_DEBUG
  }   
  return collides;
}

ostream &operator<<( ostream &out, const RigidBody &b)
{
  out << "RigidBody(" << b.linkname << "," << b.jointname << ")";
  return out;
}
