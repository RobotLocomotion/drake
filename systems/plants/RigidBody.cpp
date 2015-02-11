
#include "RigidBodyManipulator.h"
#include <stdexcept>

using namespace std;

const int defaultRobotNum[1] = {0};
const set<int> RigidBody::defaultRobotNumSet(defaultRobotNum,defaultRobotNum+1);

RigidBody::RigidBody(void) :
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
    dJdotVdv(TWIST_SIZE, 0)
{
	dofnum = 0;
	velocity_num_start = 0;
	mass = 0.0;
	floating = 0;
	com << Vector3d::Zero(), 1;
	T = Matrix4d::Identity();
	Tdot = Matrix4d::Zero();
	Ttree = Matrix4d::Identity();
	T_body_to_joint = Matrix4d::Identity();
}

void RigidBody::setN(int n) {
  dTdq = MatrixXd::Zero(3*n,4);
  dTdqdot = MatrixXd::Zero(3*n,4);
  ddTdqdq = MatrixXd::Zero(3*n*n,4);

  dJdq.resize(J.size(), n);
  dTdq_new.resize(T.size(), n);
  dtwistdq.resize(twist.size(), n);

  dJdotVdq.resize(TWIST_SIZE, n);
  dJdotVdv.resize(TWIST_SIZE, n); // TODO: nv

  dqdot_to_v_dq.resize(Eigen::NoChange, n);
  dv_to_qdot_dq.resize(Eigen::NoChange, n);
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

void RigidBody::computeAncestorDOFs(RigidBodyManipulator* model)
{
  if (dofnum>=0) {
    int i,j;
    if (parent>=0) {
      ancestor_dofs = model->bodies[parent]->ancestor_dofs;
      ddTdqdq_nonzero_rows = model->bodies[parent]->ddTdqdq_nonzero_rows;
    }

    if (floating==1) {
    	for (j=0; j<6; j++) {
    		ancestor_dofs.insert(dofnum+j);
    		for (i=0; i<3*model->NB; i++) {
    			ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum + j);
    			ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i + j);
    		}
    	}
    } else if (floating==2) {
    	for (j=0; j<7; j++) {
    		ancestor_dofs.insert(dofnum+j);
    		for (i=0; i<3*model->NB; i++) {
    			ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum + j);
    			ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i + j);
    		}
    	}
    }
    else {
    	ancestor_dofs.insert(dofnum);
    	for (i=0; i<3*model->NB; i++) {
    		ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum);
    		ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i);
    	}
    }


    // compute matrix blocks
    IndexRange ind;  ind.start=-1; ind.length=0;
    for (i=0; i<3*model->NB*model->NB; i++) {
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
  return parent >= 0;
}

ostream &operator<<( ostream &out, const RigidBody &b)
{
	out << "RigidBody(" << b.linkname << "," << b.jointname << ")";
	return out;
}



