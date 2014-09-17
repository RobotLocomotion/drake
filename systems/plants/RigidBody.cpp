
#include "RigidBodyManipulator.h"
#include <stdexcept>

using namespace std;

const int defaultRobotNum[1] = {0};
const set<int> RigidBody::defaultRobotNumSet(defaultRobotNum,defaultRobotNum+1);

RigidBody::RigidBody(void)
{
	dofnum=0;
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
}


#if !defined(_WIN32) && !defined(_WIN64)

void RigidBody::setJoint(std::unique_ptr<DrakeJoint> joint)
{
  this->joint = move(joint);
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
#endif

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

ostream &operator<<( ostream &out, const RigidBody &b)
{
	out << "RigidBody(" << b.linkname << "," << b.jointname << ")";
	return out;
}



