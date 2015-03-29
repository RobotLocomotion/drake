#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * compares C++ robots generated via the matlab constructModelmex with the same robot generated via the c++ parser
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:compareParsersmex:NotEnoughInputs","Usage compareParsersmex(model_ptr, urdf_file, floating_base_type)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *matlab_model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  char urdf_file[1000]; mxGetString(prhs[1],urdf_file,1000);
  char floating_base_type[100] = "rpy";
  if (nrhs>2) mxGetString(prhs[2],floating_base_type,100);
  RigidBodyManipulator* cpp_model = new RigidBodyManipulator(urdf_file,floating_base_type);

/*
  for (int i=0; i<cpp_model->num_bodies; i++) {
  	if (cpp_model->bodies[i]->hasParent())
  		cout << cpp_model->bodies[i]->getJoint().getName() << endl;
  }

  cout << " --- " << endl;

  for (int i=0; i<matlab_model->num_bodies; i++) {
  	if (matlab_model->bodies[i]->hasParent())
  		cout << matlab_model->bodies[i]->getJoint().getName() << endl;
  }
*/

  // Compute coordinate transform between the two models (in case they are not identical)
  MatrixXd P = MatrixXd::Zero(cpp_model->num_positions,matlab_model->num_positions);  // projection from the coordinates of matlab_model to cpp_model
  for (int i=0; i<cpp_model->num_bodies; i++) {
  	if (cpp_model->bodies[i]->hasParent() && cpp_model->bodies[i]->getJoint().getNumPositions()>0) {
  		shared_ptr<RigidBody> b = matlab_model->findJoint(cpp_model->bodies[i]->getJoint().getName());
  		if (b==nullptr) continue;
  		for (int j=0;j<b->getJoint().getNumPositions(); j++) {
  			P(cpp_model->bodies[i]->position_num_start+j,b->position_num_start+j) = 1.0;
  		}
  	}
  }

  std::default_random_engine generator;  // note: gets the same seed every time, would have to seed it manually
	std::normal_distribution<double> distribution(0.0,1.0);
  for (int trial=0; trial<1; trial++) {
  	// generate random q
  	VectorXd matlab_q(matlab_model->num_positions), cpp_q(cpp_model->num_positions);
		matlab_model->getRandomConfiguration(matlab_q,generator);
		cpp_q.noalias() = P*matlab_q;

		if ((matlab_model->num_positions != matlab_model->num_velocities) ||
				(cpp_model->num_positions != cpp_model->num_velocities)) {
			mexErrMsgTxt("ERROR: num_positions!=num_velocities have to generate another P for this case");
		}

		// generate random v
		VectorXd matlab_v(matlab_model->num_velocities), cpp_v(cpp_model->num_velocities);
		for (int i=0; i<matlab_model->num_velocities; i++) matlab_v[i] = distribution(generator);
		cpp_v.noalias() = P*matlab_v;

		{ // run new and old kinematics
			matlab_model->setUseNewKinsol(false);
			cpp_model->setUseNewKinsol(false);

			matlab_model->doKinematics(matlab_q,false,matlab_v);
			cpp_model->doKinematics(cpp_q,false,cpp_v);

			matlab_model->setUseNewKinsol(true);
			cpp_model->setUseNewKinsol(true);

			matlab_model->doKinematicsNew(matlab_q,matlab_v);
			cpp_model->doKinematicsNew(cpp_q,cpp_v);
		}

		{ // compare joint limits
			if (matlab_model->joint_limit_max.isApprox(P*cpp_model->joint_limit_max,1e-8)) {
				mexErrMsgTxt("ERROR: joint_limit_max doesn't match");
			}
			if (matlab_model->joint_limit_min.isApprox(P*cpp_model->joint_limit_min,1e-8)) {
				mexErrMsgTxt("ERROR: joint_limit_min doesn't match");
			}
		}

		{ // compare position constraints
			auto matlab_phi = matlab_model->positionConstraintsNew<double>(1);
			auto cpp_phi = cpp_model->positionConstraintsNew<double>(1);

			if (!matlab_phi.value().isApprox(cpp_phi.value(),1e-8)) {
				cout << endl;
				for (int i=0; i<matlab_model->loops.size(); i++) {
					// could be the same error vector, but in a different coordinate frame
					Vector3d m_phi = matlab_phi.value().middleRows(i*3,3);
					Vector3d c_phi = cpp_phi.value().middleRows(i*3,3);

					// rotate the vector into the non-fixed coordinate system
					shared_ptr<RigidBody> b = cpp_model->loops[i].bodyB;
					while (b->hasParent() && b->getJoint().getNumPositions()==0) {
						c_phi = b->getJoint().getTransformToParentBody().matrix().topLeftCorner(3,3) * c_phi;
						b = b->parent;
					}
					if (!m_phi.isApprox(c_phi,1e-8)) {
						mexErrMsgTxt("ERROR: phi doesn't match (see terminal output)");
					}
				}
			}

			matlab_model->setUseNewKinsol(false);
			cpp_model->setUseNewKinsol(false);

			VectorXd matlab_phi_old(matlab_model->loops.size()*3), cpp_phi_old(cpp_model->loops.size()*3);
			MatrixXd matlab_dphi_old(matlab_model->loops.size()*3,matlab_model->num_positions), cpp_dphi_old(cpp_model->loops.size()*3,cpp_model->num_positions);
			matlab_model->positionConstraints(matlab_phi_old,matlab_dphi_old);
			cpp_model->positionConstraints(cpp_phi_old,cpp_dphi_old);

			if (!matlab_phi.value().isApprox(matlab_phi_old,1e-8)) {
				cout << "matlab phi new:" << matlab_phi.value().transpose() << endl;
				cout << "matlab phi old:" << matlab_phi_old.transpose() << endl;
				mexErrMsgTxt("ERROR: matlab phi old doesn't match new");
			}
			if (!cpp_phi.value().isApprox(cpp_phi_old,1e-8)) {
				mexErrMsgTxt("ERROR: cpp phi old doesn't match new");
			}

			if (!matlab_phi.gradient().value().isApprox(matlab_dphi_old,1e-8)) {
				mexErrMsgTxt("ERROR: matlab phi old gradient doesn't match new");
			}
			if (!cpp_phi.gradient().value().isApprox(cpp_dphi_old,1e-8)) {
				mexErrMsgTxt("ERROR: cpp phi old gradient doesn't match new");
			}
		}
  }

  delete cpp_model;
}
