#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * provides a projection matrix which maps from the matlab robot positions to the positions of robot loaded with the c++ parser
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:matlabToCppProjectionmex:NotEnoughInputs","Usage matlabToCppProjectionmex(model_ptr, urdf_file)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *matlab_model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  char urdf_file[1000]; mxGetString(prhs[1],urdf_file,1000);
  RigidBodyManipulator* cpp_model = new RigidBodyManipulator(urdf_file);

  plhs[0] = mxCreateDoubleMatrix(cpp_model->num_positions,matlab_model->num_positions,mxREAL);
  Map<MatrixXd> P(mxGetPr(plhs[0]),cpp_model->num_positions,matlab_model->num_positions);
  P = MatrixXd::Zero(cpp_model->num_positions,matlab_model->num_positions);

  // now compute the actual projection:
  for (int i=0; i<cpp_model->num_bodies; i++) {
  	if (cpp_model->bodies[i]->hasParent() && cpp_model->bodies[i]->getJoint().getNumPositions()>0) {
  		shared_ptr<RigidBody> b = matlab_model->findJoint(cpp_model->bodies[i]->getJoint().getName());
  		if (b==nullptr) continue;
  		for (int j=0;j<b->getJoint().getNumPositions(); j++) {
  			P(cpp_model->bodies[i]->position_num_start+j,b->position_num_start+j) = 1.0;
  		}
  	}
  }

  delete cpp_model;
}
