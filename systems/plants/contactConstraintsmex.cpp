#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;
/*
 * mex interface for contact constraints
 * pulled from systems/controllers/controlUtil.cpp
 *
 * MATLAB signature:
 * [phi,normal,d,xA,xB,idxA,idxB,mu,n,D,dn,dD] = 
 *     contactConstraints(obj,kinsol,allow_multiple_contacts, active_collision_options)
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) 
{
  // first get the model_ptr back from matlab
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  cout << model->num_dof << endl;
}
