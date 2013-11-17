#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include <Eigen/Dense>
#include "RigidBodyConstraint.h"
#include "RigidBodyManipulator.h"
#include "constructPtrDrakeConstraint.h"
#include <cstdio>

using namespace Eigen;
using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  //DEBUG
  //cout << "nrhs = " << nrhs << endl;
  //END_DEBUG
  if(nrhs < 4 || nrhs > 5)
  {
    mexErrMsgIdAndTxt("Drake:constructPtrAllBodiesClosestDistanceConstraintmex",
        "Usage ptr = constructPtrAllBodiesClosestDistanceConstraintmex(obj.robot.mex_model_ptr,lb,ub,tspan)");
  }
  RigidBodyManipulator* model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  Vector2d tspan;
  if(nrhs == 4)
  {
    tspan<< -mxGetInf(), mxGetInf();
  }
  else
  {
    drakeKinCnstParseTspan(prhs[3],tspan);
  }

  double lb = (double) mxGetScalar(prhs[1]);
  double ub = (double) mxGetScalar(prhs[2]);

  auto cnst = new AllBodiesClosestDistanceConstraint(model,lb,ub,tspan);
  plhs[0] = createDrakeConstraintMexPointer((void*)cnst,"deleteRigidBodyConstraintmex",
                                  "AllBodiesClosestDistanceConstraint");
}
