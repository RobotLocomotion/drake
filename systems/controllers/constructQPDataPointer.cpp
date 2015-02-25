#include "QPCommon.h"
#include <Eigen/StdVector>
// #include <limits>
// #include <cmath>
// #include "drakeUtil.h"

using namespace std;


void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = constructQPDataPointerMex(robot_obj,B,umin,umax,terrain_map_ptr,gurobi_opts);");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");

  int narg = 0;
  struct QPControllerData* pdata;
  pdata = new struct QPControllerData;

  if (!mxIsNumeric(prhs[narg]) || mxGetNumberOfElements(prhs[narg])!=1)
      mexErrMsgIdAndTxt("Drake:constructQPDataPointer:BadInputs","the first argument should be the robot mex ptr");

  memcpy(&(pdata->r), mxGetData(prhs[narg]), sizeof(pdata->r));
  narg++;

  pdata->B.resize(mxGetM(prhs[narg]),mxGetN(prhs[narg]));
  memcpy(pdata->B.data(),mxGetPr(prhs[narg]),sizeof(double)*mxGetM(prhs[narg])*mxGetN(prhs[narg]));
  narg++;

  int nq = pdata->r->num_dof, nu = pdata->B.cols();

  pdata->umin.resize(nu);
  pdata->umax.resize(nu);
  memcpy(pdata->umin.data(),mxGetPr(prhs[narg++]),sizeof(double)*nu);
  memcpy(pdata->umax.data(),mxGetPr(prhs[narg++]),sizeof(double)*nu);

  pdata->B_act.resize(nu,nu);
  pdata->B_act = pdata->B.bottomRows(nu);

  if (!mxIsNumeric(prhs[narg]) || mxGetNumberOfElements(prhs[narg])!=1)
    mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs","this should be a map pointer");

  memcpy(&pdata->map_ptr,mxGetPr(prhs[narg]),sizeof(pdata->map_ptr));
  narg++;

  if (!pdata->map_ptr)
    mexWarnMsgTxt("Map ptr is NULL. Assuming flat ground.");

  //create gurobi environment
  int error = GRBloadenv(&(pdata->env),NULL);
  if (error)
    mexErrMsgTxt("Cannot load gurobi environment.");


  // set solver params (http://www.gurobi.com/documentation/5.5/reference-manual/node798#sec:Parameters)
  const mxArray* psolveropts = prhs[narg];
  int method = (int) mxGetScalar(myGetField(psolveropts,"method"));
  CGE ( GRBsetintparam(pdata->env,"outputflag",0), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  // CGE ( GRBsetintparam(pdata->env,"method",method), pdata->env );
  CGE ( GRBsetintparam(pdata->env,"presolve",0), pdata->env );
  if (method==2) {
    CGE ( GRBsetintparam(pdata->env,"bariterlimit",20), pdata->env );
    CGE ( GRBsetintparam(pdata->env,"barhomogeneous",0), pdata->env );
    CGE ( GRBsetdblparam(pdata->env,"barconvtol",0.0005), pdata->env );
  }

  mxClassID cid;
  if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
  else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
  else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
  
  plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
  memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));

  
  // preallocate some memory
  pdata->H.resize(nq,nq);
  pdata->H_float.resize(6,nq);
  pdata->H_act.resize(nu,nq);

  pdata->C.resize(nq);
  pdata->C_float.resize(6);
  pdata->C_act.resize(nu);

  pdata->J.resize(3,nq);
  pdata->Jdot.resize(3,nq);
  pdata->J_xy.resize(2,nq);
  pdata->Jdot_xy.resize(2,nq);
  pdata->Hqp.resize(nq,nq);
  pdata->fqp.resize(nq);
  pdata->Ag.resize(6,nq);
  pdata->Agdot.resize(6,nq);
  pdata->Ak.resize(3,nq);
  pdata->Akdot.resize(3,nq);

  pdata->vbasis_len = 0;
  pdata->cbasis_len = 0;
  pdata->vbasis = NULL;
  pdata->cbasis = NULL;
  return;
}












