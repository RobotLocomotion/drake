#include "mex.h"
#include "drakeUtil.h"
#include "IKoptions.h"
#include "RigidBodyManipulator.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
  if(nlhs != 1 || nrhs < 3)
  {
    mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Usage ptr = updatePtrIKoptionsmex(ikoptions_ptr,field,varargin)");
  }
  IKoptions* ikoptions = (IKoptions*) getDrakeMexPointer(prhs[0]);
  mwSize strlen = mxGetNumberOfElements(prhs[1])+1;
  char* field = new char[strlen];
  mxGetString(prhs[1],field,strlen);
  string field_str(field);
  int nq = ikoptions->getRobotPtr()->num_dof;
  IKoptions* ikoptions_new = new IKoptions(*ikoptions);
  if(field_str == "Q")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Q must be nq x nq double matrix"); 
    }
    MatrixXd Q(nq,nq);
    memcpy(Q.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    ikoptions_new->setQ(Q);
  }
  else if(field_str == "Qa")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Qa must be nq x nq double matrix"); 
    }
    MatrixXd Qa(nq,nq);
    memcpy(Qa.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    ikoptions_new->setQa(Qa);
  }
  else if(field_str == "Qv")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Qv must be nq x nq double matrix"); 
    }
    MatrixXd Qv(nq,nq);
    memcpy(Qv.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    ikoptions_new->setQv(Qv);
  }
  else if (field_str == "debug")
  {
    if(!mxIsLogicalScalar(prhs[2]))
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","debug must be a single boolean");
    }
    bool flag = *mxGetLogicals(prhs[2]);
    ikoptions_new = new IKoptions(*ikoptions);
    ikoptions_new->setDebug(flag);
  }
  else if(field_str == "sequentialSeedFlag")
  {
    if(!mxIsLogicalScalar(prhs[2]))
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","sequentialSeedFlag must be a boolean scalar");
    }
    bool flag = *mxGetLogicals(prhs[2]);
    ikoptions_new->setSequentialSeedFlag(flag);
  }
  else if(field_str == "majorOptimalityTolerance")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorOptimalityTolerance must be a double scalar");
    }
    double tol = mxGetScalar(prhs[2]);
    if(tol<=0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorOptimalityTolerance must be positive");
    }
    ikoptions_new->setMajorOptimalityTolerance(tol);
  }
  else if(field_str == "majorFeasibilityTolerance")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorFeasibilityTolerance must be a double scalar");
    }
    double tol = mxGetScalar(prhs[2]);
    if(tol<=0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorFeasibilityTolerance must be positive");
    }
    ikoptions_new->setMajorFeasibilityTolerance(tol);
  }
  else if(field_str == "superbasicsLimit")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","SuperbasicsLimit must be an integer scalar");
    }
    int limit = (int) mxGetScalar(prhs[2]);
    if(limit <=0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","SuperbasicsLimit must be positive");
    }
    ikoptions_new->setSuperbasicsLimit(limit);
  }
  else if(field_str == "majorIterationsLimit")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorIterationsLimit must be an integer scalar");
    }
    int limit = (int) mxGetScalar(prhs[2]);
    if(limit <=0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","MajorIterationsLimit must be positive");
    }
    ikoptions_new->setMajorIterationsLimit(limit);
  }
  else if(field_str == "iterationsLimit")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","IterationsLimit must be an integer scalar");
    }
    int limit = (int) mxGetScalar(prhs[2]);
    if(limit <=0)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","IterationsLimit must be positive");
    }
    ikoptions_new->setIterationsLimit(limit);
  }
  else if(field_str == "fixInitialState")
  {
    if(!mxIsLogicalScalar(prhs[2]))
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","fixInitialState must be a boolean scalar");
    }
    bool flag = *mxGetLogicals(prhs[2]);
    ikoptions_new->setFixInitialState(flag);
  }
  else if(field_str == "q0")
  {
    if(!mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]) || mxGetM(prhs[2]) != nq || mxGetM(prhs[3]) != nq || mxGetN(prhs[2]) != 1 || mxGetN(prhs[3]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","q0_lb,q0_ub must be nq x 1 column double vector");
    }
    VectorXd lb(nq);
    VectorXd ub(nq);
    memcpy(lb.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
    memcpy(ub.data(),mxGetPr(prhs[3]),sizeof(double)*nq);
    ikoptions_new->setq0(lb,ub);
  }
  else if(field_str == "qd0")
  {
    if(!mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]) || mxGetM(prhs[2]) != nq || mxGetM(prhs[3]) != nq || mxGetN(prhs[2]) != 1 || mxGetN(prhs[3]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","qd0_lb,qd0_ub must be nq x 1 column double vector");
    }
    VectorXd lb(nq);
    VectorXd ub(nq);
    memcpy(lb.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
    memcpy(ub.data(),mxGetPr(prhs[3]),sizeof(double)*nq);
    ikoptions_new->setqd0(lb,ub);
  }
  else if(field_str == "qdf")
  {
    if(!mxIsNumeric(prhs[2]) || !mxIsNumeric(prhs[3]) || mxGetM(prhs[2]) != nq || mxGetM(prhs[3]) != nq || mxGetN(prhs[2]) != 1 || mxGetN(prhs[3]) != 1)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","qdf_lb,qdf_ub must be nq x 1 column double vector");
    }
    VectorXd lb(nq);
    VectorXd ub(nq);
    memcpy(lb.data(),mxGetPr(prhs[2]),sizeof(double)*nq);
    memcpy(ub.data(),mxGetPr(prhs[3]),sizeof(double)*nq);
    ikoptions_new->setqdf(lb,ub);
  }
  else if(field_str == "additionaltSamples")
  {
    RowVectorXd t_samples;
    if(mxGetNumberOfElements(prhs[2]) == 0)
    {
      t_samples.resize(0);
    }
    else
    {
      if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != 1)
      {
        mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","additional_tSamples must be a row double vector");
      }
      t_samples.resize(mxGetN(prhs[2]));
      memcpy(t_samples.data(),mxGetPr(prhs[2]),sizeof(double)*mxGetN(prhs[2]));
    }
    ikoptions_new->setAdditionaltSamples(t_samples);
  }
  else if(field_str == "robot")
  {
    RigidBodyManipulator* new_robot = (RigidBodyManipulator*) getDrakeMexPointer(prhs[2]);
    ikoptions_new->updateRobot(new_robot);
  }
  else
  {
    mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Unsupported field");
  }
  plhs[0] = createDrakeMexPointer((void*) ikoptions_new,"deletePtrIKoptionsmex","IKoptions");
  delete[] field;
}
