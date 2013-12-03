#include "mex.h"
#include "drakeUtil.h"
#include "IKoptions.h"

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
  if(field_str == "Q")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Q must be nq x nq double matrix"); 
    }
    MatrixXd Q(nq,nq);
    memcpy(Q.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    IKoptions* ikoptions_new = new IKoptions(*ikoptions);
    ikoptions_new->setQ(Q);
    plhs[0] = createDrakeMexPointer((void*) ikoptions_new,"deletePtrIKoptionsmex","IKoptions");
  }
  else if(field_str == "Qa")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Qa must be nq x nq double matrix"); 
    }
    MatrixXd Qa(nq,nq);
    memcpy(Qa.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    IKoptions* ikoptions_new = new IKoptions(*ikoptions);
    ikoptions_new->setQa(Qa);
    plhs[0] = createDrakeMexPointer((void*) ikoptions_new,"deletePtrIKoptionsmex","IKoptions");
  }
  else if(field_str == "Qv")
  {
    if(!mxIsNumeric(prhs[2]) || mxGetM(prhs[2]) != nq || mxGetN(prhs[2]) != nq)
    {
      mexErrMsgIdAndTxt("Drake:updatePtrIKoptionsmex:BadInputs","Qv must be nq x nq double matrix"); 
    }
    MatrixXd Qv(nq,nq);
    memcpy(Qv.data(),mxGetPr(prhs[2]),sizeof(double)*nq*nq);
    IKoptions* ikoptions_new = new IKoptions(*ikoptions);
    ikoptions_new->setQv(Qv);
    plhs[0] = createDrakeMexPointer((void*) ikoptions_new,"deletePtrIKoptionsmex","IKoptions");
  }
}
