#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "../../PlanarModel.h"

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:constructModelpmex:NotEnoughInputs","Usage model_ptr = constructModelpmex(model)");
  }

  PlanarModel *model = NULL;
  
  mxArray* featherstone = mxGetProperty(prhs[0],0,"featherstone");
  if (!featherstone) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs", "can't find field model.featherstone.  Are you passing in the correct structure?");
  
  // set up the model
  mxArray* pNB = mxGetField(featherstone,0,"NB");
  if (!pNB) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.NB.  Are you passing in the correct structure?");
  model = new PlanarModel((int) mxGetScalar(pNB));
  
  mxArray* pparent = mxGetField(featherstone,0,"parent");
  if (!pparent) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.parent.");
  double* ppparent = mxGetPr(pparent);
  
  mxArray* pjcode = mxGetField(featherstone,0,"jcode");
  if (!pjcode) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.jcode.");
  double* ppjcode = mxGetPr(pjcode);
  
  mxArray* pXtree = mxGetField(featherstone,0,"Xtree");
  if (!pXtree) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.Xtree.");
  
  mxArray* pI = mxGetField(featherstone,0,"I");
  if (!pI) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.I.");
  
  for (int i=0; i<model->NB; i++) {
    model->parent[i] = ((int) ppparent[i]) - 1;  // since it will be used as a C index
    model->jcode[i] = (int) ppjcode[i];
    
    mxArray* pXtreei = mxGetCell(pXtree,i);
    if (!pXtreei) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.Xtree{%d}",i);
    // todo: check that the size is 3x3
    memcpy(model->Xtree[i].data(),mxGetPr(pXtreei),sizeof(double)*3*3);
    
    mxArray* pIi = mxGetCell(pI,i);
    if (!pIi) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.I{%d}",i);
    // todo: check that the size is 3x3
    memcpy(model->I[i].data(),mxGetPr(pIi),sizeof(double)*3*3);
  }
  
  mxArray* pBodies = mxGetProperty(prhs[0],0,"body");
  
  if (!pBodies) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.body.  Are you passing in the correct structure?");
  for (int i=0; i<model->NB + 1; i++) {
    mxArray* pbjcode = mxGetProperty(pBodies,i,"jcode");
    model->bodies[i].jcode = (int) mxGetScalar(pbjcode);
    
    mxArray* pbjsign = mxGetProperty(pBodies,i,"jsign");
    model->bodies[i].jsign = (int) mxGetScalar(pbjsign);
    
    mxArray* pbdofnum = mxGetProperty(pBodies,i,"dofnum");
    model->bodies[i].dofnum = (int) mxGetScalar(pbdofnum) - 1;  //zero-indexed
    
    //Todo--do something safer here!
    //Want to undo the -1 above wrt parents.  This assumes the bodies are ordered identically, which they should be...
    //but parent_ind should probably just be a field in body
    if (i==0) {
      model->bodies[i].parent = -1;
    } else {
      model->bodies[i].parent = model->parent[i-1] + 1;
    }
//       mxArray* pbparent = mxGetProperty(pBodies, i, "parent");
//       model->bodies[i].parent = (int) mxGetScalar(pbparent) - 1;  //zero-indexed
    
    mxArray* pbTtreei = mxGetProperty(pBodies,i,"Ttree");
    // todo: check that the size is 3x3
    memcpy(model->bodies[i].Ttree.data(),mxGetPr(pbTtreei),sizeof(double)*3*3);
  }
  
  mxArray* a_grav_array = mxGetProperty(prhs[0],0,"gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array)==2) {
    double* p = mxGetPr(a_grav_array);
    model->a_grav(1) = p[0];
    model->a_grav(2) = p[1];
  } else {
    mexErrMsgIdAndTxt("Drake:HandCmex:BadGravity","Couldn't find a 2 element gravity vector in the object.");
  }
  
  if (nlhs>0) {  // return a pointer to the model
    mxClassID cid;
    if (sizeof(model)==4) cid = mxUINT32_CLASS;
    else if (sizeof(model)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:HandCpmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&model,sizeof(model));
  }
}
