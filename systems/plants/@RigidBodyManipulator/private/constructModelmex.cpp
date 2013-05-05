#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "../../RigidBodyManipulator.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<3) {
    mexErrMsgIdAndTxt("Drake:constructModelmex:NotEnoughInputs","Usage model_ptr = constructModelmex(featherstone,bodies,gravity)");
  }

  RigidBodyManipulator *model=NULL;

  const mxArray* featherstone = prhs[0]; //mxGetProperty(prhs[0],0,"featherstone");
  if (!featherstone) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs", "can't find field model.featherstone.  Are you passing in the correct structure?");

  const mxArray* pBodies = prhs[1]; //mxGetProperty(prhs[0],0,"body");
  if (!pBodies) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.body.  Are you passing in the correct structure?");
  int num_bodies = mxGetNumberOfElements(prhs[1]);
  
  // set up the model
  mxArray* pNB = mxGetField(featherstone,0,"NB");
  if (!pNB) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.NB.  Are you passing in the correct structure?");
  model = new RigidBodyManipulator((int) mxGetScalar(pNB), (int) mxGetScalar(pNB), num_bodies);
  
  mxArray* pparent = mxGetField(featherstone,0,"parent");
  if (!pparent) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.parent.");
  double* ppparent = mxGetPr(pparent);
  
  mxArray* ppitch = mxGetField(featherstone,0,"pitch");
  if (!ppitch) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.pitch.");
  double* pppitch = mxGetPr(ppitch);
  
  mxArray* pdofnum = mxGetField(featherstone,0,"dofnum");
  if (!pdofnum) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.dofnum.");
  double* ppdofnum = mxGetPr(pdofnum);

  mxArray* pXtree = mxGetField(featherstone,0,"Xtree");
  if (!pXtree) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.Xtree.");
  
  mxArray* pI = mxGetField(featherstone,0,"I");
  if (!pI) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't find field model.featherstone.I.");
  
  for (int i=0; i<model->NB; i++) {
    model->parent[i] = ((int) ppparent[i]) - 1;  // since it will be used as a C index
    model->pitch[i] = (int) pppitch[i];
    model->dofnum[i] = (int) ppdofnum[i] - 1; // zero-indexed

    mxArray* pXtreei = mxGetCell(pXtree,i);
    if (!pXtreei) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.featherstone.Xtree{%d}",i);
    
    // todo: check that the size is 6x6
    memcpy(model->Xtree[i].data(),mxGetPr(pXtreei),sizeof(double)*6*6);
    
    mxArray* pIi = mxGetCell(pI,i);
    if (!pIi) mexErrMsgIdAndTxt("Drake:constructModelmex:BadInputs","can't access model.featherstone.I{%d}",i);
    
    // todo: check that the size is 6x6
    memcpy(model->I[i].data(),mxGetPr(pIi),sizeof(double)*6*6);
  }

  char buf[100];
  for (int i=0; i<model->num_bodies; i++) {
    mxArray* plinkname = mxGetProperty(pBodies,i,"linkname");
    mxGetString(plinkname,buf,100);
    model->bodies[i].linkname.assign(buf,strlen(buf));

    mxArray* pjointname = mxGetProperty(pBodies,i,"jointname");
    mxGetString(pjointname,buf,100);
    model->bodies[i].jointname.assign(buf,strlen(buf));

    mxArray* pmass = mxGetProperty(pBodies,i,"mass");
    model->bodies[i].mass = (double) mxGetScalar(pmass);

    mxArray* pcom = mxGetProperty(pBodies,i,"com");
    if (!mxIsEmpty(pcom)) memcpy(model->bodies[i].com.data(),mxGetPr(pcom),sizeof(double)*3);

    mxArray* pbdofnum = mxGetProperty(pBodies,i,"dofnum");
    model->bodies[i].dofnum = (int) mxGetScalar(pbdofnum) - 1;  //zero-indexed
    
    mxArray* pfloating = mxGetProperty(pBodies,i,"floating");
    model->bodies[i].floating = (int) mxGetScalar(pfloating);

    mxArray* pbpitch = mxGetProperty(pBodies,i,"pitch");
    model->bodies[i].pitch = (int) mxGetScalar(pbpitch);

    {  // lookup parent index
      mxArray *in_args[2] = { mxGetProperty(pBodies,i,"parent"), const_cast<mxArray*>(pBodies) };
      if (mxIsEmpty(in_args[0])) {
        model->bodies[i].parent = -1; 
      } else {
        mxArray *out_args[2];
        mexCallMATLAB(2,out_args,2,in_args,"ismember");
        model->bodies[i].parent = (int) mxGetScalar(out_args[1]) - 1; // zero-indexed
      }
    }
    
    if (model->bodies[i].dofnum>=0) {
      mxArray* pjointlim = mxGetProperty(pBodies,i,"joint_limit_min");
      model->joint_limit_min[model->bodies[i].dofnum] = mxGetScalar(pjointlim);
      pjointlim = mxGetProperty(pBodies,i,"joint_limit_max");
      model->joint_limit_max[model->bodies[i].dofnum] = mxGetScalar(pjointlim);
    }    
    
    mxArray* pbTtreei = mxGetProperty(pBodies,i,"Ttree");
    // todo: check that the size is 4x4
    memcpy(model->bodies[i].Ttree.data(),mxGetPr(pbTtreei),sizeof(double)*4*4);
    
    mxArray* pbT_body_to_jointi = mxGetProperty(pBodies,i,"T_body_to_joint");
    memcpy(model->bodies[i].T_body_to_joint.data(),mxGetPr(pbT_body_to_jointi),sizeof(double)*4*4);
  }
  
  const mxArray* a_grav_array = prhs[2]; //mxGetProperty(prhs[0],0,"gravity");
  if (a_grav_array && mxGetNumberOfElements(a_grav_array)==3) {
    double* p = mxGetPr(a_grav_array);
    model->a_grav[3] = p[0];
    model->a_grav[4] = p[1];
    model->a_grav[5] = p[2];
  } else {
    mexErrMsgIdAndTxt("Drake:constructModelmex:BadGravity","Couldn't find a 3 element gravity vector in the object.");
  }

  model->compile();
  
  if (nlhs>0) {  // return a pointer to the model
    mxClassID cid;
    if (sizeof(model)==4) cid = mxUINT32_CLASS;
    else if (sizeof(model)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:constructModelmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
    
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&model,sizeof(model));
  }

}
