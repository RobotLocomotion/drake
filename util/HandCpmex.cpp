#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

/*
 * A C version of the HandCp function from the Featherstone library
 *
 * To set up the model, use HandCpmex(model[,grav_accn])
 * Then to evaluate the dynamics use
 *   [H,C] = HandCpmex(q,qd[,f_ext]);
 */

class Model {
public:
  int NB;
  int *jcode;
  int *parent;
  Matrix3d *Xtree;
  Matrix3d* I;
  Vector3d a_grav;
  
  Vector3d* S;
  Matrix3d* Xup;
  Vector3d* v;
  Vector3d* avp;
  Vector3d* fvp;
  Matrix3d* IC;
  MatrixXd H;
  MatrixXd C;

  Model(int n) {
    this->NB = n;
    this->jcode = new int[n];
    this->parent = new int[n];
    this->Xtree = new Matrix3d[n];
    this->I = new Matrix3d[n];
    this->a_grav << 0.0, 0.0, 0.0;
    
    this->S = new Vector3d[n];
    this->Xup = new Matrix3d[n];
    this->v = new Vector3d[n];
    this->avp = new Vector3d[n];
    this->fvp = new Vector3d[n];
    this->IC = new Matrix3d[n];

    this->H = MatrixXd::Zero(n,n);
    this->C.resize(n,1); // C gets over-written completely by the algorithm below.
  }
  
  ~Model() {
    delete[] this->jcode;
    delete[] this->parent;
    delete[] this->Xtree;
    delete[] this->I;
    
    delete[] S;
    delete[] Xup;
    delete[] v;
    delete[] avp;
    delete[] fvp;
    delete[] IC;
  }
};

void Xpln(double theta, Vector2d r, Matrix3d* X)
{
  double c = cos(theta);
  double s = sin(theta);

  *X << 1, 0, 0, 
        s*r(0)-c*r(1), c, s,
        c*r(0)+s*r(1), -s, c;
}

void jcalcp(int code, double q, Matrix3d* XJ, Vector3d* S)
{
  if (code == 1) {				  // revolute joint
    Xpln( q, Vector2d(0.0,0.0) , XJ);
    *S << 1.0,0.0,0.0;
  } else if (code == 2) {		// x-axis prismatic joint
    Xpln( 0.0, Vector2d(q,0.0), XJ );
    *S << 0.0,1.0,0.0;
  } else if (code == 3) {		// y-axis prismatic joint
    Xpln( 0.0, Vector2d(0.0,q) , XJ);
    *S << 0.0,0.0,1.0;
  } else {
    mexErrMsgIdAndTxt("Drake:HandCpmex:BadJointCode","unrecognised joint code");
  }
}

Matrix3d crmp( Vector3d v )
{
  Matrix3d vcross;
  vcross <<   0.0, 0.0, 0.0,
	    v(2), 0.0, -v(0),
	   -v(1),  v(0), 0.0;
  return vcross;
}

Matrix3d crfp( Vector3d v )
{
  return -crmp(v).transpose();
}


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:HandCpmex:NotEnoughInputs","Usage model_ptr = HandCpmex(model,[,grav_accn]), then [H,C] = HandCpmex(model_ptr,q,qd[,f_ext]), and finally HandCpmex(model_ptr) to free the memory.");
  }

  Model *model = NULL;
  
  if (mxIsStruct(prhs[0])) { // then it's HandCp(model[,grav_accn]);
    // set up the model    
    mxArray* pNB = mxGetField(prhs[0],0,"NB");
    if (!pNB) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.NB.  Are you passing in the correct structure?");
    model = new Model((int) mxGetScalar(pNB));

    mxArray* pparent = mxGetField(prhs[0],0,"parent");
    if (!pparent) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.parent.");
    double* ppparent = mxGetPr(pparent);
    
    mxArray* pjcode = mxGetField(prhs[0],0,"jcode");
    if (!pjcode) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.jcode.");
    double* ppjcode = mxGetPr(pjcode);
    
    mxArray* pXtree = mxGetField(prhs[0],0,"Xtree");
    if (!pXtree) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.Xtree.");
    
    mxArray* pI = mxGetField(prhs[0],0,"I");
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
    
    if (nrhs>1) {
      if (!mxIsDouble(prhs[1]) || (mxGetNumberOfElements(prhs[1])!=2))
        mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","grav_accn must be a 2x1 double");
      double* p = mxGetPr(prhs[1]);
      model->a_grav(1) = p[0];
      model->a_grav(2) = p[1];
    }
    
    if (nlhs>0) {  // return a pointer to the model
      mxClassID cid;
      if (sizeof(model)==4) cid = mxUINT32_CLASS;
      else if (sizeof(model)==8) cid = mxUINT64_CLASS;
      else mexErrMsgIdAndTxt("Drake:HandCpmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
        
      plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
      memcpy(mxGetData(plhs[0]),&model,sizeof(model));
    }
  } else if (nrhs<2) { // then it's HandCpmex(model_ptr)  : cleanup the model_ptr memory.
    // first get the model_ptr back from matlab
    if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
      mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","first argument should be the model_ptr");
    memcpy(&model,mxGetData(prhs[0]),sizeof(model));
    
    delete model;
  } else { // then it's HandCpmex(model_ptr,q,qd)   

    // first get the model_ptr back from matlab
    if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
      mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","first argument should be the model_ptr");
    memcpy(&model,mxGetData(prhs[0]),sizeof(model));
    
    double *q,*qd;
    if (mxGetNumberOfElements(prhs[1])!=model->NB || mxGetNumberOfElements(prhs[2])!=model->NB)
      mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","q and qd must be size %d x 1",model->NB);
    q = mxGetPr(prhs[1]);
    qd = mxGetPr(prhs[2]);
    if (nrhs>3) {
      mexErrMsgIdAndTxt("Drake:HandCpmex:ExternalForceNotImplementedYet","sorry, f_ext is not implemented yet (but it would be trivial)");
    }
    
    Vector3d vJ, fh;
    Matrix3d XJ;
    int i,j;

    for (i=0; i<model->NB; i++) {
      jcalcp(model->jcode[i],q[i],&XJ,&(model->S[i]));
      vJ = model->S[i] * qd[i];
      model->Xup[i] = XJ * model->Xtree[i];
      if (model->parent[i] < 0) {
        model->v[i] = vJ;
        model->avp[i] = model->Xup[i] * (-model->a_grav);
      } else {
        model->v[i] = model->Xup[i]*model->v[model->parent[i]] + vJ;
        model->avp[i] = model->Xup[i]*model->avp[model->parent[i]] + crmp(model->v[i])*vJ;
      }
      model->fvp[i] = model->I[i]*model->avp[i] + crfp(model->v[i])*model->I[i]*model->v[i];
      
      model->IC[i] = model->I[i];
    }      
    
    for (i=(model->NB-1); i>=0; i--) {
      model->C(i) = (model->S[i]).transpose() * model->fvp[i];
      if (model->parent[i] >= 0) {
        model->fvp[model->parent[i]] = model->fvp[model->parent[i]] + (model->Xup[i]).transpose()*model->fvp[i];
        model->IC[model->parent[i]] = model->IC[model->parent[i]] + (model->Xup[i]).transpose()*model->IC[i]*model->Xup[i];
      }
    }

    for (i=0; i<model->NB; i++) {
      fh = model->IC[i] * model->S[i];
      model->H(i,i) = (model->S[i]).transpose() * fh;
      j=i;
      while (model->parent[j] >= 0) {
        fh = (model->Xup[j]).transpose() * fh;
        j = model->parent[j];
        model->H(i,j) = (model->S[j]).transpose() * fh;
        model->H(j,i) = model->H(i,j);
      }
    }
    
    if (nlhs>0) {
      plhs[0] = mxCreateDoubleMatrix(model->NB,model->NB,mxREAL);
      memcpy(mxGetPr(plhs[0]),model->H.data(),sizeof(double)*model->NB*model->NB);
    }
    if (nlhs>1) {
      plhs[1] = mxCreateDoubleMatrix(model->NB,1,mxREAL);
      memcpy(mxGetPr(plhs[1]),model->C.data(),sizeof(double)*model->NB);
    }
  }
}
