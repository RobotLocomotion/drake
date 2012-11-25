#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the HandCp function from the Featherstone library
 *
 * To set up the model, use HandCpmex(model[,grav_accn])
 * Then to evaluate the dynamics use
 *   [H,C] = HandCpmex(q,qd[,f_ext]);
 */



void Xpln(double theta, Vector2d r, Matrix3d* X)
{
  double c = cos(theta);
  double s = sin(theta);

  *X << 1, 0, 0, 
        s*r(0)-c*r(1), c, s,
        c*r(0)+s*r(1), -s, c;
}

/* dXpln  coordinate transform derivative wrt theta for planar vectors.
 * dXpln(theta,r) calculates the derivative of the coordinate transform 
 * matrix from A to B coordinates for planar motion vectors, where 
 * coordinate frame B is located at point r (2D vector expressed in A 
 * coords) relative to frame A, and is rotated by an angle theta (radians) 
 * relative to A.*/
void dXpln(double theta, Vector2d r, int varIndex, Matrix3d* dX)
{
  double c = cos(theta);
  double s = sin(theta);
  if (varIndex == 1) {
    *dX << 0, 0, 0,
           c*r(1)+s*r(2), -s, c,
           -s*r(1)+c*r(2), -c, -s;
  }
  else if (varIndex == 2) {
    *dX << 0, 0, 0,
           s, 0, 0,
           c, 0, 0;
  }
  else if (varIndex == 3) {
    *dX << 0, 0, 0,
           -c, 0, 0,
            s, 0, 0;
  }
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

/* 
 *  jcalcp  Calculate derivative of joint transform
 *  [dXJ]=djcalcp(code,q) calculates the joint transform derivative
 *  matrix for revolute (code==1), x-axis prismatic (code==2) and y-axis
 *  prismatic (code==3) joints.
 */
void djcalcp(int code, double q, Matrix3d* dXJ) 
{
  if (code == 1) {				  // revolute joint
    dXpln( q, Vector2d(0.0, 0.0), 1, dXJ);
  } else if (code == 2) {		// x-axis prismatic joint
    dXpln( 0.0, Vector2d(q, 0.0), 2,  dXJ );
  } else if (code == 3) {		// y-axis prismatic joint
    dXpln( 0.0, Vector2d(0.0, q), 3, dXJ);
  } else {
    mexErrMsgIdAndTxt("Drake:HandCpmex:BadJointCode", "unrecognised joint code");
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

void dcrmp(Vector3d v, Vector3d x, MatrixXd dv, MatrixXd dx, MatrixXd* dvcross) {
 int n = dv.cols();
 
 (*dvcross).resize(3,n);
 (*dvcross).row(0) = MatrixXd::Zero(1,n);
 (*dvcross).row(1) = dv.row(2)*x(0) + v(2)*dx.row(0) - dv.row(0)*x(2) - v(0)*dx.row(2);
 (*dvcross).row(2) = -dv.row(1)*x(0) - v(1)*dx.row(0) + dv.row(0)*x(1) + v(0)*dx.row(1);
}

void dcrfp(Vector3d v, Vector3d x, MatrixXd dv, MatrixXd dx, MatrixXd* dvcross) {
 (*dvcross).resize(3,dv.cols());
 (*dvcross).row(0) = -dv.row(2)*x(1) - v(2)*dx.row(1) + dv.row(1)*x(2) + v(1)*dx.row(2);
 (*dvcross).row(1) = -dv.row(0)*x(2) - v(0)*dx.row(2);
 (*dvcross).row(2) = dv.row(0)*x(1) + v(0)*dx.row(1);
}


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[] )
{
  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:HandCpmex:NotEnoughInputs","Usage model_ptr = HandCpmex(model,[,grav_accn]), then [H,C] = HandCpmex(model_ptr,q,qd[,f_ext]), and finally HandCpmex(model_ptr) to free the memory.");
  }

  Model *model = NULL;
  
  if (mxIsStruct(prhs[0])) { // then it's HandCp(model[,grav_accn]);
    mxArray* featherstone = mxGetField(prhs[0],0,"featherstone");
    if (!featherstone) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs", "can't find field model.featherstone.  Are you passing in the correct structure?");

    // set up the model    
    mxArray* pNB = mxGetField(featherstone,0,"NB");
    if (!pNB) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't find field model.NB.  Are you passing in the correct structure?");
    model = new Model((int) mxGetScalar(pNB));

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
    
    mxArray* pBodies = mxGetField(prhs[0],0,"body");

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
    
    Vector3d vJ, fh, dfh, dvJdqd;
    Matrix3d XJ, dXJdq;
    int i,j,k;

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
      
      //Calculate gradient information if it is requested
      if (nlhs > 2) {
        djcalcp(model->jcode[i], q[i], &dXJdq);
        model->dXupdq[i] = dXJdq*model->Xtree[i];
        
        for (j=0; j<model->NB; j++) {
         model->dIC[i][j] = Matrix3d::Zero();
        }
      }
      
      if (nlhs > 3) {
        dvJdqd = model->S[i];
        if (model->parent[i] < 0) {
          model->dvdqd[i].col(i) = dvJdqd;
          model->davpdq[i].col(i) = model->dXupdq[i] * -model->a_grav;

        } else {
          j = model->parent[i];
          model->dvdq[i] = model->Xup[i]*model->dvdq[j];
          model->dvdq[i].col(i) += model->dXupdq[i]*model->v[j];
          model->dvdqd[i] = model->Xup[i]*model->dvdqd[j];
          model->dvdqd[i].col(i) += dvJdqd;
          
          model->davpdq[i] = model->Xup[i]*model->davpdq[j];
          model->davpdq[i].col(i) += model->dXupdq[i]*model->avp[j];
          for (k=0; k < model->NB; k++) {
            dcrmp(model->v[i],vJ,model->dvdq[i].col(k),Vector3d::Zero(),&(model->dcross));
            model->davpdq[i].col(k) += model->dcross;            
          }
          
          model->dvJdqd_mat = MatrixXd::Zero(3,model->NB);
          model->dvJdqd_mat.col(i) = dvJdqd;
          dcrmp(model->v[i],vJ,model->dvdqd[i],model->dvJdqd_mat,&(model->dcross));
          model->davpdqd[i] = model->Xup[i]*model->davpdqd[j] + model->dcross;
        }
        
        dcrfp(model->v[i],model->I[i]*model->v[i],model->dvdq[i],model->I[i]*model->dvdq[i],&(model->dcross));
        model->dfvpdq[i] = model->I[i]*model->davpdq[i] + model->dcross;
        dcrfp(model->v[i],model->I[i]*model->v[i],model->dvdqd[i],model->I[i]*model->dvdqd[i],&(model->dcross));
        model->dfvpdqd[i] = model->I[i]*model->davpdqd[i] + model->dcross;

      }      
    }    
    
    for (i=(model->NB-1); i>=0; i--) {
      model->C(i) = (model->S[i]).transpose() * model->fvp[i];
      
      if (nlhs > 3) {
       model->dC.block(i,0,1,model->NB) = model->S[i].transpose()*model->dfvpdq[i];
       model->dC.block(i,model->NB,1,model->NB) = model->S[i].transpose()*model->dfvpdqd[i];
      }
      
      if (model->parent[i] >= 0) {
        model->fvp[model->parent[i]] = model->fvp[model->parent[i]] + (model->Xup[i]).transpose()*model->fvp[i];
        model->IC[model->parent[i]] = model->IC[model->parent[i]] + (model->Xup[i]).transpose()*model->IC[i]*model->Xup[i];
        
        if (nlhs > 2) {
          for (k=0; k < model->NB; k++) {
            model->dIC[model->parent[i]][k] += model->Xup[i].transpose()*model->dIC[i][k]*model->Xup[i];
          }
          model->dIC[model->parent[i]][i] += model->dXupdq[i].transpose()*model->IC[i]*model->Xup[i] + model->Xup[i].transpose()*model->IC[i]*model->dXupdq[i];
        }
        
        if (nlhs > 3) {
          model->dfvpdq[model->parent[i]] += model->Xup[i].transpose()*model->dfvpdq[i];
          model->dfvpdq[model->parent[i]].col(i) += model->dXupdq[i].transpose()*model->fvp[i];
          model->dfvpdqd[model->parent[i]] += model->Xup[i].transpose()*model->dfvpdqd[i];
        }
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
    
    if (nlhs > 2) {
      for (k=0; k < model->NB; k++) {
        for (i=0; i < model->NB; i++) {
          fh = model->IC[i] * model->S[i];
          dfh = model->dIC[i][k] * model->S[i];  //dfh/dqk          
          model->dH(i + i*model->NB,k) = model->S[i].transpose() * dfh;
          j = i;
          while (model->parent[j] >= 0) {
            if (j==k) {
              dfh = model->Xup[j].transpose() * dfh + model->dXupdq[k].transpose() * fh;
            } else {
              dfh = model->Xup[j].transpose() * dfh;
            }
            fh = (model->Xup[j]).transpose() * fh;
            
            j = model->parent[j];
            model->dH(i + j*model->NB,k) = model->S[j].transpose() * dfh;
            model->dH(j + i*model->NB,k) = model->dH(i + j*model->NB,k);
          }
        }
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
    if (nlhs>2) {
      plhs[2] = mxCreateDoubleMatrix(model->NB*model->NB,model->NB,mxREAL);
      memcpy(mxGetPr(plhs[2]),model->dH.data(),sizeof(double)*model->NB*model->NB*model->NB);
    }
    if (nlhs>3) {
      plhs[3] = mxCreateDoubleMatrix(model->NB,2*model->NB,mxREAL);
      memcpy(mxGetPr(plhs[3]),model->dC.data(),sizeof(double)*2*model->NB*model->NB);
    }
  }
}
