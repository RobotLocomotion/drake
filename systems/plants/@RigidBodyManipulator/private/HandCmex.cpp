#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "Model.h"

#define INF -2147483648

using namespace Eigen;
using namespace std;

/*
 * A C version of the HandC function from the Featherstone library
 *
 * To set up the model, use HandCpmex(model[,grav_accn])
 * Then to evaluate the dynamics use
 *   [H,C] = HandCmex(q,qd[,f_ext]);
 */


void Xrotz(double theta, MatrixXd* X) {
	double c = cos(theta);
	double s = sin(theta);

	(*X).resize(6,6);
	*X <<  c, s, 0, 0, 0, 0,
		  -s, c, 0, 0, 0, 0,
		   0, 0, 1, 0, 0, 0,
		   0, 0, 0, c, s, 0,
		   0, 0, 0,-s, c, 0,
		   0, 0, 0, 0, 0, 1;
}

void dXrotz(double theta, MatrixXd* dX) {
	double dc = -sin(theta); 
	double ds = cos(theta); 

	(*dX).resize(6,6);
	*dX << dc, ds, 0, 0, 0, 0,
      	  -ds, dc, 0, 0, 0, 0,
       		0, 0, 0, 0, 0, 0,
       		0, 0, 0, dc, ds, 0,
       		0, 0, 0,-ds, dc, 0,
       		0, 0, 0, 0, 0, 0;
}

void Xtrans(Vector3d r, MatrixXd* X) {
	(*X).resize(6,6);
	*X <<  	1, 0, 0, 0, 0, 0,
		   	0, 1, 0, 0, 0, 0,
		   	0, 0, 1, 0, 0, 0,
           	0, r[2],-r[1], 1, 0, 0,
		   -r[2], 0, r[0], 0, 1, 0,
          	r[1],-r[0], 0, 0, 0, 1;
}

void dXtrans(MatrixXd* dX) {
 	(*dX).resize(36,3);
	(*dX)(4,2) = -1; 
	(*dX)(5,1) = 1; 
	(*dX)(9,2) = 1; 
	(*dX)(11,0) = -1; 
	(*dX)(15,1) = -1; 
	(*dX)(16,0) = 1;
}

void dXtransPitch(MatrixXd* dX) {
	(*dX).resize(6,6);
	*dX << 0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0,
   		   0, 1, 0, 0, 0, 0,
       	  -1, 0, 0, 0, 0, 0,
           0, 0, 0, 0, 0, 0;
}


void jcalc(int pitch, double q, MatrixXd* Xj, VectorXd* S) {
	(*Xj).resize(6,6);
	(*S).resize(6);

	if (pitch == 0) { // revolute joint
	  	Xrotz(q,Xj);
	    *S << 0,0,1,0,0,0;
	}	
	else if (pitch == INF) { // prismatic joint
  		Xtrans(Vector3d(0.0,0.0,q),Xj);
  		*S << 0,0,0,0,0,1;
	}
	else { // helical joint
		MatrixXd A(6,6);
		MatrixXd B(6,6);  		
		Xrotz(q,&A);
	    Xtrans(Vector3d(0.0,0.0,q*pitch),&B);
		*Xj = A*B;		
  		*S << 0,0,1,0,0,pitch;
	}
}

void djcalc(int pitch, double q, MatrixXd* dXj) {
	(*dXj).resize(6,6);

	if (pitch == 0) { // revolute joint
  		dXrotz(q,dXj);
	}
	else if (pitch == INF) { // prismatic joint
  		dXtransPitch(dXj); 
    }
	else { // helical joint
	    MatrixXd X(6,6),Xj(6,6),dXrz(6,6),dXjp(6,6);
	    Xrotz(q,&Xj);
	    dXrotz(q,&dXrz);
	    Xtrans(Vector3d(0.0,0.0,q*pitch),&X);
	    dXtransPitch(&dXjp);
		*dXj = Xj * dXjp * pitch + dXrz * X;
	}
}

MatrixXd crm(VectorXd v)
{
  	MatrixXd vcross(6,6);
  	vcross << 0, -v[2], v[1], 0, 0, 0,
	    	v[2], 0,-v[0], 0, 0, 0,
	   	   -v[1], v[0], 0, 0, 0, 0,
	    	0, -v[5], v[4], 0, -v[2], v[1],
	    	v[5], 0,-v[3], v[2], 0, -v[0],
	   	   -v[4], v[3], 0,-v[1], v[0], 0;
  	return vcross;
}

MatrixXd crf(VectorXd v)
{
 	MatrixXd vcross(6,6);
 	vcross << 0,-v[2], v[1], 0,-v[5], v[4],
	    	v[2], 0,-v[0], v[5], 0,-v[3],
	   	   -v[1], v[0], 0,-v[4], v[3], 0,
	    	0, 0, 0, 0,-v[2], v[1],
	    	0, 0, 0, v[2], 0,-v[0],
	    	0, 0, 0,-v[1], v[0], 0;
 	return vcross;
}

void dcrm(VectorXd v, VectorXd x, MatrixXd dv, MatrixXd dx, MatrixXd* dvcross) {
 	(*dvcross).resize(6,dv.cols());
 	(*dvcross).row(0) = -dv.row(2)*x[1] + dv.row(1)*x[2] - v[2]*dx.row(1) + v[1]*dx.row(2); 
	(*dvcross).row(1) =  dv.row(2)*x[0] - dv.row(0)*x[2] + v[2]*dx.row(0) - v[0]*dx.row(2);
  	(*dvcross).row(2) = -dv.row(1)*x[0] + dv.row(0)*x[1] - v[1]*dx.row(0) + v[0]*dx.row(1);
  	(*dvcross).row(3) = -dv.row(5)*x[1] + dv.row(4)*x[2] - dv.row(2)*x[4] + dv.row(1)*x[5] - v[5]*dx.row(1) + v[4]*dx.row(2) - v[2]*dx.row(4) + v[1]*dx.row(5);
  	(*dvcross).row(4) =  dv.row(5)*x[0] - dv.row(3)*x[2] + dv.row(2)*x[3] - dv.row(0)*x[5] + v[5]*dx.row(0) - v[3]*dx.row(2) + v[2]*dx.row(3) - v[0]*dx.row(5);
  	(*dvcross).row(5) = -dv.row(4)*x[0] + dv.row(3)*x[1] - dv.row(1)*x[3] + dv.row(0)*x[4] - v[4]*dx.row(0) + v[3]*dx.row(1) - v[1]*dx.row(3) + v[0]*dx.row(4);
}

void dcrf(VectorXd v, VectorXd x, MatrixXd dv, MatrixXd dx, MatrixXd* dvcross) {
 	(*dvcross).resize(6,dv.cols());
 	(*dvcross).row(0) =  dv.row(2)*x[1] - dv.row(1)*x[2] + dv.row(5)*x[4] - dv.row(4)*x[5] + v[2]*dx.row(1) - v[1]*dx.row(2) + v[5]*dx.row(4) - v[4]*dx.row(5);
  	(*dvcross).row(1) = -dv.row(2)*x[0] + dv.row(0)*x[2] - dv.row(5)*x[3] + dv.row(3)*x[5] - v[2]*dx.row(0) + v[0]*dx.row(2) - v[5]*dx.row(3) + v[3]*dx.row(5);
 	(*dvcross).row(2) =  dv.row(1)*x[0] - dv.row(0)*x[1] + dv.row(4)*x[3] - dv.row(3)*x[4] + v[1]*dx.row(0) - v[0]*dx.row(1) + v[4]*dx.row(3) - v[3]*dx.row(4);
  	(*dvcross).row(3) =  dv.row(2)*x[4] - dv.row(1)*x[5] + v[2]*dx.row(4) - v[1]*dx.row(5);
  	(*dvcross).row(4) = -dv.row(2)*x[3] + dv.row(0)*x[5] - v[2]*dx.row(3) + v[0]*dx.row(5);
  	(*dvcross).row(5) =  dv.row(1)*x[3] - dv.row(0)*x[4] + v[1]*dx.row(3) - v[0]*dx.row(4);
	*dvcross = -(*dvcross);
}





void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {

  if (nrhs<1) {
    mexErrMsgIdAndTxt("Drake:HandCmex:NotEnoughInputs","Usage model_ptr = HandCmex(model), then [H,C] = HandCmex(model,q,qd[,f_ext]), and finally HandCmex(model,model_ptr) to free the memory.");
  }

  Model *model=NULL;

  if (nrhs==1) { // then it's HandC(model);
    mxArray* featherstone = mxGetProperty(prhs[0],0,"featherstone");
    if (!featherstone) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs", "can't find field model.featherstone.  Are you passing in the correct structure?");

    // set up the model    
    mxArray* pNB = mxGetField(featherstone,0,"NB");
    if (!pNB) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.featherstone.NB.  Are you passing in the correct structure?");
    model = new Model((int) mxGetScalar(pNB));

    mxArray* pparent = mxGetField(featherstone,0,"parent");
    if (!pparent) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.featherstone.parent.");
    double* ppparent = mxGetPr(pparent);

    mxArray* ppitch = mxGetField(featherstone,0,"pitch");
    if (!ppitch) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.featherstone.pitch.");
    double* pppitch = mxGetPr(ppitch);

    mxArray* pXtree = mxGetField(featherstone,0,"Xtree");
    if (!pXtree) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.featherstone.Xtree.");

    mxArray* pI = mxGetField(featherstone,0,"I");
    if (!pI) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.featherstone.I.");

    for (int i=0; i<model->NB; i++) {
      model->parent[i] = ((int) ppparent[i]) - 1;  // since it will be used as a C index
      model->pitch[i] = (int) pppitch[i];

      mxArray* pXtreei = mxGetCell(pXtree,i);
      if (!pXtreei) mexErrMsgIdAndTxt("Drake:HandCpmex:BadInputs","can't access model.featherstone.Xtree{%d}",i);

      // todo: check that the size is 6x6
      memcpy(model->Xtree[i].data(),mxGetPr(pXtreei),sizeof(double)*6*6);

      mxArray* pIi = mxGetCell(pI,i);
      if (!pIi) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't access model.featherstone.I{%d}",i);

      // todo: check that the size is 6x6
      memcpy(model->I[i].data(),mxGetPr(pIi),sizeof(double)*6*6);
    }

    mxArray* pBodies = mxGetProperty(prhs[0],0,"body");
    if (!pBodies) mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","can't find field model.body.  Are you passing in the correct structure?");


    for (int i=0; i<model->NB + 1; i++) {
      mxArray* pbpitch = mxGetProperty(pBodies,i,"pitch");
      model->bodies[i].pitch = (int) mxGetScalar(pbpitch);
      
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
      // todo: check that the size is 4x4
      memcpy(model->bodies[i].Ttree.data(),mxGetPr(pbTtreei),sizeof(double)*4*4);      

      mxArray* pbT_body_to_jointi = mxGetProperty(pBodies,i,"T_body_to_joint");
      memcpy(model->bodies[i].T_body_to_joint.data(),mxGetPr(pbT_body_to_jointi),sizeof(double)*4*4);      

//      mxArray* pbTi = mxGetProperty(pBodies,i,"T");
//      // todo: check that the size is 4x4
//      memcpy(model->bodies[i].T.data(),mxGetPr(pbTi),sizeof(double)*4*4);      

    }

    mxArray* a_grav_array = mxGetProperty(prhs[0],0,"gravity");
    if (a_grav_array && mxGetNumberOfElements(a_grav_array)==3) {
      double* p = mxGetPr(a_grav_array);
      model->a_grav[3] = p[0];
      model->a_grav[4] = p[1];
      model->a_grav[5] = p[2];
    } else {
      mexErrMsgIdAndTxt("Drake:HandCmex:BadGravity","Couldn't find a 3 element gravity vector in the object.");
    }

    if (nlhs>0) {  // return a pointer to the model
      mxClassID cid;
      if (sizeof(model)==4) cid = mxUINT32_CLASS;
      else if (sizeof(model)==8) cid = mxUINT64_CLASS;
      else mexErrMsgIdAndTxt("Drake:HandCmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
        
      plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
      memcpy(mxGetData(plhs[0]),&model,sizeof(model));
    }
  } else if (nrhs==2) { // then it's HandC(model,model_ptr);
    if (!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
        mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","the second argument should be the model_ptr");
    memcpy(&model,mxGetData(prhs[1]),sizeof(model));
    
    delete model;
  } else { // then it's HandCmex(model_ptr,q,qd)   

    // first get the model_ptr back from matlab
    mxArray* mex_model_ptr = mxGetProperty(prhs[0],0,"mex_model_ptr");
    if (!mex_model_ptr)  mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs","first argument should be the model class object");
    memcpy(&model,mxGetData(mex_model_ptr),sizeof(model));
    
    double *q,*qd;
    if (mxGetNumberOfElements(prhs[1])!=model->NB || mxGetNumberOfElements(prhs[2])!=model->NB)
      mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","q and qd must be size %d x 1",model->NB);
    q = mxGetPr(prhs[1]);
    qd = mxGetPr(prhs[2]);
    if (nrhs>3) {
      mexErrMsgIdAndTxt("Drake:HandCmex:ExternalForceNotImplementedYet","sorry, f_ext is not implemented yet (but it would be trivial)");
    }
    
    VectorXd vJ(6), fh(6), dfh(6), dvJdqd(6);
    MatrixXd XJ(6,6), dXJdq(6,6);
    int i,j,k;

    for (i=0; i<model->NB; i++) {
      jcalc(model->pitch[i],q[i],&XJ,&(model->S[i]));
      vJ = model->S[i] * qd[i];
      model->Xup[i] = XJ * model->Xtree[i];

      if (model->parent[i] < 0) {
        model->v[i] = vJ;
        model->avp[i] = model->Xup[i] * (-model->a_grav);
      } else {
        model->v[i] = model->Xup[i]*model->v[model->parent[i]] + vJ;
        model->avp[i] = model->Xup[i]*model->avp[model->parent[i]] + crm(model->v[i])*vJ;
      }
      model->fvp[i] = model->I[i]*model->avp[i] + crf(model->v[i])*model->I[i]*model->v[i];
      model->IC[i] = model->I[i];
      
      //Calculate gradient information if it is requested
      if (nlhs > 2) {
        djcalc(model->pitch[i], q[i], &dXJdq);
        model->dXupdq[i] = dXJdq * model->Xtree[i];
   
        for (j=0; j<model->NB; j++) {
         model->dIC[i][j] = MatrixXd::Zero(6,6);
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
            dcrm(model->v[i],vJ,model->dvdq[i].col(k),VectorXd::Zero(6),&(model->dcross));
            model->davpdq[i].col(k) += model->dcross;            
          }
          
          model->dvJdqd_mat = MatrixXd::Zero(6,model->NB);
          model->dvJdqd_mat.col(i) = dvJdqd;
          dcrm(model->v[i],vJ,model->dvdqd[i],model->dvJdqd_mat,&(model->dcross));
          model->davpdqd[i] = model->Xup[i]*model->davpdqd[j] + model->dcross;
        }
        
        dcrf(model->v[i],model->I[i]*model->v[i],model->dvdq[i],model->I[i]*model->dvdq[i],&(model->dcross));
        model->dfvpdq[i] = model->I[i]*model->davpdq[i] + model->dcross;
        dcrf(model->v[i],model->I[i]*model->v[i],model->dvdqd[i],model->I[i]*model->dvdqd[i],&(model->dcross));
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
