#include "mex.h"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "RigidBodyManipulator.h"

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
    mexErrMsgIdAndTxt("Drake:HandCmex:NotEnoughInputs","Usage [H,C] = HandCmex(model_ptr,q,qd[,f_ext]).");
  }

  RigidBodyManipulator *model=NULL;

  // first get the model_ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","the first argument should be the model_ptr");
  memcpy(&model,mxGetData(prhs[0]),sizeof(model));
  
  double *q,*qd;
  MatrixXd f_ext;
  if (mxGetNumberOfElements(prhs[1])!=model->num_dof || mxGetNumberOfElements(prhs[2])!=model->num_dof)
    mexErrMsgIdAndTxt("Drake:HandCmex:BadInputs","q and qd must be size %d x 1",model->num_dof);
  q = mxGetPr(prhs[1]);
  qd = mxGetPr(prhs[2]);
  if (nrhs>3) {
    if (!mxIsEmpty(prhs[3])) {
      f_ext.resize(6,model->NB);
      memcpy(f_ext.data(), mxGetPr(prhs[3]), sizeof(double)*6*model->NB);
    }
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
    if (f_ext.cols()>0)
      model->fvp[i] = model->fvp[i] - f_ext.col(i);
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
    plhs[0] = mxCreateDoubleMatrix(model->num_dof,model->num_dof,mxREAL);
    memcpy(mxGetPr(plhs[0]),model->H.data(),sizeof(double)*model->num_dof*model->num_dof);
  }
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(model->num_dof,1,mxREAL);
    memcpy(mxGetPr(plhs[1]),model->C.data(),sizeof(double)*model->num_dof);
  }
  if (nlhs>2) {
    plhs[2] = mxCreateDoubleMatrix(model->num_dof*model->num_dof,model->num_dof,mxREAL);
    memcpy(mxGetPr(plhs[2]),model->dH.data(),sizeof(double)*model->num_dof*model->num_dof*model->num_dof);
  }
  if (nlhs>3) {
    plhs[3] = mxCreateDoubleMatrix(model->num_dof,model->num_dof,mxREAL);
    memcpy(mxGetPr(plhs[3]),model->dC.data(),sizeof(double)*model->num_dof*model->num_dof);
  }
}
