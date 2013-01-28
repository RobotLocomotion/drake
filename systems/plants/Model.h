#include <Eigen/Dense>
#include "RigidBody.h"
using namespace Eigen;
using namespace std;

class Model 
{
public:
  int NB;
  int *pitch;
  int *parent;
  MatrixXd* Xtree;
  MatrixXd* I;
  VectorXd a_grav;
  
  VectorXd* S;
  MatrixXd* Xup;
  VectorXd* v;
  VectorXd* avp;
  VectorXd* fvp;
  MatrixXd* IC;
  MatrixXd H;
  MatrixXd C;
  
  //Variables for gradient calculations
  MatrixXd* dXupdq;
  MatrixXd** dIC;
  MatrixXd dH;
  MatrixXd dC;
  
  MatrixXd* dvdq;
  MatrixXd* dvdqd;
  MatrixXd* davpdq;
  MatrixXd* davpdqd;
  MatrixXd* dfvpdq;
  MatrixXd* dfvpdqd;
  MatrixXd dvJdqd_mat;
  MatrixXd dcross;
  
  RigidBody* bodies;
  bool kinematicsInit;
  double *cached_q;
  int secondDerivativesCached;

  Model(int n) {
    NB = n;
    pitch = new int[n];
    parent = new int[n];
    Xtree = new MatrixXd[n];
    I = new MatrixXd[n];
    a_grav = VectorXd::Zero(6);
    
    S = new VectorXd[n];
    Xup = new MatrixXd[n];
    v = new VectorXd[n];
    avp = new VectorXd[n];
    fvp = new VectorXd[n];
    IC = new MatrixXd[n];
    
        
    for(int i=0; i < n; i++) {
      Xtree[i] = MatrixXd::Zero(6,6);
      I[i] = MatrixXd::Zero(6,6);
      S[i] = VectorXd::Zero(6);
      Xup[i] = MatrixXd::Zero(6,6);
      v[i] = VectorXd::Zero(6);
      avp[i] = VectorXd::Zero(6);
      fvp[i] = VectorXd::Zero(6);
      IC[i] = MatrixXd::Zero(6,6);
    }
    
    H = MatrixXd::Zero(n,n);
    C.resize(n,1); // C gets over-written completely by the algorithm below.
    
    //Variable allocation for gradient calculations
    dXupdq = new MatrixXd[n];
    dIC = new MatrixXd*[n];
    for(int i=0; i < n; i++) {
      dIC[i] = new MatrixXd[n];
	    for(int j=0; j < n; j++) {
        dIC[i][j] = MatrixXd::Zero(6,6);
      }
    }
    dH = MatrixXd::Zero(n*n,n);
    dvJdqd_mat = MatrixXd::Zero(6,n);
//     dcross.resize(6,n);
    dC = MatrixXd::Zero(n,3*n);
    
    dvdq = new MatrixXd[n];
    dvdqd = new MatrixXd[n];
    davpdq = new MatrixXd[n];
    davpdqd = new MatrixXd[n];
    dfvpdq = new MatrixXd[n];
    dfvpdqd = new MatrixXd[n];
    
    for(int i=0; i < n; i++) {
      dvdq[i] = MatrixXd::Zero(6,n);
      dvdqd[i] = MatrixXd::Zero(6,n);
      davpdq[i] = MatrixXd::Zero(6,n);
      davpdqd[i] = MatrixXd::Zero(6,n);
      dfvpdq[i] = MatrixXd::Zero(6,n);
      dfvpdqd[i] = MatrixXd::Zero(6,n);
    }
    //This assumes that there is only one "world" object
    bodies = new RigidBody[n+1];
    
    for(int i=0; i < n+1; i++) {
      bodies[i].setN(n);
    }
    
    kinematicsInit = false;
    cached_q = new double[n];
    secondDerivativesCached = 0;
  }
  
  ~Model() {
    delete[] pitch;
    delete[] parent;
    delete[] Xtree;
    delete[] I;
    
    delete[] S;
    delete[] Xup;
    delete[] v;
    delete[] avp;
    delete[] fvp;
    delete[] IC;
    
    delete[] dXupdq;
    for (int i=0; i<NB; i++) {
      delete[] dIC[i];
    }
    delete[] dIC;
    delete[] dvdq;
    delete[] dvdqd;
    delete[] davpdq;
    delete[] davpdqd;
    delete[] dfvpdq;
    delete[] dfvpdqd;
    
    delete[] bodies;
    delete[] cached_q;
  }

  Vector3d getCOM(void)
  {
    double m = 0.0;
    double bm;
    Vector3d com = Vector3d::Zero();
    Vector3d bc;

    for (int i=0; i<=NB; i++) {
      bm = bodies[i].mass;
      if (bm>0) {
	bc = forwardKin(i,bodies[i].com,false);
	com = (m*com + bm*bc)/(m+bm);
	m = m+bm;
      }
    }
    return com;
  }

  MatrixXd getCOMJac(void)
  {
    double m = 0.0;
    double bm;
    MatrixXd J = MatrixXd::Zero(3,NB);
    MatrixXd bJ;

    for (int i=0; i<=NB; i++) {
      bm = bodies[i].mass;
      if (bm>0) {
	bJ = forwardJac(i,bodies[i].com,false);
	J = (m*J + bm*bJ)/(m+bm);
	m = m+bm;
      }
    }
    return J;
  }
  
  MatrixXd getCOMJacDot(void)
  {
    double m = 0.0;
    double bm;
    MatrixXd dJ = MatrixXd::Zero(3,NB*NB);
    MatrixXd bdJ;

    for (int i=0; i<=NB; i++) {
      bm = bodies[i].mass;
      if (bm>0) {
	bdJ = forwardJacDot(i,bodies[i].com,false);
	dJ = (m*dJ + bm*bdJ)/(m+bm);
	m = m+bm;
      }
    }
    return dJ;
  }

  MatrixXd forwardKin(const int body_ind, const MatrixXd pts, const bool include_rotations)
  {
    int dim=3, n_pts = pts.cols();
    MatrixXd T = bodies[body_ind].T.topLeftCorner(dim,dim+1);
    if (include_rotations) {
      Vector3d rpy;
      rpy << atan2(T(2,1),T(2,2)), atan2(-T(2,0),sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2))), atan2(T(1,0),T(0,0));
      // NOTE: we're assuming an X-Y-Z convention was used to construct T
      
      MatrixXd X = MatrixXd::Zero(2*dim,n_pts);
      X.block(0,0,3,n_pts) = T*pts;
      X.block(3,0,3,n_pts) = rpy.replicate(1,n_pts);
      return X;
    } else {
      return T*pts;
    }
  }

  MatrixXd forwardJac(const int body_ind, const MatrixXd pts, const bool include_rotations)
  {
    int dim = 3, n_pts = pts.cols();
    MatrixXd tmp = bodies[body_ind].dTdq.topLeftCorner(dim*NB,dim+1)*pts;
    MatrixXd Jt = Map<MatrixXd>(tmp.data(),NB,dim*n_pts);
    MatrixXd J = Jt.transpose();
    
    if (include_rotations) {
      MatrixXd R = bodies[body_ind].T.topLeftCorner(dim,dim);
      /* 
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */

      VectorXd dR21_dq(NB),dR22_dq(NB),dR20_dq(NB),dR00_dq(NB),dR10_dq(NB);
      for (int i=0; i<NB; i++) {
	dR21_dq(i) = bodies[body_ind].dTdq(2*NB+i,1);
	dR22_dq(i) = bodies[body_ind].dTdq(2*NB+i,2);
	dR20_dq(i) = bodies[body_ind].dTdq(2*NB+i,0);
	dR00_dq(i) = bodies[body_ind].dTdq(i,0);
	dR10_dq(i) = bodies[body_ind].dTdq(NB+i,0);
      }
      double sqterm1 = R(2,1)*R(2,1) + R(2,2)*R(2,2);
      double sqterm2 = R(0,0)*R(0,0) + R(1,0)*R(1,0);
      
      MatrixXd Jr = MatrixXd::Zero(3,NB);
      
      Jr.block(0,0,1,NB) = ((R(2,2)*dR21_dq - R(2,1)*dR22_dq)/sqterm1).transpose(); 
      Jr.block(1,0,1,NB) = ((-sqrt(sqterm1)*dR20_dq + R(2,0)/sqrt(sqterm1)*(R(2,1)*dR21_dq + R(2,2)*dR22_dq) )/(R(2,0)*R(2,0) + R(2,1)*R(2,1) + R(2,2)*R(2,2))).transpose();
      Jr.block(2,0,1,NB)= ((R(0,0)*dR10_dq - R(1,0)*dR00_dq)/sqterm2).transpose();
      
      MatrixXd Jfull = MatrixXd::Zero(2*dim*n_pts,NB);
      for (int i=0; i<n_pts; i++) {
	Jfull.block(i*6,0,3,NB) = J.block(i*3,0,3,NB);
	Jfull.block(i*6+3,0,3,NB) = Jr;
      }
      return Jfull;
    } else {
      return J;
    }
  }

  MatrixXd forwardJacDot(const int body_ind, const MatrixXd pts, const bool include_rotations)
  {
    int dim=3, n_pts=pts.cols();
    if (include_rotations)
      mexErrMsgIdAndTxt("Drake:forwardKinmex:NotImplemented","Second derivatives of rotations are not implemented yet");
    
    int i,j;
    MatrixXd dJ_reshaped = MatrixXd(NB, dim*n_pts*NB);
    for (i = 0; i < NB; i++) {
      MatrixXd tmp = bodies[body_ind].ddTdqdq.block(i*NB*(dim+1),0,dim*NB,dim+1)*pts;  //dim*NB x n_pts
      for (j = 0; j < n_pts; j++) {
        dJ_reshaped.block(i,j*dim*NB,1,dim*NB) = tmp.col(j).transpose();
      }
      //       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
    }
    MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), NB*NB, dim*n_pts);
    MatrixXd dJ = dJ_t.transpose();
    return dJ;
  }

};
