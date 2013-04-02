//#include <iostream>
//#include "mex.h"
#include "RigidBodyManipulator.h"


Matrix3d rotz(double theta) {
	// returns 3D rotation matrix (about the z axis)
	Matrix3d M;
	double c=cos(theta); 
	double s=sin(theta);
	M << c,-s, 0,
		 s, c, 0,
		 0, 0, 1;
	return M;
} 

void Tjcalc(int pitch, double q, Matrix4d* TJ)
{
	*TJ = Matrix4d::Identity();

  if (pitch==0) { // revolute joint
    (*TJ).topLeftCorner(3,3) = rotz(q);
  } else if (pitch == INF) { // prismatic joint
    (*TJ)(2,3) = q;
	}	else { // helical joint
    (*TJ).topLeftCorner(3,3) = rotz(q);
    (*TJ)(2,3) = q*pitch;
  }
}

void dTjcalc(int pitch, double q, Matrix4d* dTJ)
{
	double s=sin(q); 
	double c=cos(q);
  	if (pitch==0) { // revolute joint
  		*dTJ << -s,-c, 0, 0, 
  				 c,-s, 0, 0,
  				 0, 0, 0, 0,
  				 0, 0, 0, 0;
  	} else if (pitch == INF) { // prismatic joint
  		*dTJ <<  0, 0, 0, 0,
  				 0, 0, 0, 0,
  				 0, 0, 0, 1,
  				 0, 0, 0, 0;
    } else { // helical joint
  		*dTJ << -s,-c, 0, 0,
  				 c,-s, 0, 0,
  				 0, 0, 0, pitch,
  				 0, 0, 0, 0;
  	}
}

void ddTjcalc(int pitch, double q, Matrix4d* ddTJ)
{
  	double c = cos(q);
  	double s = sin(q);

  	if (pitch==0) { // revolute joint  	
  		*ddTJ << -c, s, 0, 0,
  				 -s,-c, 0, 0,
  				  0, 0, 0, 0,
  				  0, 0, 0, 0;
  	} else if (pitch == INF) { // prismatic joint
      *ddTJ = Matrix4d::Zero();
    } else { // helical joint
      *ddTJ << -c, s, 0, 0,
              -s,-c, 0, 0,
              0, 0, 0, 0,
              0, 0, 0, 0;
    }
}

RigidBodyManipulator::RigidBodyManipulator(int n) {
  NB = n;
  pitch = new int[n];
  parent = new int[n];
  joint_limit_min = new double[n];
  joint_limit_max = new double[n];
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
    bodies[i].dofnum = i-1;  // setup default dofnums
  }
  
  // preallocate matrices used in doKinematics
  TJ = Matrix4d::Zero();
  dTJ = Matrix4d::Zero();
  ddTJ = Matrix4d::Zero();
  Tbinv = Matrix4d::Zero();
  Tb = Matrix4d::Zero();
  Tmult = Matrix4d::Zero();
  TdTmult = Matrix4d::Zero();
    
  // preallocate for COM functions
  com = Vector3d::Zero();
  bc = Vector3d::Zero();
  Jcom = MatrixXd::Zero(3,NB);
  bJ = MatrixXd::Zero(3,NB);
  dJcom = MatrixXd::Zero(3,NB*NB);
  bdJ = MatrixXd::Zero(3,NB*NB);
  
  kinematicsInit = false;
  cached_q = new double[n];
  secondDerivativesCached = 0;
}

RigidBodyManipulator::~RigidBodyManipulator() {
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


void RigidBodyManipulator::doKinematics(double* q, int b_compute_second_derivatives)
{
  int i,j,k;
  //Check against cached values for bodies[1];
  
  if (kinematicsInit) {
    bool skip = true;
    if (b_compute_second_derivatives && !secondDerivativesCached)
      skip = false;
    for (i = 0; i < NB; i++) {
      if (q[i] - cached_q[i] > 1e-8 || q[i] - cached_q[i] < -1e-8) {
        skip = false;
        break;
      }
    }
    if (skip) {
      return;
    }
  }

  MatrixXd dTmult, dTdTmult = MatrixXd::Zero(4*NB,4);
  MatrixXd ddTmult, TddTmult;
  for (i = 1; i < NB + 1; i++) {
    int parent = this->parent[i-1]+1;
    if (parent < 0) {
      bodies[i].T = bodies[i].Ttree;
      //dTdq, ddTdqdq initialized as all zeros
    }
    else {
      double qi = q[bodies[i].dofnum];
      Tjcalc(pitch[i-1],qi,&TJ);
      dTjcalc(pitch[i-1],qi,&dTJ);
      
      Tb = bodies[i].T_body_to_joint;
      Tbinv = Tb.inverse();

      Tmult = bodies[i].Ttree * Tbinv * TJ * Tb;
      
      bodies[i].T = bodies[parent].T * Tmult;
      
      /*
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */
      
      bodies[i].dTdq = bodies[parent].dTdq * Tmult;  // note: could only compute non-zero entries here
      
      dTmult = bodies[i].Ttree * Tbinv * dTJ * Tb;
      TdTmult = bodies[parent].T * dTmult;
      bodies[i].dTdq.row(bodies[i].dofnum) += TdTmult.row(0);
      bodies[i].dTdq.row(bodies[i].dofnum + NB) += TdTmult.row(1);
      bodies[i].dTdq.row(bodies[i].dofnum + 2*NB) += TdTmult.row(2);
      bodies[i].dTdq.row(bodies[i].dofnum + 3*NB) += TdTmult.row(3);
      
      if (b_compute_second_derivatives) {
        //ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
	//bodies[i].ddTdqdq = bodies[parent].ddTdqdq * Tmult // pushed this into the loop below to exploit the sparsity
	for (k=0; k<=bodies[i].dofnum; k++)  // i think it should be ok with bodies[parent].dofnum
	  for (j=0; j<4; j++) 
	    //	    bodies[i].ddTdqdq.block((4*k+j)*NB,0,1+bodies[parent].dofnum,4) = bodies[parent].ddTdqdq.block((4*k+j)*NB,0,1+bodies[parent].dofnum,4) * Tmult;  // only compute non-zero entries
	    bodies[i].ddTdqdq.block((4*k+j)*NB,0,1+bodies[i].dofnum,4) = bodies[parent].ddTdqdq.block((4*k+j)*NB,0,1+bodies[i].dofnum,4) * Tmult;  // only compute non-zero entries

        dTdTmult = bodies[parent].dTdq * dTmult;
        for (j = 0; j < 4; j++) {
          for (k = 0; k < NB; k++) { 
            bodies[i].ddTdqdq.row(bodies[i].dofnum + (4*k+j)*NB) += dTdTmult.row(j*NB+k);
          }
        }
        for (j = 0; j < 4*NB; j++) {
          bodies[i].ddTdqdq.row(4*NB*(bodies[i].dofnum) + j) += dTdTmult.row(j);
        }
        
        ddTjcalc(pitch[i-1],qi,&ddTJ);
        TddTmult = bodies[parent].T*bodies[i].Ttree * Tbinv * ddTJ * Tb;
        
        bodies[i].ddTdqdq.row(4*NB*(bodies[i].dofnum) + bodies[i].dofnum) += TddTmult.row(0);
        bodies[i].ddTdqdq.row(4*NB*(bodies[i].dofnum) + bodies[i].dofnum + NB) += TddTmult.row(1);
        bodies[i].ddTdqdq.row(4*NB*(bodies[i].dofnum) + bodies[i].dofnum + 2*NB) += TddTmult.row(2);
        bodies[i].ddTdqdq.row(4*NB*(bodies[i].dofnum) + bodies[i].dofnum + 3*NB) += TddTmult.row(3);
      }
    }
  }
  
  kinematicsInit = true;
  for (i = 0; i < NB; i++) {
    cached_q[i] = q[i];
  }
  secondDerivativesCached = b_compute_second_derivatives;
}

Vector3d RigidBodyManipulator::getCOM(void)
{
  double m = 0.0;
  double bm;
  com = Vector3d::Zero();
  
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

MatrixXd RigidBodyManipulator::getCOMJac(void)
{
  double m = 0.0;
  double bm;
  Jcom = 0*Jcom;
  
  for (int i=0; i<=NB; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      bJ = forwardJac(i,bodies[i].com,false);
      Jcom = (m*Jcom + bm*bJ)/(m+bm);
      m = m+bm;
    }
  }
  return Jcom;
}

MatrixXd RigidBodyManipulator::getCOMJacDot(void)
{
  double m = 0.0;
  double bm;
  dJcom = 0*dJcom;
  
  for (int i=0; i<=NB; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      bdJ = forwardJacDot(i,bodies[i].com,false);
      dJcom = (m*dJcom + bm*bdJ)/(m+bm);
      m = m+bm;
    }
  }
  return dJcom;
}

MatrixXd RigidBodyManipulator::forwardKin(const int body_ind, const MatrixXd pts, const bool include_rotations)
{
  // WARNING:  pts should have a trailing 1 attached to it (4xn_pts)
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

MatrixXd RigidBodyManipulator::forwardJac(const int body_ind, const MatrixXd pts, const bool include_rotations)
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

MatrixXd RigidBodyManipulator::forwardJacDot(const int body_ind, const MatrixXd pts, const bool include_rotations)
{
  int dim=3, n_pts=pts.cols();
//  if (include_rotations)
//    mexErrMsgIdAndTxt("Drake:forwardKinmex:NotImplemented","Second derivatives of rotations are not implemented yet");
    
  
  int i,j;
  MatrixXd dJ_reshaped = MatrixXd(NB, dim*n_pts*NB);
  MatrixXd tmp = MatrixXd(dim*NB,n_pts);
  for (i = 0; i < NB; i++) {
    tmp = bodies[body_ind].ddTdqdq.block(i*NB*(dim+1),0,dim*NB,dim+1)*pts;  //dim*NB x n_pts
    for (j = 0; j < n_pts; j++) {
      dJ_reshaped.block(i,j*dim*NB,1,dim*NB) = tmp.col(j).transpose();
    }
    //       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
  }
  MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), NB*NB, dim*n_pts);
  MatrixXd dJ = dJ_t.transpose();
  return dJ;
}

