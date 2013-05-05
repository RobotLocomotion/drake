#include <iostream>

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

RigidBodyManipulator::RigidBodyManipulator(int ndof, int num_featherstone_bodies, int num_rigid_body_objects) 
{
  num_dof = ndof;
  if (num_featherstone_bodies<0)
    NB = num_dof;
  else
    NB = num_featherstone_bodies;
  pitch = new int[NB];
  parent = new int[NB];
  dofnum = new int[NB];
  joint_limit_min = new double[NB];
  joint_limit_max = new double[NB];
  Xtree = new MatrixXd[NB];
  I = new MatrixXd[NB];
  a_grav = VectorXd::Zero(6);
  
  S = new VectorXd[NB];
  Xup = new MatrixXd[NB];
  v = new VectorXd[NB];
  avp = new VectorXd[NB];
  fvp = new VectorXd[NB];
  IC = new MatrixXd[NB];
  
  for(int i=0; i < NB; i++) {
    Xtree[i] = MatrixXd::Zero(6,6);
    I[i] = MatrixXd::Zero(6,6);
    S[i] = VectorXd::Zero(6);
    Xup[i] = MatrixXd::Zero(6,6);
    v[i] = VectorXd::Zero(6);
    avp[i] = VectorXd::Zero(6);
    fvp[i] = VectorXd::Zero(6);
    IC[i] = MatrixXd::Zero(6,6);
  }
  
  H = MatrixXd::Zero(num_dof,num_dof);
  C.resize(num_dof,1); // C gets over-written completely by the algorithm below.
  
  //Variable allocation for gradient calculations
  dXupdq = new MatrixXd[NB];
  dIC = new MatrixXd*[NB];
  for(int i=0; i < NB; i++) {
    dIC[i] = new MatrixXd[NB];
    for(int j=0; j < NB; j++) {
      dIC[i][j] = MatrixXd::Zero(6,6);
    }
  }
  dH = MatrixXd::Zero(num_dof*num_dof,num_dof);
  dvJdqd_mat = MatrixXd::Zero(6,num_dof);
//     dcross.resize(6,n);
  dC = MatrixXd::Zero(num_dof,num_dof);
  
  dvdq = new MatrixXd[NB];
  dvdqd = new MatrixXd[NB];
  davpdq = new MatrixXd[NB];
  davpdqd = new MatrixXd[NB];
  dfvpdq = new MatrixXd[NB];
  dfvpdqd = new MatrixXd[NB];
  
  for(int i=0; i < NB; i++) {
    dvdq[i] = MatrixXd::Zero(6,num_dof);
    dvdqd[i] = MatrixXd::Zero(6,num_dof);
    davpdq[i] = MatrixXd::Zero(6,num_dof);
    davpdqd[i] = MatrixXd::Zero(6,num_dof);
    dfvpdq[i] = MatrixXd::Zero(6,num_dof);
    dfvpdqd[i] = MatrixXd::Zero(6,num_dof);
  }

  if (num_rigid_body_objects<0)
    num_bodies = NB+1;  // this was my old assumption, so leave it here as the default behavior
  else
    num_bodies = num_rigid_body_objects;
  bodies = new RigidBody[num_bodies];
  
  for(int i=0; i < num_bodies; i++) {
    bodies[i].setN(NB);
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
  Jcom = MatrixXd::Zero(3,num_dof);
  Jcomdot = MatrixXd::Zero(3,num_dof);
  bJ = MatrixXd::Zero(3,num_dof);
  dJcom = MatrixXd::Zero(3,num_dof*num_dof);
  bdJ = MatrixXd::Zero(3,num_dof*num_dof);
  
  initialized = false;
  kinematicsInit = false;
  cached_q = new double[num_dof];
  cached_qd = new double[num_dof];
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
  delete[] cached_qd;
}

void RigidBodyManipulator::compile(void) 
{
  // precompute sparsity pattern for each rigid body
  for (int i=0; i<num_bodies; i++) 
    bodies[i].computeAncestorDOFs(this);
  initialized=true;
}

void RigidBodyManipulator::doKinematics(double* q, bool b_compute_second_derivatives, double* qd)
{
  int i,j,k;

  //Check against cached values for bodies[1];
  if (kinematicsInit) {
    bool skip = true;
    if (b_compute_second_derivatives && !secondDerivativesCached)
      skip = false;
    for (i = 0; i < num_dof; i++) {
      if (q[i] - cached_q[i] > 1e-8 || q[i] - cached_q[i] < -1e-8) {
        skip = false;
        break;
      }
    }
    if (skip) {
      return;
    }
  }

  if (!initialized) compile();

  Matrix4d TJdot, dTJdot, dTdotmult;
  MatrixXd dTdTmult, ddTmult, TddTmult;
  
  for (i = 0; i < num_bodies; i++) {
    int parent = bodies[i].parent;
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
      bodies[i].dTdq.row(bodies[i].dofnum + num_dof) += TdTmult.row(1);
      bodies[i].dTdq.row(bodies[i].dofnum + 2*num_dof) += TdTmult.row(2);
      
      if (b_compute_second_derivatives) {
        //ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        //	bodies[i].ddTdqdq = bodies[parent].ddTdqdq * Tmult; // pushed this into the loop below to exploit the sparsity
        for (std::set<IndexRange>::iterator iter = bodies[parent].ddTdqdq_nonzero_rows_grouped.begin(); iter != bodies[parent].ddTdqdq_nonzero_rows_grouped.end(); iter++) {
          bodies[i].ddTdqdq.block(iter->start,0,iter->length,4) = bodies[parent].ddTdqdq.block(iter->start,0,iter->length,4) * Tmult;
        }

        dTdTmult = bodies[parent].dTdq * dTmult;
        for (j = 0; j < 3*num_dof; j++) {
          bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + j) = dTdTmult.row(j);
        }

        for (j = 0; j < 3; j++) {
          for (k = 0; k < num_dof; k++) { 
            if (k == bodies[i].dofnum) {
              bodies[i].ddTdqdq.row(bodies[i].dofnum + (3*k+j)*num_dof) += dTdTmult.row(j*num_dof+k);
            } else {
              bodies[i].ddTdqdq.row(bodies[i].dofnum + (3*k+j)*num_dof) = dTdTmult.row(j*num_dof+k);
            }
          }
        }
        
        ddTjcalc(pitch[i-1],qi,&ddTJ);
        TddTmult = bodies[parent].T*bodies[i].Ttree * Tbinv * ddTJ * Tb;
        
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum) += TddTmult.row(0);
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum + num_dof) += TddTmult.row(1);
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum + 2*num_dof) += TddTmult.row(2);
      }
      
      if (qd) {
        double qdi = qd[bodies[i].dofnum];
        TJdot = dTJ*qdi;
        ddTjcalc(pitch[i-1],qi,&ddTJ);
        dTJdot = ddTJ*qdi;

//        body.Tdot = body.parent.Tdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        dTdotmult = bodies[i].Ttree * Tbinv * TJdot * Tb;
        bodies[i].Tdot = bodies[parent].Tdot*Tmult + bodies[parent].T * dTdotmult;
//        body.dTdqdot = body.parent.dTdqdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        bodies[i].dTdqdot = bodies[parent].dTdqdot* Tmult + bodies[parent].dTdq * dTdotmult;  

//        body.dTdqdot(this_dof_ind,:) = body.dTdqdot(this_dof_ind,:) + body.parent.Tdot(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot*body.T_body_to_joint;
        dTdotmult = bodies[parent].Tdot*dTmult + bodies[parent].T*bodies[i].Ttree*Tbinv*dTJdot*Tb;
        bodies[i].dTdqdot.row(bodies[i].dofnum) += dTdotmult.row(0);
        bodies[i].dTdqdot.row(bodies[i].dofnum + num_dof) += dTdotmult.row(1);
        bodies[i].dTdqdot.row(bodies[i].dofnum + 2*num_dof) += dTdotmult.row(2);
      }
    }
  }
  
  kinematicsInit = true;
  for (i = 0; i < num_dof; i++) {
    cached_q[i] = q[i];
    if (qd) cached_qd[i] = qd[i];
  }
  secondDerivativesCached = b_compute_second_derivatives;
}

Vector3d RigidBodyManipulator::getCOM(void)
{
  double m = 0.0;
  double bm;
  com = Vector3d::Zero();
  
  for (int i=0; i<num_bodies; i++) {
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
  
  for (int i=0; i<num_bodies; i++) {
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
  Jcomdot = 0*Jcomdot;
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      bJ = forwardJacDot(i,bodies[i].com);
      Jcomdot = (m*Jcomdot + bm*bJ)/(m+bm);
      m = m+bm;
    }
  }
  return Jcomdot;
}

MatrixXd RigidBodyManipulator::getCOMdJac(void)
{
  double m = 0.0;
  double bm;
  dJcom = 0*dJcom;
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      bdJ = forwarddJac(i,bodies[i].com);
      dJcom = (m*dJcom + bm*bdJ)/(m+bm);
      m = m+bm;
    }
  }
  return dJcom;
}

/*
 * rotation_type  0, no rotation
 * 		  1, output Euler angles
 * 		  2, output quaternion [w,x,y,z], with w>=0
 */
MatrixXd RigidBodyManipulator::forwardKin(const int body_ind, const MatrixXd pts, const int rotation_type)
{
  // WARNING:  pts should have a trailing 1 attached to it (4xn_pts)
  int dim=3, n_pts = pts.cols();
  MatrixXd T = bodies[body_ind].T.topLeftCorner(dim,dim+1);
  if (rotation_type == 0) {
    return T*pts;
  } else if (rotation_type == 1) {
    Vector3d rpy;
    rpy << atan2(T(2,1),T(2,2)), atan2(-T(2,0),sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2))), atan2(T(1,0),T(0,0));
    // NOTE: we're assuming an X-Y-Z convention was used to construct T
    
    MatrixXd X = MatrixXd::Zero(2*dim,n_pts);
    X.block(0,0,3,n_pts) = T*pts;
    X.block(3,0,3,n_pts) = rpy.replicate(1,n_pts);
    return X;
  } else if(rotation_type == 2) {
    Vector4d quat;
    double qw = sqrt(1+T(0,0)+T(1,1)+T(2,2))/2;
    double qx = (T(2,1)-T(1,2))/(4*qw);
    double qy = (T(0,2)-T(2,0))/(4*qw);
    double qz = (T(1,0)-T(0,1))/(4*qw);
    quat << qw, qx, qy, qz;
    MatrixXd X = MatrixXd::Zero(7,n_pts);
    X.block(0,0,3,n_pts) = T*pts;
    X.block(3,0,4,n_pts) = quat.replicate(1,n_pts);
    return X;
  }
}

MatrixXd RigidBodyManipulator::forwardJac(const int body_ind, const MatrixXd pts, const int rotation_type)
{
  int dim = 3, n_pts = pts.cols();
  MatrixXd tmp = bodies[body_ind].dTdq.topLeftCorner(dim*NB,dim+1)*pts;
  MatrixXd Jt = Map<MatrixXd>(tmp.data(),num_dof,dim*n_pts);
  MatrixXd J = Jt.transpose();
  
  if (rotation_type == 1) {
    MatrixXd R = bodies[body_ind].T.topLeftCorner(dim,dim);
    /*
     * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
     * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
     */
    
    VectorXd dR21_dq(num_dof),dR22_dq(num_dof),dR20_dq(num_dof),dR00_dq(num_dof),dR10_dq(num_dof);
    for (int i=0; i<num_dof; i++) {
      dR21_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,1);
      dR22_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,2);
      dR20_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,0);
      dR00_dq(i) = bodies[body_ind].dTdq(i,0);
      dR10_dq(i) = bodies[body_ind].dTdq(num_dof+i,0);
    }
    double sqterm1 = R(2,1)*R(2,1) + R(2,2)*R(2,2);
    double sqterm2 = R(0,0)*R(0,0) + R(1,0)*R(1,0);
    
    MatrixXd Jr = MatrixXd::Zero(3,num_dof);
    
    Jr.block(0,0,1,num_dof) = ((R(2,2)*dR21_dq - R(2,1)*dR22_dq)/sqterm1).transpose();
    Jr.block(1,0,1,num_dof) = ((-sqrt(sqterm1)*dR20_dq + R(2,0)/sqrt(sqterm1)*(R(2,1)*dR21_dq + R(2,2)*dR22_dq) )/(R(2,0)*R(2,0) + R(2,1)*R(2,1) + R(2,2)*R(2,2))).transpose();
    Jr.block(2,0,1,num_dof)= ((R(0,0)*dR10_dq - R(1,0)*dR00_dq)/sqterm2).transpose();
    
    MatrixXd Jfull = MatrixXd::Zero(2*dim*n_pts,num_dof);
    for (int i=0; i<n_pts; i++) {
      Jfull.block(i*6,0,3,num_dof) = J.block(i*3,0,3,num_dof);
      Jfull.block(i*6+3,0,3,num_dof) = Jr;
    }
    return Jfull;
  } else if(rotation_type == 0){
    return J;
  }
  else if(rotation_type == 2)
  {
    MatrixXd R = bodies[body_ind].T.topLeftCorner(dim,dim);
    /*
     * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
     * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
     */
    
    VectorXd dR21_dq(num_dof),dR22_dq(num_dof),dR20_dq(num_dof),dR00_dq(num_dof),dR10_dq(num_dof),dR01_dq(num_dof),dR02_dq(num_dof),dR11_dq(num_dof),dR12_dq(num_dof);
    for (int i=0; i<num_dof; i++) {
      dR21_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,1);
      dR22_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,2);
      dR20_dq(i) = bodies[body_ind].dTdq(2*num_dof+i,0);
      dR00_dq(i) = bodies[body_ind].dTdq(i,0);
      dR10_dq(i) = bodies[body_ind].dTdq(num_dof+i,0);
      dR01_dq(i) = bodies[body_ind].dTdq(i,1);
      dR02_dq(i) = bodies[body_ind].dTdq(i,2);
      dR11_dq(i) = bodies[body_ind].dTdq(num_dof+i,1);
      dR12_dq(i) = bodies[body_ind].dTdq(num_dof+i,2);
    }

    double qw = sqrt(1+R(0,0)+R(1,1)+R(2,2))/2;
    MatrixXd Jq = MatrixXd::Zero(4,num_dof);
    VectorXd dqwdq = (dR00_dq+dR11_dq+dR22_dq)/(4*sqrt(1+R(0,0)+R(1,1)+R(2,2)));
    double wsquare4 = 4*qw*qw;
    Jq.block(0,0,1,num_dof) = dqwdq.transpose();
    Jq.block(1,0,1,num_dof) = (((dR21_dq-dR12_dq)*qw-(R(2,1)-R(1,2))*dqwdq).transpose())/wsquare4;
    Jq.block(2,0,1,num_dof) = (((dR02_dq-dR20_dq)*qw-(R(0,2)-R(2,0))*dqwdq).transpose())/wsquare4;
    Jq.block(3,0,1,num_dof) = (((dR10_dq-dR01_dq)*qw-(R(1,0)-R(0,1))*dqwdq).transpose())/wsquare4;
    
    MatrixXd Jfull = MatrixXd::Zero(7*n_pts,num_dof);
    for (int i=0;i<n_pts;i++)
    {
	    Jfull.block(i*7,0,3,num_dof) = J.block(i*3,0,3,num_dof);
	    Jfull.block(i*7+3,0,4,num_dof) = Jq;
    }
    return Jfull;
  }
}


MatrixXd RigidBodyManipulator::forwardJacDot(const int body_ind, const MatrixXd pts)
{
  int dim = 3, n_pts = pts.cols();
  MatrixXd tmp = bodies[body_ind].dTdqdot*pts;
  MatrixXd Jdott = Map<MatrixXd>(tmp.data(),num_dof,dim*n_pts);
  MatrixXd Jdot = Jdott.transpose();
  
  return Jdot;
}

MatrixXd RigidBodyManipulator::forwarddJac(const int body_ind, const MatrixXd pts)
{
  int dim=3, n_pts=pts.cols();
  
  int i,j;
  MatrixXd dJ_reshaped = MatrixXd(num_dof, dim*n_pts*num_dof);
  MatrixXd tmp = MatrixXd(dim*num_dof,n_pts);
  for (i = 0; i < num_dof; i++) {
    tmp = bodies[body_ind].ddTdqdq.block(i*num_dof*dim,0,dim*num_dof,dim+1)*pts;  //dim*num_dof x n_pts
    for (j = 0; j < n_pts; j++) {
      dJ_reshaped.block(i,j*dim*num_dof,1,dim*num_dof) = tmp.col(j).transpose();
    }
    //       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
  }
  MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), num_dof*num_dof, dim*n_pts);
  MatrixXd dJ = dJ_t.transpose();
  return dJ;
}

