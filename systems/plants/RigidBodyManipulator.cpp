#include <iostream>

//#include "mex.h"
#include "RigidBodyManipulator.h"

template<typename Derived>
bool isnotnan(const MatrixBase<Derived>& x)
{
  return (x.array() == x.array()).all();
}

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

void rotx(double theta, Matrix3d &M, Matrix3d &dM, Matrix3d &ddM)
{
  double c=cos(theta), s=sin(theta);
  M << 1,0,0, 0,c,-s, 0,s,c;
  dM << 0,0,0, 0,-s,-c, 0,c,-s;
  ddM << 0,0,0, 0,-c,s, 0,-s,-c;
}

void roty(double theta, Matrix3d &M, Matrix3d &dM, Matrix3d &ddM)
{
  theta=-theta;
  double c=cos(theta), s=sin(theta);
  M << c,0,-s, 0,1,0, s,0,c;
  dM << -s,0,-c, 0,0,0, c,0,-s;  dM = -dM;
  ddM << -c,0,s, 0,0,0, -s,0,-c;
}

void rotz(double theta, Matrix3d &M, Matrix3d &dM, Matrix3d &ddM)
{
  double c=cos(theta), s=sin(theta);
  M << c,-s,0, s,c,0, 0,0,1;
  dM << -s,-c,0, c,-s,0, 0,0,0;
  ddM << -c,s,0, -s,-c,0, 0,0,0;
}

RigidBodyManipulator::RigidBodyManipulator(int ndof, int num_featherstone_bodies, int num_rigid_body_objects) 
#ifdef BULLET_COLLISION
  :  bt_collision_configuration(),
     bt_collision_dispatcher( &bt_collision_configuration ),
     bt_collision_broadphase(),
     bt_collision_world( &bt_collision_dispatcher,
                       &bt_collision_broadphase,
                       &bt_collision_configuration )
#endif
{
  num_dof = ndof;
  if (num_featherstone_bodies<0)
    NB = num_dof;
  else
    NB = num_featherstone_bodies;
  pitch = new int[NB];
  parent = new int[NB];
  dofnum = new int[NB];
  damping = new double[NB];
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
  
  //Variable allocation for gradient calculations
  dXupdq = new MatrixXd[NB];
  dIC = new MatrixXd*[NB];
  for(int i=0; i < NB; i++) {
    dIC[i] = new MatrixXd[NB];
    for(int j=0; j < NB; j++) {
      dIC[i][j] = MatrixXd::Zero(6,6);
    }
  }
  dvJdqd_mat = MatrixXd::Zero(6,num_dof);
//     dcross.resize(6,n);
  
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
  
  // preallocate for COM functions
  bc = Vector3d::Zero();
  bJ = MatrixXd::Zero(3,num_dof);
  bdJ = MatrixXd::Zero(3,num_dof*num_dof);
  dTdTmult = MatrixXd::Zero(3*num_dof,4);

  initialized = false;
  kinematicsInit = false;
  cached_q = new double[num_dof];
  cached_qd = new double[num_dof];
  secondDerivativesCached = 0;
}

RigidBodyManipulator::~RigidBodyManipulator() {

#ifdef BULLET_COLLISION
  for (int i=0; i<num_bodies; i++) {
	  for (std::vector<RigidBody::CollisionObject>::iterator iter=bodies[i].collision_objects.begin(); iter!=bodies[i].collision_objects.end(); iter++) {
	  	bt_collision_world.removeCollisionObject(iter->bt_obj);
	  	delete iter->bt_obj;
	  	delete iter->bt_shape;
	  }
  }
#endif


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
  num_contact_pts = 0;
  for (int i=0; i<num_bodies; i++) {
    num_contact_pts += bodies[i].contact_pts.cols();
            
    // precompute sparsity pattern for each rigid body
    bodies[i].computeAncestorDOFs(this);
  }  
  
  initialized=true;
}

#ifdef BULLET_COLLISION
void RigidBodyManipulator::updateCollisionObjects(int body_ind)
{
	btMatrix3x3 rot;
	btVector3 pos;
	btTransform btT;
	Matrix4d T;
	for (std::vector<RigidBody::CollisionObject>::iterator iter=bodies[body_ind].collision_objects.begin(); iter!=bodies[body_ind].collision_objects.end(); iter++) {
		T = bodies[body_ind].T*(iter->T);
		  rot.setValue( T(0,0), T(0,1), T(0,2),
  			T(1,0), T(1,1), T(1,2),
  			T(2,0), T(2,1), T(2,2) );
    	btT.setBasis(rot);
    	pos.setValue( T(0,3), T(1,3), T(2,3) );
    	btT.setOrigin(pos);

    	iter->bt_obj->setWorldTransform(btT);
    	bt_collision_world.updateSingleAabb(iter->bt_obj);
	}

}

class MyCollisionResultCollector : public btCollisionWorld::ContactResultCallback
{
public:
	std::vector<Vector3d> ptsA, ptsB;

	virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
	{
		Vector3d pt;
		btVector3 bt_pt = cp.getPositionWorldOnA();
		pt(0) = (double) bt_pt.getX();
		pt(1) = (double) bt_pt.getY();
		pt(2) = (double) bt_pt.getZ();
		ptsA.push_back(pt);

		bt_pt = cp.getPositionWorldOnB();
		pt(0) = (double) bt_pt.getX();
		pt(1) = (double) bt_pt.getY();
		pt(2) = (double) bt_pt.getZ();
		ptsB.push_back(pt);

		return 0;
	}
};

bool RigidBodyManipulator::getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB)
{
	MyCollisionResultCollector c;
	for (std::vector<RigidBody::CollisionObject>::iterator iterA=bodies[body_indA].collision_objects.begin(); iterA!=bodies[body_indA].collision_objects.end(); iterA++) {
		for (std::vector<RigidBody::CollisionObject>::iterator iterB=bodies[body_indB].collision_objects.begin(); iterB!=bodies[body_indB].collision_objects.end(); iterB++) {
			bt_collision_world.contactPairTest(iterA->bt_obj,iterB->bt_obj,c);
		}
	}

	ptsA.resize(3,c.ptsA.size());
	ptsB.resize(3,c.ptsB.size());

	for (int i=0; i<c.ptsA.size(); i++) {
		ptsA.col(i) = c.ptsA[i];
		ptsB.col(i) = c.ptsB[i];
	}

  return (c.ptsA.size() > 0);
}

#endif

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

  Matrix4d TJ, dTJ, ddTJ, Tbinv, Tb, Tmult, dTmult, dTdotmult, TdTmult, TJdot, dTJdot, TddTmult;
  Matrix4d fb_dTJ[6], fb_dTJdot[6], fb_dTmult[6];  // will be 7 when quats implemented...

  Matrix3d rx,drx,ddrx,ry,dry,ddry,rz,drz,ddrz;
  
  for (i = 0; i < num_bodies; i++) {
    int parent = bodies[i].parent;
    if (parent < 0) {
      bodies[i].T = bodies[i].Ttree;
      //dTdq, ddTdqdq initialized as all zeros
    } else if (bodies[i].floating == 1) {
      double qi[6];
      for (int j=0; j<6; j++) qi[j] = q[bodies[i].dofnum+j]; 
      
      rotx(qi[3],rx,drx,ddrx);
      roty(qi[4],ry,dry,ddry);
      rotz(qi[5],rz,drz,ddrz);

      Tb = bodies[i].T_body_to_joint;
      Tbinv = Tb.inverse();

      TJ = Matrix4d::Identity();  TJ.block<3,3>(0,0) = rz*ry*rx;  TJ(0,3)=qi[0]; TJ(1,3)=qi[1]; TJ(2,3)=qi[2];

      Tmult = bodies[i].Ttree * Tbinv * TJ * Tb;
      bodies[i].T = bodies[parent].T * Tmult;
      
      // see notes below
      bodies[i].dTdq = bodies[parent].dTdq * Tmult;
      dTmult = bodies[i].Ttree * Tbinv * dTJ * Tb;
      TdTmult = bodies[parent].T * dTmult;
      
      fb_dTJ[0] << 0,0,0,1, 0,0,0,0, 0,0,0,0, 0,0,0,0;
      fb_dTJ[1] << 0,0,0,0, 0,0,0,1, 0,0,0,0, 0,0,0,0;
      fb_dTJ[2] << 0,0,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0;
      fb_dTJ[3] = Matrix4d::Zero(); fb_dTJ[3].block<3,3>(0,0) = rz*ry*drx;
      fb_dTJ[4] = Matrix4d::Zero(); fb_dTJ[4].block<3,3>(0,0) = rz*dry*rx;
      fb_dTJ[5] = Matrix4d::Zero(); fb_dTJ[5].block<3,3>(0,0) = drz*ry*rx;

      for (int j=0; j<6; j++) {
        fb_dTmult[j] = bodies[i].Ttree * Tbinv * fb_dTJ[j] * Tb;
        TdTmult = bodies[parent].T * fb_dTmult[j];
        bodies[i].dTdq.row(bodies[i].dofnum + j) += TdTmult.row(0);
        bodies[i].dTdq.row(bodies[i].dofnum + j + num_dof) += TdTmult.row(1);
        bodies[i].dTdq.row(bodies[i].dofnum + j + 2*num_dof) += TdTmult.row(2);
      }

      if (b_compute_second_derivatives) {
        std::cerr << "mex kinematics for floating base second derivatives are not implemented yet" << std::endl;
      }
      if (qd) {
        double qdi[6];

        TJdot = Matrix4d::Zero();
        for (int j=0; j<6; j++) {
          qdi[j] = qd[bodies[i].dofnum+j];
          TJdot += fb_dTJ[j]*qdi[j];
        }

        fb_dTJdot[0] = Matrix4d::Zero();
        fb_dTJdot[1] = Matrix4d::Zero();
        fb_dTJdot[2] = Matrix4d::Zero();
        fb_dTJdot[3] = Matrix4d::Zero();  fb_dTJdot[3].block<3,3>(0,0) = (drz*qdi[5])*ry*drx + rz*(dry*qdi[4])*drx + rz*ry*(ddrx*qdi[3]);
        fb_dTJdot[4] = Matrix4d::Zero();  fb_dTJdot[4].block<3,3>(0,0) = (drz*qdi[5])*dry*rx + rz*(ddry*qdi[4])*rx + rz*dry*(drx*qdi[3]);
        fb_dTJdot[5] = Matrix4d::Zero();  fb_dTJdot[5].block<3,3>(0,0) = (ddrz*qdi[5])*ry*rx + drz*(dry*qdi[4])*rx + drz*ry*(drx*qdi[3]);

        dTdotmult = bodies[i].Ttree * Tbinv * TJdot * Tb;
        bodies[i].Tdot = bodies[parent].Tdot*Tmult + bodies[parent].T * dTdotmult;

        bodies[i].dTdqdot = bodies[parent].dTdqdot* Tmult + bodies[parent].dTdq * dTdotmult;  

        for (int j=0; j<6; j++) {
          dTdotmult = bodies[parent].Tdot*fb_dTmult[j] + bodies[parent].T*bodies[i].Ttree*Tbinv*fb_dTJdot[j]*Tb;
          bodies[i].dTdqdot.row(bodies[i].dofnum + j) += dTdotmult.row(0);
          bodies[i].dTdqdot.row(bodies[i].dofnum + j + num_dof) += dTdotmult.row(1);
          bodies[i].dTdqdot.row(bodies[i].dofnum + j + 2*num_dof) += dTdotmult.row(2);
        }
      }
      
    } else if (bodies[i].floating == 2) {
      std::cerr << "mex kinematics for quaternion floating bases are not implemented yet" << std::endl;
    } else {
      double qi = q[bodies[i].dofnum];
      Tjcalc(bodies[i].pitch,qi,&TJ);
      dTjcalc(bodies[i].pitch,qi,&dTJ);
      
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
        
        ddTjcalc(bodies[i].pitch,qi,&ddTJ);
        TddTmult = bodies[parent].T*bodies[i].Ttree * Tbinv * ddTJ * Tb;
        
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum) += TddTmult.row(0);
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum + num_dof) += TddTmult.row(1);
        bodies[i].ddTdqdq.row(3*num_dof*(bodies[i].dofnum) + bodies[i].dofnum + 2*num_dof) += TddTmult.row(2);
      }
      
      if (qd) {
        double qdi = qd[bodies[i].dofnum];
        TJdot = dTJ*qdi;
        ddTjcalc(bodies[i].pitch,qi,&ddTJ);
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

#ifdef BULLET_COLLISION
    if (bodies[i].parent>=0) {
    	updateCollisionObjects(i);
    }
#endif

  }

  kinematicsInit = true;
  for (i = 0; i < num_dof; i++) {
    cached_q[i] = q[i];
    if (qd) cached_qd[i] = qd[i];
  }
  secondDerivativesCached = b_compute_second_derivatives;
}

template <typename Derived>
void RigidBodyManipulator::getCOM(MatrixBase<Derived> &com)
{
  double m = 0.0;
  double bm;
  com = Vector3d::Zero();
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      forwardKin(i,bodies[i].com,0,bc);
      com = (m*com + bm*bc)/(m+bm);
      m = m+bm;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMJac(MatrixBase<Derived> &Jcom)
{
  double m = 0.0;
  double bm;
  Jcom = MatrixXd::Zero(3,num_dof);
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      forwardJac(i,bodies[i].com,0,bJ);
      Jcom = (m*Jcom + bm*bJ)/(m+bm);
      m = m+bm;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMJacDot(MatrixBase<Derived> &Jcomdot)
{
  double m = 0.0;
  double bm;
  Jcomdot = MatrixXd::Zero(3,num_dof);
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      forwardJacDot(i,bodies[i].com,bJ);
      Jcomdot = (m*Jcomdot + bm*bJ)/(m+bm);
      m = m+bm;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMdJac(MatrixBase<Derived> &dJcom)
{
  double m = 0.0;
  double bm;
  dJcom = MatrixXd::Zero(3,num_dof*num_dof);
  
  for (int i=0; i<num_bodies; i++) {
    bm = bodies[i].mass;
    if (bm>0) {
      forwarddJac(i,bodies[i].com,bdJ);
      dJcom = (m*dJcom + bm*bdJ)/(m+bm);
      m = m+bm;
    }
  }
}

int RigidBodyManipulator::getNumContacts(const std::set<int> &body_idx)
{
  int n=0,nb=body_idx.size(),bi;
  if (nb==0) nb=num_bodies; 
  std::set<int>::iterator iter = body_idx.begin();
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    n += bodies[bi].contact_pts.cols();
  }
  return n;
}


template <typename Derived>
void RigidBodyManipulator::getContactPositions(MatrixBase<Derived> &pos, const std::set<int> &body_idx)
{
  int n=0,nc,nb=body_idx.size(),bi;
  if (nb==0) nb=num_bodies; 
  std::set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = bodies[bi].contact_pts.cols();
    if (nc>0) {
      // note: it's possible to pass pos.block in directly but requires such an ugly hack that I think it's not worth it:
      // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
      p.resize(3,nc);
      forwardKin(bi,bodies[bi].contact_pts,0,p);
      pos.block(0,n,3,nc) = p;
      n += nc;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getContactPositionsJac(MatrixBase<Derived> &J, const std::set<int> &body_idx)
{
  int n=0,nc,nb=body_idx.size(),bi;
  if (nb==0) nb=num_bodies; 
  std::set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = bodies[bi].contact_pts.cols();
    if (nc>0) {
      p.resize(3*nc,num_dof);
      forwardJac(bi,bodies[bi].contact_pts,0,p);
      J.block(3*n,0,3*nc,num_dof) = p;
      n += nc;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const std::set<int> &body_idx)
{
  int n=0,nc,nb=body_idx.size(),bi;
  if (nb==0) nb=num_bodies; 
  std::set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = bodies[bi].contact_pts.cols();
    if (nc>0) {
      p.resize(3*nc,num_dof);
      forwardJacDot(bi,bodies[bi].contact_pts,p);
      Jdot.block(3*n,0,3*nc,num_dof) = p;
      n += nc;
    }
  }
}


/*
 * rotation_type  0, no rotation
 * 		  1, output Euler angles
 * 		  2, output quaternion [w,x,y,z], with w>=0
 */

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwardKin(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x)
{
  int dim=3, n_pts = pts.cols();
  MatrixXd T = bodies[body_ind].T.topLeftCorner(dim,dim+1);
  if (rotation_type == 0) {
    x = T*pts;
  } else if (rotation_type == 1) {
    Vector3d rpy;
    rpy << atan2(T(2,1),T(2,2)), atan2(-T(2,0),sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2))), atan2(T(1,0),T(0,0));
    // NOTE: we're assuming an X-Y-Z convention was used to construct T
    
    x = MatrixXd::Zero(2*dim,n_pts);
    x.block(0,0,3,n_pts) = T*pts;
    x.block(3,0,3,n_pts) = rpy.replicate(1,n_pts);
  } else if(rotation_type == 2) {
    Vector4d quat;
    double qw = sqrt(1+T(0,0)+T(1,1)+T(2,2))/2;
    double qx = (T(2,1)-T(1,2))/(4*qw);
    double qy = (T(0,2)-T(2,0))/(4*qw);
    double qz = (T(1,0)-T(0,1))/(4*qw);
    quat << qw, qx, qy, qz;
    x = MatrixXd::Zero(7,n_pts);
    x.block(0,0,3,n_pts) = T*pts;
    x.block(3,0,4,n_pts) = quat.replicate(1,n_pts);
  }
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::bodyKin(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x)
{
  int dim=3, n_pts = pts.cols();
  MatrixXd Tinv = bodies[body_ind].T.inverse();
  x = Tinv.topLeftCorner(dim,dim+1)*pts;
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwardJac(const int body_ind, const MatrixBase<DerivedA> &pts, const int rotation_type, MatrixBase<DerivedB> &J)
{
  int dim = 3, n_pts = pts.cols();
  MatrixXd tmp = bodies[body_ind].dTdq.topLeftCorner(dim*num_dof,dim+1)*pts;
  MatrixXd Jt = Map<MatrixXd>(tmp.data(),num_dof,dim*n_pts);
  J.topLeftCorner(dim*n_pts,num_dof) = Jt.transpose();
  
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
    J=Jfull;
  } else if(rotation_type == 2) {
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

    double qw = sqrt(1.0+R(0,0)+R(1,1)+R(2,2))/2.0;
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
    J =  Jfull;
  }
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwardJacDot(const int body_ind, const MatrixBase<DerivedA> &pts, MatrixBase<DerivedB>& Jdot)
{
  int dim = 3, n_pts = pts.cols();
  MatrixXd tmp = bodies[body_ind].dTdqdot*pts;
  MatrixXd Jdott = Map<MatrixXd>(tmp.data(),num_dof,dim*n_pts);
  Jdot = Jdott.transpose();
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwarddJac(const int body_ind, const MatrixBase<DerivedA> &pts, MatrixBase<DerivedB>& dJ)
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
  dJ = dJ_t.transpose();
}

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
void RigidBodyManipulator::HandC(double * const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH, MatrixBase<DerivedE> *dC)
{
  H = MatrixXd::Zero(num_dof,num_dof);
  if (dH) *dH = MatrixXd::Zero(num_dof*num_dof,num_dof);
  // C gets overwritten completely in the algorithm below

  VectorXd vJ(6), fh(6), dfh(6), dvJdqd(6);
  MatrixXd XJ(6,6), dXJdq(6,6);
  int i,j,k,n,np;
  
  for (i=0; i<NB; i++) {
    n = dofnum[i];
    jcalc(pitch[i],q[n],&XJ,&(S[i]));
    vJ = S[i] * qd[n];
    Xup[i] = XJ * Xtree[i];
    
    if (parent[i] < 0) {
      v[i] = vJ;
      avp[i] = Xup[i] * (-a_grav);
    } else {
      v[i] = Xup[i]*v[parent[i]] + vJ;
      avp[i] = Xup[i]*avp[parent[i]] + crm(v[i])*vJ;
    }
    fvp[i] = I[i]*avp[i] + crf(v[i])*I[i]*v[i];
    if (f_ext)
      fvp[i] = fvp[i] - f_ext->col(i);
    IC[i] = I[i];
    
    //Calculate gradient information if it is requested
    if (dH || dC) {
      djcalc(pitch[i], q[n], &dXJdq);
      dXupdq[i] = dXJdq * Xtree[i];
      
      for (j=0; j<NB; j++) {
        dIC[i][j] = MatrixXd::Zero(6,6);
      }
    }
    
    if (dC) {
      dvJdqd = S[i];
      if (parent[i] < 0) {
        dvdqd[i].col(n) = dvJdqd;
        davpdq[i].col(n) = dXupdq[i] * (-a_grav);
      } else {
        j = parent[i];
        dvdq[i] = Xup[i]*dvdq[j];
        dvdq[i].col(n) += dXupdq[i]*v[j];
        dvdqd[i] = Xup[i]*dvdqd[j];
        dvdqd[i].col(n) += dvJdqd;
        
        davpdq[i] = Xup[i]*davpdq[j];
        davpdq[i].col(n) += dXupdq[i]*avp[j];
        for (k=0; k < NB; k++) {
          dcrm(v[i],vJ,dvdq[i].col(k),VectorXd::Zero(6),&(dcross));
          davpdq[i].col(k) += dcross;
        }
        
        dvJdqd_mat = MatrixXd::Zero(6,NB);
        dvJdqd_mat.col(n) = dvJdqd;
        dcrm(v[i],vJ,dvdqd[i],dvJdqd_mat,&(dcross));
        davpdqd[i] = Xup[i]*davpdqd[j] + dcross;
      }
      
      dcrf(v[i],I[i]*v[i],dvdq[i],I[i]*dvdq[i],&(dcross));
      dfvpdq[i] = I[i]*davpdq[i] + dcross;
      dcrf(v[i],I[i]*v[i],dvdqd[i],I[i]*dvdqd[i],&(dcross));
      dfvpdqd[i] = I[i]*davpdqd[i] + dcross;
      
    }
  }
  
  for (i=(NB-1); i>=0; i--) {
    n = dofnum[i];
    C(n) = (S[i]).transpose() * fvp[i] + damping[i]*qd[n];
    
    if (dC) {
      (*dC).block(n,0,1,NB) = S[i].transpose()*dfvpdq[i];
      (*dC).block(n,NB,1,NB) = S[i].transpose()*dfvpdqd[i];
      (*dC)(n,NB+n) += damping[i];
    }
    
    if (parent[i] >= 0) {
      fvp[parent[i]] += (Xup[i]).transpose()*fvp[i];
      IC[parent[i]] += (Xup[i]).transpose()*IC[i]*Xup[i];
      
      if (dH) {
        for (k=0; k < NB; k++) {
          dIC[parent[i]][k] += Xup[i].transpose()*dIC[i][k]*Xup[i];
        }
        dIC[parent[i]][n] += dXupdq[i].transpose()*IC[i]*Xup[i] + Xup[i].transpose()*IC[i]*dXupdq[i];
      }
      
      if (dC) {
        dfvpdq[parent[i]] += Xup[i].transpose()*dfvpdq[i];
        dfvpdq[parent[i]].col(n) += dXupdq[i].transpose()*fvp[i];
        dfvpdqd[parent[i]] += Xup[i].transpose()*dfvpdqd[i];
      }
    }
  }
  
  for (i=0; i<NB; i++) {
    n = dofnum[i];
    fh = IC[i] * S[i];
    H(n,n) = (S[i]).transpose() * fh;
    j=i;
    while (parent[j] >= 0) {
      fh = (Xup[j]).transpose() * fh;
      j = parent[j];
      np = dofnum[j];
      
      H(n,np) = (S[j]).transpose() * fh;
      H(np,n) = H(n,np);
    }
  }
  
  if (dH) {
    for (i=0; i < NB; i++) {
      n = dofnum[i];
      for (k=0; k < NB; k++) {
        fh = IC[i] * S[i];
        dfh = dIC[i][k] * S[i];  //dfh/dqk
        (*dH)(n + n*NB,k) = S[i].transpose() * dfh;
        j = i; np=n;
        while (parent[j] >= 0) {
          dfh = Xup[j].transpose() * dfh;
          if (np==k) {
            dfh += dXupdq[j].transpose() * fh;
          }
          fh = Xup[j].transpose() * fh;
          
          j = parent[j];
          np = dofnum[j];
          (*dH)(n + np*NB,k) = S[j].transpose() * dfh;
          (*dH)(np + n*NB,k) = (*dH)(n + np*NB,k);
        }
      }
    }
  }
}


// explicit instantiations (required for linking):
template void RigidBodyManipulator::getCOM(MatrixBase< Map<Vector3d> > &);
template void RigidBodyManipulator::getCOMJac(MatrixBase< Map<MatrixXd> > &);
template void RigidBodyManipulator::getCOMdJac(MatrixBase< Map<MatrixXd> > &);
template void RigidBodyManipulator::getCOMJacDot(MatrixBase< Map<MatrixXd> > &);
template void RigidBodyManipulator::getCOM(MatrixBase< Vector3d > &);
template void RigidBodyManipulator::getCOMJac(MatrixBase< MatrixXd > &);
template void RigidBodyManipulator::getCOMdJac(MatrixBase< MatrixXd > &);
template void RigidBodyManipulator::getCOMJacDot(MatrixBase< MatrixXd > &);

template void RigidBodyManipulator::getContactPositions(MatrixBase <MatrixXd > &, const std::set<int> &);
template void RigidBodyManipulator::getContactPositionsJac(MatrixBase <MatrixXd > &,const std::set<int> &);
template void RigidBodyManipulator::getContactPositionsJacDot(MatrixBase <MatrixXd > &,const std::set<int> &);

template void RigidBodyManipulator::forwardKin(const int, const MatrixBase< MatrixXd >&, const int, MatrixBase< Map<MatrixXd> > &);
template void RigidBodyManipulator::forwardJac(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< Map<MatrixXd> > &);
template void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< MatrixXd > &, MatrixBase< Map<MatrixXd> >&);
template void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< MatrixXd > &, MatrixBase< Map<MatrixXd> >&);
template void RigidBodyManipulator::forwardKin(const int, const MatrixBase< MatrixXd >&, const int, MatrixBase< MatrixXd > &);
template void RigidBodyManipulator::forwardJac(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< MatrixXd > &);
template void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd >&);
template void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd >&);
template void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Vector3d > &);
template void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Matrix<double,6,1> > &);
template void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Matrix<double,7,1> > &);
//template void RigidBodyManipulator::forwardKin(const int, const MatrixBase< Vector4d >&, const int, MatrixBase< Vector3d > &);
//template void RigidBodyManipulator::forwardJac(const int, const MatrixBase< Vector4d > &, const int, MatrixBase< MatrixXd > &);
//template void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< Vector4d > &, MatrixBase< MatrixXd >&);
//template void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< Vector4d > &, MatrixBase< MatrixXd >&);
template void RigidBodyManipulator::bodyKin(const int, const MatrixBase< MatrixXd >&, MatrixBase< Map<MatrixXd> > &);

template void RigidBodyManipulator::HandC(double* const, double * const, MatrixBase< Map<MatrixXd> > * const, MatrixBase< Map<MatrixXd> > &, MatrixBase< Map<VectorXd> > &, MatrixBase< Map<MatrixXd> > *, MatrixBase< Map<MatrixXd> > *);
template void RigidBodyManipulator::HandC(double* const, double * const, MatrixBase< MatrixXd > * const, MatrixBase< MatrixXd > &, MatrixBase< VectorXd > &, MatrixBase< MatrixXd > *, MatrixBase< MatrixXd > *);
