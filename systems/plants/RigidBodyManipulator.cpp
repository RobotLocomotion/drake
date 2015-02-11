#include <iostream>
#include <map>

//#include "mex.h"
#include "drakeGeometryUtil.h"
#include "RigidBodyManipulator.h"
#include "DrakeJoint.h"

#include <algorithm>
#include <string>
#include <regex>
#include <stdexcept>
#include <limits>

//DEBUG
//#include <stdexcept>
//END_DEBUG

using namespace std;

std::set<int> emptyIntSet;

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




RigidBodyManipulator::RigidBodyManipulator(int ndof, int num_featherstone_bodies, int num_rigid_body_objects, int num_rigid_body_frames)
  :  collision_model(DrakeCollision::newModel()), collision_model_no_margins(DrakeCollision::newModel())
{
  use_new_kinsol = false;
  num_dof=0; NB=0; num_bodies=0; num_frames=0;
  a_grav = VectorXd::Zero(6);
  resize(ndof,num_featherstone_bodies,num_rigid_body_objects,num_rigid_body_frames);
}

RigidBodyManipulator::~RigidBodyManipulator(void)
{
  //  if (collision_model)
  //    delete collision_model;
}


void RigidBodyManipulator::resize(int ndof, int num_featherstone_bodies, int num_rigid_body_objects, int num_rigid_body_frames)
{
  int last_num_dof = num_dof, last_NB = NB, last_num_bodies = num_bodies;

  num_dof = ndof;
  joint_limit_min.conservativeResize(num_dof);
  joint_limit_max.conservativeResize(num_dof);
  for (int i=last_num_dof; i<num_dof; i++) {
    joint_limit_min[i] = -std::numeric_limits<double>::infinity(); 
    joint_limit_max[i] = std::numeric_limits<double>::infinity(); 
  }

  if (num_featherstone_bodies<0)
    NB = ndof;
  else
    NB = num_featherstone_bodies;

  pitch.conservativeResize(NB);
  parent.conservativeResize(NB);
  dofnum.conservativeResize(NB);
  damping.conservativeResize(NB);
  coulomb_friction.conservativeResize(NB);
  static_friction.conservativeResize(NB);
  coulomb_window.conservativeResize(NB);

  // note: these are std:vectors (and the above are eigen vectors)
  Xtree.resize(NB);
  I.resize(NB);
  S.resize(NB);
  Xup.resize(NB);
  v.resize(NB);
  avp.resize(NB);
  fvp.resize(NB);
  IC.resize(NB);
  for(int i=last_NB; i < NB; i++) {
    Xtree[i] = MatrixXd::Zero(6,6);
    I[i] = MatrixXd::Zero(6,6);
    S[i] = VectorXd::Zero(6);
    Xup[i] = MatrixXd::Zero(6,6);
    v[i] = VectorXd::Zero(6);
    avp[i] = VectorXd::Zero(6);
    fvp[i] = VectorXd::Zero(6);
    IC[i] = MatrixXd::Zero(6,6);
  }

  if (num_rigid_body_objects<0)
    num_bodies = NB+1;  // this was my old assumption, so leave it here as the default behavior
  else
    num_bodies = num_rigid_body_objects;

  Ic_new.resize(num_bodies);
  for (int i = 0; i < num_bodies; i++) {
    Ic_new[i] = Matrix<double, TWIST_SIZE, TWIST_SIZE>::Zero();
  }

  bodies.reserve(num_bodies);
  for (int i = last_num_bodies; i < num_bodies; i++) {
    bodies.push_back(std::unique_ptr<RigidBody>(new RigidBody()));
//    bodies[i]->dofnum = i - 1;
  } // setup default dofnums

  for (int i = 0; i < num_bodies; i++) {
    bodies[i]->setN(NB);
  }

  collision_model->resize(num_bodies);
  collision_model_no_margins->resize(num_bodies);

  num_frames = num_rigid_body_frames;
  frames.resize(num_frames);

  //Variable allocation for gradient calculations
  dXupdq.resize(NB);
  dIC.resize(NB);
  for(int i=0; i < NB; i++) {
    dIC[i].resize(NB);
    for(int j=0; j < NB; j++) {
      dIC[i][j] = MatrixXd::Zero(6,6);
    }
  }

  dIc_new.resize(num_bodies);
  for (int i = 0; i < num_bodies; i++)
    dIc_new[i] = MatrixXd::Zero(Ic_new[i].size(), num_dof);

  // don't need to resize dcross (it gets resized in dcrm)

  dvdq.resize(NB);
  dvdqd.resize(NB);
  davpdq.resize(NB);
  davpdqd.resize(NB);
  dfvpdq.resize(NB);
  dfvpdqd.resize(NB);

  dvJdqd_mat = MatrixXd::Zero(6,num_dof);
  for(int i=0; i < NB; i++) {
    dvdq[i] = MatrixXd::Zero(6,num_dof);
    dvdqd[i] = MatrixXd::Zero(6,num_dof);
    davpdq[i] = MatrixXd::Zero(6,num_dof);
    davpdqd[i] = MatrixXd::Zero(6,num_dof);
    dfvpdq[i] = MatrixXd::Zero(6,num_dof);
    dfvpdqd[i] = MatrixXd::Zero(6,num_dof);
  }

  // preallocate for COM functions
  bc = Vector3d::Zero();
  bJ = MatrixXd::Zero(3,num_dof);
  bdJ = MatrixXd::Zero(3,num_dof*num_dof);
  dTdTmult = MatrixXd::Zero(3*num_dof,4);

  // preallocate for CMM function
  Ic.resize(NB); // composite rigid body inertias
  dIc.resize(NB); // derivative of composite rigid body inertias
  phi.resize(NB); // joint axis vectors
  Xworld.resize(NB); // spatial transforms from world to each body
  dXworld.resize(NB); // dXworld_dq * qd
  dXup.resize(NB); // dXup_dq * qd
  for(int i=0; i < NB; i++) {
    Ic[i] = MatrixXd::Zero(6,6);
    dIc[i] = MatrixXd::Zero(6,6);
    phi[i] = VectorXd::Zero(6);
    Xworld[i] = MatrixXd::Zero(6,6);
    dXworld[i] = MatrixXd::Zero(6,6);
    dXup[i] = MatrixXd::Zero(6,6);
  }

  Xg = MatrixXd::Zero(6,6); // spatial centroidal projection matrix for a single body
  dXg = MatrixXd::Zero(6,6);  // dXg_dq * qd
  Xcom = MatrixXd::Zero(6,6); // spatial transform from centroid to world
  Jcom = MatrixXd::Zero(3,num_dof);
  dXcom = MatrixXd::Zero(6,6);
  Xi = MatrixXd::Zero(6,6);
  dXidq = MatrixXd::Zero(6,6);

  initialized = false;
  kinematicsInit = false;
  cached_q.resize(num_dof);
  cached_qd.resize(num_dof);
  secondDerivativesCached = 0;
}


void RigidBodyManipulator::compile(void)
{
  for (int i=0; i<num_bodies; i++) {
    // precompute sparsity pattern for each rigid body
    bodies[i]->computeAncestorDOFs(this);
  }

  initialized=true;
}

void RigidBodyManipulator::addCollisionElement(const int body_ind, const Matrix4d & T_elem_to_lnk, DrakeCollision::Shape shape, vector<double> params, string group_name)
{
  bool is_static = (bodies[body_ind]->parent==0);
  collision_model->addElement(body_ind,bodies[body_ind]->parent,T_elem_to_lnk,shape,params,group_name,is_static, true);
  collision_model_no_margins->addElement(body_ind,bodies[body_ind]->parent,T_elem_to_lnk,shape,params,group_name,is_static, false);
}

void RigidBodyManipulator::updateCollisionElements(const int body_ind)
{
  collision_model->updateElementsForBody(body_ind, bodies[body_ind]->T);
  collision_model_no_margins->updateElementsForBody(body_ind, bodies[body_ind]->T);
};

bool RigidBodyManipulator::setCollisionFilter(const int body_ind,
                                              const uint16_t group,
                                              const uint16_t mask)
{
  //DEBUG
  //cout << "RigidBodyManipulator::setCollisionFilter: Group: " << group << endl;
  //cout << "RigidBodyManipulator::setCollisionFilter: Mask: " << mask << endl;
  //END_DEBUG
  bool status_one = collision_model->setCollisionFilter(body_ind,group,mask);
  bool status_two = collision_model_no_margins->setCollisionFilter(body_ind, group, mask);
  assert(status_one == status_two);
  return status_one && status_two;
}

bool RigidBodyManipulator::getPairwiseCollision(const int body_indA, const int body_indB, 
        MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals, bool use_margins)
{
  if (use_margins)
    return collision_model->getPairwiseCollision(body_indA,body_indB,ptsA,ptsB,normals);
  else
    return collision_model_no_margins->getPairwiseCollision(body_indA,body_indB,ptsA,ptsB,normals);
};

bool RigidBodyManipulator::getPairwisePointCollision(const int body_indA,
                                                     const int body_indB,
                                                     const int body_collision_indA,
                                                     Vector3d &ptA,
                                                     Vector3d &ptB,
                                                     Vector3d &normal, 
                                                     bool use_margins)
{
  if ( (use_margins && collision_model->getPairwisePointCollision(body_indA, body_indB,
          body_collision_indA,
          ptA,ptB,normal)) 
       ||
       (!use_margins && collision_model_no_margins->getPairwisePointCollision(body_indA, body_indB,
          body_collision_indA,
          ptA,ptB,normal))
     ) {
    return true;
  } else {
		ptA << 1,1,1;
		ptB << -1,-1,-1;
		normal << 0,0,1;
    return false;
  }
};

bool RigidBodyManipulator::getPointCollision(const int body_ind,
                                             const int body_collision_ind,
                                             Vector3d &ptA, Vector3d &ptB,
                                             Vector3d &normal, 
                                             bool use_margins)
{
  if ( (use_margins && 
          collision_model->getPointCollision(body_ind, body_collision_ind, ptA,ptB,
          normal))
       ||
       (!use_margins && 
          collision_model_no_margins->getPointCollision(body_ind, body_collision_ind, ptA,ptB,
          normal))
     ){
    return true;
  } else {
    ptA << 1,1,1;
    ptB << -1,-1,-1;
    normal << 0,0,1;
    return false;
  }
};

bool RigidBodyManipulator::getPairwiseClosestPoint(const int body_indA, 
                                             const int body_indB, 
                                             Vector3d &ptA, Vector3d &ptB,
                                             Vector3d &normal, 
                                             double &distance, 
                                             bool use_margins)
{
  if (use_margins)
    return collision_model->getClosestPoints(body_indA,body_indB,ptA,ptB,normal,distance);
  else
    return collision_model_no_margins->getClosestPoints(body_indA,body_indB,ptA,ptB,normal,distance);
};

bool RigidBodyManipulator::collisionRaycast(const Matrix3Xd &origins, 
                                            const Matrix3Xd &ray_endpoints, 
                                            VectorXd &distances, 
                                            bool use_margins )
{
  if (use_margins)
    return collision_model->collisionRaycast(origins, ray_endpoints, distances);
  else
    return collision_model_no_margins->collisionRaycast(origins, ray_endpoints, distances);
}

bool RigidBodyManipulator::collisionDetect( VectorXd& phi,
                                            MatrixXd& normal,
                                            MatrixXd& xA,
                                            MatrixXd& xB,
                                            vector<int>& bodyA_idx,
                                            vector<int>& bodyB_idx,
                                            const vector<int>& bodies_idx,
                                            const set<string>& active_element_groups,
                                            bool use_margins)
{
  if (use_margins)
    return collision_model->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,bodies_idx,active_element_groups);
  else
    return collision_model_no_margins->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,bodies_idx,active_element_groups);
}

bool RigidBodyManipulator::collisionDetect( VectorXd& phi,
                                            MatrixXd& normal, 
                                            MatrixXd& xA, 
                                            MatrixXd& xB,
                                            vector<int>& bodyA_idx, 
                                            vector<int>& bodyB_idx,
                                            const vector<int>& bodies_idx,
                                            bool use_margins)
{
  if (use_margins)
    return collision_model->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,bodies_idx);
  else
    return collision_model_no_margins->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,bodies_idx);
}

bool RigidBodyManipulator::collisionDetect( VectorXd& phi,
                                            MatrixXd& normal, 
                                            MatrixXd& xA, 
                                            MatrixXd& xB,
                                            vector<int>& bodyA_idx, 
                                            vector<int>& bodyB_idx,
                                            const set<string>& active_element_groups,
                                            bool use_margins)
{
  if (use_margins)
    return collision_model->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,active_element_groups);
  else
    return collision_model_no_margins->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,
      normal,phi,active_element_groups);
}

bool RigidBodyManipulator::collisionDetect( VectorXd& phi,
                                            MatrixXd& normal, 
                                            MatrixXd& xA, 
                                            MatrixXd& xB,
                                            vector<int>& bodyA_idx, 
                                            vector<int>& bodyB_idx,
                                            bool use_margins)
{
  if (use_margins)
    return collision_model->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,normal,phi);
  else
    return collision_model_no_margins->closestPointsAllBodies(bodyA_idx,bodyB_idx,xA,xB,normal,phi);
}

bool RigidBodyManipulator::allCollisions(vector<int>& bodyA_idx,
                                         vector<int>& bodyB_idx,
                                         MatrixXd& ptsA, MatrixXd& ptsB,
                                         bool use_margins)
{
  if (use_margins)
    return collision_model->allCollisions(bodyA_idx, bodyB_idx, ptsA, ptsB);
  else
    return collision_model_no_margins->allCollisions(bodyA_idx, bodyB_idx, ptsA, ptsB);
}


//bool RigidBodyManipulator::closestDistanceAllBodies(VectorXd& distance,
                                                        //MatrixXd& Jd)
//{
  //MatrixXd ptsA, ptsB, normal, JA, JB;
  //vector<int> bodyA_idx, bodyB_idx;
  //bool return_val = closestPointsAllBodies(bodyA_idx,bodyB_idx,ptsA,ptsB,normal,distance, JA,JB,Jd);
  //DEBUG
  //cout << "RigidBodyManipulator::closestDistanceAllBodies: distance.size() = " << distance.size() << endl;
  //END_DEBUG
  //return return_val;
//};

void RigidBodyManipulator::doKinematics(double* q, bool b_compute_second_derivatives, double* qd)
{
  if (use_new_kinsol) {
    doKinematicsNew(q, true, qd, true);
  }

  //DEBUG
  //try{
  //END_DEBUG
  int i,j,k,l;

  //DEBUG
  //cout << "RigidBodyManipulator::doKinematics: q = " << endl;
  //for (i = 0; i < num_dof; i++) {
    //cout << q[i] << endl;
  //}
  //END_DEBUG
  //Check against cached values for bodies[1];
  if (kinematicsInit) {
    bool skip = true;
    if (b_compute_second_derivatives && !secondDerivativesCached)
      skip = false;
    for (i = 0; i < num_dof; i++) {
      if (q[i] != cached_q[i] || q[i] != cached_q[i]) {
        skip = false;
        break;
      }
    }
    if (skip) {
      return;
    }
  }

  if (!initialized) { compile(); }

  Matrix4d TJ, dTJ, ddTJ, Tbinv, Tb, Tmult, dTmult, dTdotmult, TdTmult, TJdot, dTJdot, TddTmult;
  Matrix4d fb_dTJ[6], fb_dTJdot[6], fb_dTmult[6], fb_ddTJ[3][3];  // will be 7 when quats implemented...

  Matrix3d rx,drx,ddrx,ry,dry,ddry,rz,drz,ddrz;

  for (i = 0; i < num_bodies; i++) {
    int parent = bodies[i]->parent;
    if (parent < 0) {
      bodies[i]->T = bodies[i]->Ttree;
      //dTdq, ddTdqdq initialized as all zeros
    } else if (bodies[i]->floating == 1) {
      double qi[6];
      for (j=0; j<6; j++) qi[j] = q[bodies[i]->dofnum+j];

      rotx(qi[3],rx,drx,ddrx);
      roty(qi[4],ry,dry,ddry);
      rotz(qi[5],rz,drz,ddrz);

      Tb = bodies[i]->T_body_to_joint;
      Tbinv = Tb.inverse();

      TJ = Matrix4d::Identity();  TJ.block<3,3>(0,0) = rz*ry*rx;  TJ(0,3)=qi[0]; TJ(1,3)=qi[1]; TJ(2,3)=qi[2];

      Tmult = bodies[i]->Ttree * Tbinv * TJ * Tb;
      bodies[i]->T = bodies[parent]->T * Tmult;

      // see notes below
      bodies[i]->dTdq = bodies[parent]->dTdq * Tmult;
      dTmult = bodies[i]->Ttree * Tbinv * dTJ * Tb;
      TdTmult = bodies[parent]->T * dTmult;

      fb_dTJ[0] << 0,0,0,1, 0,0,0,0, 0,0,0,0, 0,0,0,0;
      fb_dTJ[1] << 0,0,0,0, 0,0,0,1, 0,0,0,0, 0,0,0,0;
      fb_dTJ[2] << 0,0,0,0, 0,0,0,0, 0,0,0,1, 0,0,0,0;
      fb_dTJ[3] = Matrix4d::Zero(); fb_dTJ[3].block<3,3>(0,0) = rz*ry*drx;
      fb_dTJ[4] = Matrix4d::Zero(); fb_dTJ[4].block<3,3>(0,0) = rz*dry*rx;
      fb_dTJ[5] = Matrix4d::Zero(); fb_dTJ[5].block<3,3>(0,0) = drz*ry*rx;

      for (j=0; j<6; j++) {
        fb_dTmult[j] = bodies[i]->Ttree * Tbinv * fb_dTJ[j] * Tb;
        TdTmult = bodies[parent]->T * fb_dTmult[j];
        bodies[i]->dTdq.row(bodies[i]->dofnum + j) += TdTmult.row(0);
        bodies[i]->dTdq.row(bodies[i]->dofnum + j + num_dof) += TdTmult.row(1);
        bodies[i]->dTdq.row(bodies[i]->dofnum + j + 2*num_dof) += TdTmult.row(2);
      }

      if (b_compute_second_derivatives) {
        fb_ddTJ[0][0] << rz*ry*ddrx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[0][1] << rz*dry*drx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[0][2] << drz*ry*drx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[1][0] << rz*dry*drx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[1][1] << rz*ddry*rx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[1][2] << drz*dry*rx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[2][0] << drz*ry*drx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[2][1] << drz*dry*rx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);
        fb_ddTJ[2][2] << ddrz*ry*rx, MatrixXd::Zero(3,1), MatrixXd::Zero(1,4);

        // ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        bodies[i]->ddTdqdq = MatrixXd::Zero(3*num_dof*num_dof,4);  // note: could be faster if I skipped this (like I do for floating == 0 below)

        //        bodies[i]->ddTdqdq = bodies[parent]->ddTdqdq * Tmult;
        for (set<IndexRange>::iterator iter = bodies[parent]->ddTdqdq_nonzero_rows_grouped.begin(); iter != bodies[parent]->ddTdqdq_nonzero_rows_grouped.end(); iter++) {
          bodies[i]->ddTdqdq.block(iter->start,0,iter->length,4) = bodies[parent]->ddTdqdq.block(iter->start,0,iter->length,4) * Tmult;
        }

        for (j=0; j<6; j++) {
          dTmult = bodies[i]->Ttree * Tbinv * fb_dTJ[j] * Tb;
          dTdTmult = bodies[parent]->dTdq * dTmult;
          for (k=0; k<3*num_dof; k++) {
            bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum+j) + k) += dTdTmult.row(k);
          }

          for (l=0; l<3; l++) {
            for (k=0;k<num_dof;k++) {
              if (k>=bodies[i]->dofnum && k<=bodies[i]->dofnum+j) {
                bodies[i]->ddTdqdq.row(bodies[i]->dofnum+j + (3*k+l)*num_dof) += dTdTmult.row(l*num_dof+k);
              } else {
                bodies[i]->ddTdqdq.row(bodies[i]->dofnum+j + (3*k+l)*num_dof) += dTdTmult.row(l*num_dof+k);
              }
            }
          }

          if (j>=3) {
          	for (k=3; k<6; k++) {
              TddTmult = bodies[parent]->T*bodies[i]->Ttree * Tbinv * fb_ddTJ[j-3][k-3] * Tb;
              bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum+k) + bodies[i]->dofnum+j) += TddTmult.row(0);
              bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum+k) + bodies[i]->dofnum+j + num_dof) += TddTmult.row(1);
              bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum+k) + bodies[i]->dofnum+j + 2*num_dof) += TddTmult.row(2);
          	}
          }
        }
      }
      if (qd) {
        double qdi[6];

        TJdot = Matrix4d::Zero();
        for (int j=0; j<6; j++) {
          qdi[j] = qd[bodies[i]->dofnum+j];
          TJdot += fb_dTJ[j]*qdi[j];
        }

        fb_dTJdot[0] = Matrix4d::Zero();
        fb_dTJdot[1] = Matrix4d::Zero();
        fb_dTJdot[2] = Matrix4d::Zero();
        fb_dTJdot[3] = Matrix4d::Zero();  fb_dTJdot[3].block<3,3>(0,0) = (drz*qdi[5])*ry*drx + rz*(dry*qdi[4])*drx + rz*ry*(ddrx*qdi[3]);
        fb_dTJdot[4] = Matrix4d::Zero();  fb_dTJdot[4].block<3,3>(0,0) = (drz*qdi[5])*dry*rx + rz*(ddry*qdi[4])*rx + rz*dry*(drx*qdi[3]);
        fb_dTJdot[5] = Matrix4d::Zero();  fb_dTJdot[5].block<3,3>(0,0) = (ddrz*qdi[5])*ry*rx + drz*(dry*qdi[4])*rx + drz*ry*(drx*qdi[3]);

        dTdotmult = bodies[i]->Ttree * Tbinv * TJdot * Tb;
        bodies[i]->Tdot = bodies[parent]->Tdot*Tmult + bodies[parent]->T * dTdotmult;

        bodies[i]->dTdqdot = bodies[parent]->dTdqdot* Tmult + bodies[parent]->dTdq * dTdotmult;

        for (int j=0; j<6; j++) {
          dTdotmult = bodies[parent]->Tdot*fb_dTmult[j] + bodies[parent]->T*bodies[i]->Ttree*Tbinv*fb_dTJdot[j]*Tb;
          bodies[i]->dTdqdot.row(bodies[i]->dofnum + j) += dTdotmult.row(0);
          bodies[i]->dTdqdot.row(bodies[i]->dofnum + j + num_dof) += dTdotmult.row(1);
          bodies[i]->dTdqdot.row(bodies[i]->dofnum + j + 2*num_dof) += dTdotmult.row(2);
        }
      }

    } else if (bodies[i]->floating == 2) {
      cerr << "mex kinematics for quaternion floating bases are not implemented yet" << endl;
    } else {
      double qi = q[bodies[i]->dofnum];
      Tjcalc(bodies[i]->pitch,qi,&TJ);
      dTjcalc(bodies[i]->pitch,qi,&dTJ);

      Tb = bodies[i]->T_body_to_joint;
      Tbinv = Tb.inverse();

      Tmult = bodies[i]->Ttree * Tbinv * TJ * Tb;

      bodies[i]->T = bodies[parent]->T * Tmult;

      /*
       * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
       * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
       */

      bodies[i]->dTdq = bodies[parent]->dTdq * Tmult;  // note: could only compute non-zero entries here

      dTmult = bodies[i]->Ttree * Tbinv * dTJ * Tb;
      TdTmult = bodies[parent]->T * dTmult;
      bodies[i]->dTdq.row(bodies[i]->dofnum) += TdTmult.row(0);
      bodies[i]->dTdq.row(bodies[i]->dofnum + num_dof) += TdTmult.row(1);
      bodies[i]->dTdq.row(bodies[i]->dofnum + 2*num_dof) += TdTmult.row(2);

      if (b_compute_second_derivatives) {
        //ddTdqdq = [d(dTdq)dq1; d(dTdq)dq2; ...]
        //	bodies[i]->ddTdqdq = bodies[parent]->ddTdqdq * Tmult; // pushed this into the loop below to exploit the sparsity
        for (set<IndexRange>::iterator iter = bodies[parent]->ddTdqdq_nonzero_rows_grouped.begin(); iter != bodies[parent]->ddTdqdq_nonzero_rows_grouped.end(); iter++) {
          bodies[i]->ddTdqdq.block(iter->start,0,iter->length,4) = bodies[parent]->ddTdqdq.block(iter->start,0,iter->length,4) * Tmult;
        }

        dTdTmult = bodies[parent]->dTdq * dTmult;
        for (j = 0; j < 3*num_dof; j++) {
          bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum) + j) = dTdTmult.row(j);
        }

        // ind = reshape(reshape(body.dofnum+0:num_dof:3*num_dof*num_dof,3,[])',[],1); % ddTdqidq
        for (j = 0; j < 3; j++) {
          for (k = 0; k < num_dof; k++) {
            if (k == bodies[i]->dofnum) {
              bodies[i]->ddTdqdq.row(bodies[i]->dofnum + (3*k+j)*num_dof) += dTdTmult.row(j*num_dof+k);
            } else {
              bodies[i]->ddTdqdq.row(bodies[i]->dofnum + (3*k+j)*num_dof) = dTdTmult.row(j*num_dof+k);
            }
          }
        }

        ddTjcalc(bodies[i]->pitch,qi,&ddTJ);
        TddTmult = bodies[parent]->T*bodies[i]->Ttree * Tbinv * ddTJ * Tb;

        bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum) + bodies[i]->dofnum) += TddTmult.row(0);
        bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum) + bodies[i]->dofnum + num_dof) += TddTmult.row(1);
        bodies[i]->ddTdqdq.row(3*num_dof*(bodies[i]->dofnum) + bodies[i]->dofnum + 2*num_dof) += TddTmult.row(2);
      }

      if (qd) {
        double qdi = qd[bodies[i]->dofnum];
        TJdot = dTJ*qdi;
        ddTjcalc(bodies[i]->pitch,qi,&ddTJ);
        dTJdot = ddTJ*qdi;

//        body.Tdot = body.parent.Tdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.T*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        dTdotmult = bodies[i]->Ttree * Tbinv * TJdot * Tb;
        bodies[i]->Tdot = bodies[parent]->Tdot*Tmult + bodies[parent]->T * dTdotmult;
//        body.dTdqdot = body.parent.dTdqdot*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint + body.parent.dTdq*body.Ttree*inv(body.T_body_to_joint)*TJdot*body.T_body_to_joint;
        bodies[i]->dTdqdot = bodies[parent]->dTdqdot* Tmult + bodies[parent]->dTdq * dTdotmult;

//        body.dTdqdot(this_dof_ind,:) = body.dTdqdot(this_dof_ind,:) + body.parent.Tdot(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJ*body.T_body_to_joint + body.parent.T(1:3,:)*body.Ttree*inv(body.T_body_to_joint)*dTJdot*body.T_body_to_joint;
        dTdotmult = bodies[parent]->Tdot*dTmult + bodies[parent]->T*bodies[i]->Ttree*Tbinv*dTJdot*Tb;
        bodies[i]->dTdqdot.row(bodies[i]->dofnum) += dTdotmult.row(0);
        bodies[i]->dTdqdot.row(bodies[i]->dofnum + num_dof) += dTdotmult.row(1);
        bodies[i]->dTdqdot.row(bodies[i]->dofnum + 2*num_dof) += dTdotmult.row(2);
      }
    }

    if (bodies[i]->parent>=0) {
      //DEBUG
      //cout << "RigidBodyManipulator::doKinematics: updating body " << i << " ..." << endl;
      //END_DEBUG
      collision_model->updateElementsForBody(i,bodies[i]->T);
      collision_model_no_margins->updateElementsForBody(i,bodies[i]->T);
      //DEBUG
      //cout << "RigidBodyManipulator::doKinematics: done updating body " << i << endl;
      //END_DEBUG
    }
  }

  kinematicsInit = true;
  for (i = 0; i < num_dof; i++) {
    cached_q[i] = q[i];
    if (qd) cached_qd[i] = qd[i];
  }
  secondDerivativesCached = b_compute_second_derivatives;
  //DEBUG
  //} catch (const out_of_range& oor) {
    //string msg("In RigidBodyManipulator::doKinematics:\n");
    //throw std::out_of_range(msg + oor.what());
  //}
  //END_DEBUG
}

void RigidBodyManipulator::doKinematicsNew(double* q, bool compute_gradients, double* v, bool compute_JdotV) {
  int nq = num_dof;
  int nv = num_dof; // FIXME

  // other bodies
  for (int i = 0; i < bodies.size(); i++) {
    RigidBody& body = *bodies[i];

    if (body.hasParent()) {
      double* q_body = &q[body.dofnum];

      // transform
      Isometry3d T_body_to_parent = Isometry3d(body.Ttree) * body.getJoint().jointTransform(q_body);
      body.T_new = bodies[body.parent]->T_new * T_body_to_parent;

      // motion subspace in body frame
      Eigen::MatrixXd* dSdq = compute_gradients ? &(body.dSdqi) : nullptr;
      body.getJoint().motionSubspace(q_body, body.S, dSdq);

      // motion subspace in world frame
      body.J = transformSpatialMotion(body.T_new, body.S);

      // qdot to v
      Eigen::MatrixXd* dqdot_to_v = compute_gradients ? &(body.dqdot_to_v_dqi) : nullptr;
      body.getJoint().qdot2v(q, body.qdot_to_v, dqdot_to_v);
      if (compute_gradients) {
        body.dqdot_to_v_dq.setZero();
        body.dqdot_to_v_dq.middleCols(body.dofnum, body.getJoint().getNumPositions()) = body.dqdot_to_v_dqi;
      }

      // v to qdot
      Eigen::MatrixXd* dv_to_qdot = compute_gradients ? &(body.dv_to_qdot_dqi) : nullptr;
      body.getJoint().v2qdot(q, body.v_to_qdot, dv_to_qdot);
      if (compute_gradients) {
        body.dv_to_qdot_dq.setZero();
        body.dv_to_qdot_dq.middleCols(body.dofnum, body.getJoint().getNumPositions()) = body.dv_to_qdot_dqi;
      }

      if (compute_gradients) {
        // gradient of transform
        auto dT_body_to_parentdqi = dHomogTrans(T_body_to_parent, body.S, body.qdot_to_v).eval();
        Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type dT_body_to_parentdq(HOMOGENEOUS_TRANSFORM_SIZE, nq);
        dT_body_to_parentdq.setZero();
        dT_body_to_parentdq.middleCols(body.dofnum, body.getJoint().getNumPositions()) = dT_body_to_parentdqi;
        body.dTdq_new = matGradMultMat(bodies[body.parent]->T_new.matrix(), T_body_to_parent.matrix(), bodies[body.parent]->dTdq_new, dT_body_to_parentdq);

        // gradient of motion subspace in world
        MatrixXd dSdq = MatrixXd::Zero(body.S.size(), nq);
        dSdq.middleCols(body.dofnum, body.getJoint().getNumPositions()) = body.dSdqi;
        body.dJdq = dTransformSpatialMotion(body.T_new, body.S, body.dTdq_new, dSdq);
      }

      if (v) {
        // twist
        double* v_body = &v[body.dofnum]; // FIXME: using dofnum for velocity...
        Map<VectorXd> v_body_map(v_body, body.getJoint().getNumVelocities());
        typedef Matrix<double, TWIST_SIZE, 1> Vector6d;
        Vector6d joint_twist = body.J * v_body_map;
        body.twist = bodies[body.parent]->twist + joint_twist;

        Gradient<Vector6d, Eigen::Dynamic>::type djoint_twistdq(TWIST_SIZE, nq);
        if (compute_gradients) {
          djoint_twistdq = matGradMult(body.dJdq, v_body_map);
          // dtwistdq
          body.dtwistdq = bodies[body.parent]->dtwistdq + djoint_twistdq;
        }

        if (compute_JdotV) {
          // Sdotv
          auto dSdotVdqi = compute_gradients ? &body.dSdotVdqi : nullptr;
          auto dSdotVdvi = compute_gradients ? &body.dSdotVdvi : nullptr;
          body.getJoint().motionSubspaceDotTimesV(q_body, v_body, body.SdotV, dSdotVdqi, dSdotVdvi);

          // Jdotv
          auto joint_accel = (crm(body.twist) * joint_twist + transformSpatialMotion(body.T_new, body.SdotV)).eval();
          body.JdotV = bodies[body.parent]->JdotV + joint_accel;

          if (compute_gradients) {
            // dJdotvdq
            // TODO: exploit sparsity better
            Matrix<double, TWIST_SIZE, Eigen::Dynamic> dSdotVdq(TWIST_SIZE, nq);
            dSdotVdq.setZero();
            dSdotVdq.middleCols(body.dofnum, body.getJoint().getNumPositions()) = body.dSdotVdqi;
            MatrixXd dcrm_twist_joint_twistdq(TWIST_SIZE, nq);
            dcrm(body.twist, joint_twist, body.dtwistdq, djoint_twistdq, &dcrm_twist_joint_twistdq); // TODO: make dcrm templated
            body.dJdotVdq = bodies[body.parent]->dJdotVdq
                + dcrm_twist_joint_twistdq
                + dTransformSpatialMotion(body.T_new, body.SdotV, body.dTdq_new, dSdotVdq);

            // dJdotvdv
            // TODO: exploit sparsity better
            std::vector<int> v_indices;
            auto jacobian = geometricJacobian<double>(0, i, 0, 0, false, &v_indices);

            Matrix<double, TWIST_SIZE, Eigen::Dynamic> dtwistdv(TWIST_SIZE, nv);
            dtwistdv.setZero();
            for (int j = 0; j < v_indices.size(); j++) {
              dtwistdv.col(v_indices[j]) = jacobian.value().col(j);
            }
            Matrix<double, TWIST_SIZE, Eigen::Dynamic> djoint_twistdv(TWIST_SIZE, nv);
            djoint_twistdv.setZero();
            djoint_twistdv.middleCols(body.dofnum, body.getJoint().getNumVelocities()) = body.S; // FIXME: using dofnum for velocity...

            MatrixXd djoint_acceldv(TWIST_SIZE, nv);
            dcrm(body.twist, joint_twist, dtwistdv, djoint_twistdv, &djoint_acceldv);
            Matrix<double, TWIST_SIZE, Eigen::Dynamic> dSdotVdv(TWIST_SIZE, nv);
            dSdotVdv.setZero();
            dSdotVdv.middleCols(body.dofnum, body.getJoint().getNumVelocities()) = *dSdotVdvi; // FIXME: using dofnum for velocity...
            djoint_acceldv += transformSpatialMotion(body.T_new, dSdotVdv);
            body.dJdotVdv = bodies[body.parent]->dJdotVdv + djoint_acceldv;
          }
        }
      }
    }
    else {
      body.T_new = Isometry3d(body.Ttree);
      // motion subspace in body frame is empty
      // motion subspace in world frame is empty
      // qdot to v is empty
      // v to qdot is empty
      if (compute_gradients) {
        // gradient of transform
        body.dTdq_new.setZero();
        // gradient of motion subspace in world is empty
      }
      if (v) {
        body.twist.setZero();
        if (compute_gradients) {
          body.dtwistdq.setZero();
        }
        if (compute_JdotV) {
          body.SdotV.setZero();
          if (compute_gradients) {
            body.dSdotVdqi.setZero();
            body.dSdotVdvi.setZero();
          }
          body.JdotV.setZero();
          if (compute_gradients) {
            body.dJdotVdq.setZero();
            body.dJdotVdv.setZero();
          }
        }
      }
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCMM(double* const q, double* const qd, MatrixBase<Derived> &A, MatrixBase<Derived> &Adot)
{
  // returns the centroidal momentum matrix as described in Orin & Goswami 2008
  //
  // h = A*qd, where h(4:6) is the total linear momentum and h(1:3) is the
  // total angular momentum in the centroid frame (world fram translated to COM).

  Vector3d com; getCOM(com);
  Xtrans(-com,&Xcom);

  getCOMJac(Jcom);
  Map<VectorXd> qdvec(qd,num_dof);
  Vector3d com_dot = Jcom*qdvec;
  dXcom = MatrixXd::Zero(6,6);
  dXcom(5,1) = 1*com_dot(0);
  dXcom(4,2) = -1*com_dot(0);
  dXcom(5,0) = -1*com_dot(1);
  dXcom(3,2) = 1*com_dot(1);
  dXcom(4,0) = 1*com_dot(2);
  dXcom(3,1) = -1*com_dot(2);

  A = MatrixXd::Zero(6,num_dof);
  Adot = MatrixXd::Zero(6,num_dof);

  for (int i=0; i < NB; i++) {
    Ic[i] = I[i];
    dIc[i] = MatrixXd::Zero(6,6);
  }

  int n;
  for (int i=NB-1; i >= 0; i--) {
    n = dofnum[i];
    jcalc(pitch[i],q[n],&Xi,&phi[i]);
    Xup[i] = Xi * Xtree[i];

    djcalc(pitch[i], q[n], &dXidq);
    dXup[i] = dXidq * Xtree[i] * qd[n];

    if (parent[i] >= 0) {
      Ic[parent[i]] += Xup[i].transpose()*Ic[i]*Xup[i];
      dIc[parent[i]] += (dXup[i].transpose()*Ic[i] + Xup[i].transpose()*dIc[i])*Xup[i] + Xup[i].transpose()*Ic[i]*dXup[i];
    }
  }


  for (int i=0; i < NB; i++) {
    if (parent[i] >= 0) {
      Xworld[i] = Xup[i] * Xworld[parent[i]];
      dXworld[i] = dXup[i]*Xworld[parent[i]] + Xup[i]*dXworld[parent[i]];
    }
    else {
      Xworld[i] = Xup[i];
      dXworld[i] = dXup[i];
    }

    Xg = Xworld[i] * Xcom; // spatial transform from centroid to body
    dXg = dXworld[i] * Xcom + Xworld[i] * dXcom;

    n = dofnum[i];
    A.col(n) = Xg.transpose()*Ic[i]*phi[i];
    Adot.col(n) = dXg.transpose()*Ic[i]*phi[i] + Xg.transpose()*dIc[i]*phi[i];
  }
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::centroidalMomentumMatrix(int gradient_order)
{
  if (!use_new_kinsol)
    throw std::runtime_error("method requires new kinsol format");

  if (gradient_order > 1)
    throw std::runtime_error("only first order gradient is available");

  int nq = num_dof;
  for (int i = 0; i < num_bodies; i++) {
    Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type* dTdq = nullptr;
    if (gradient_order > 0)
      dTdq = &(bodies[i]->dTdq_new);

    auto inertia_world = transformSpatialInertia(bodies[i]->T_new, dTdq, bodies[i]->I);
    Ic_new[i] = inertia_world.value();
    if (inertia_world.hasGradient()) {
      dIc_new[i] = inertia_world.gradient().value();
    }
  }

  for (int i = num_bodies - 1; i >= 0; i--) {
    if (bodies[i]->hasParent()) {
      Ic_new[bodies[i]->parent] += Ic_new[i];
      if (gradient_order > 0) {
        dIc_new[bodies[i]->parent] += dIc_new[i];
      }
    }
  }

  auto com = centerOfMass<Scalar>(gradient_order);
  int nv = num_velocities;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, nv, nq, gradient_order);
  int gradient_row_start = 0;
  for (int i = 0; i < num_bodies; i++) {
    RigidBody& body = *bodies[i];
    if (body.hasParent()) {
      int nv_joint = body.getJoint().getNumVelocities();

      ret.value().middleCols(body.velocity_num_start, nv_joint).noalias() = Ic_new[i] * body.J;

      if (gradient_order > 0) {
        ret.gradient().value().middleRows(gradient_row_start, TWIST_SIZE * nv_joint) = matGradMultMat(Ic_new[i], body.J, dIc_new[i], body.dJdq);
        gradient_row_start += TWIST_SIZE * nv_joint;
      }
    }
  }

  // TODO: could exploit structure of T better in this part
  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> T(Translation<Scalar, SPACE_DIMENSION>(-com.value()));

  if (gradient_order > 0) {
    Eigen::Matrix<double, HOMOGENEOUS_TRANSFORM_SIZE, Eigen::Dynamic> dtransform_world_to_com(HOMOGENEOUS_TRANSFORM_SIZE, nq);
    dtransform_world_to_com.setZero();
    setSubMatrixGradient<Eigen::Dynamic>(dtransform_world_to_com, (-com.gradient().value()).eval(), intRange<3>(0), intRange<1>(3), SPACE_DIMENSION + 1);
    ret.gradient().value() = dTransformSpatialForce(T, ret.value(), dtransform_world_to_com, ret.gradient().value());
  }
  ret.value() = transformSpatialForce(T, ret.value());
  return ret;
}

template <typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMass(int gradient_order, const std::set<int>& robotnum)
{
  if (!use_new_kinsol)
    throw std::runtime_error("method requires new kinsol format");

  int nq = num_dof;
  GradientVar<Scalar, SPACE_DIMENSION, 1> com(SPACE_DIMENSION, 1, nq, gradient_order);
  double m = 0.0;
  double body_mass;
  com.value().setZero();
  if (gradient_order > 0)
    com.gradient().value().setZero();
  if (gradient_order > 1)
    com.gradient().gradient().value().setZero();

  for (int i = 0; i < num_bodies; i++) {
    std::set<int>::iterator robotnum_it = robotnum.find(bodies[i]->robotnum);
    if (robotnum_it != robotnum.end())
    {
      body_mass = bodies[i]->mass;
      if (body_mass > 0) {
        Vector3d body_com_body_frame = (bodies[i]->com.topRows<SPACE_DIMENSION>());
        auto body_com = forwardKinNew(body_com_body_frame, i, 0, 0, gradient_order);
        com.value() *= m;
        com.value().noalias() += body_mass * body_com.value();
        com.value() /= (m + body_mass);

        if (gradient_order > 0) {
          com.gradient().value() *= m;
          com.gradient().value().noalias() += body_mass * body_com.gradient().value();
          com.gradient().value() /= (m + body_mass);
        }

        if (gradient_order > 1) {
          com.gradient().gradient().value() *= m;
          com.gradient().gradient().value().noalias() += body_mass * body_com.gradient().gradient().value();
          com.gradient().gradient().value() /= (m + body_mass);
        }

        m += body_mass;
      }
    }
  }
  return com;
}

template <typename Derived>
void RigidBodyManipulator::getCOM(MatrixBase<Derived> &com, const std::set<int> &robotnum)
{
  double m = 0.0;
  double bm;
  com = Vector3d::Zero();

  for (int i=0; i<num_bodies; i++) {
    set<int>::iterator robotnum_it = robotnum.find(bodies[i]->robotnum);
    if(robotnum_it != robotnum.end())
    {
      bm = bodies[i]->mass;
      if (bm>0) {
        forwardKin(i,bodies[i]->com,0,bc);
        com = (m*com + bm*bc)/(m+bm);
        m = m+bm;
      }
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMJac(MatrixBase<Derived> &Jcom, const std::set<int> &robotnum)
{
  double m = 0.0;
  double bm;
  Jcom = MatrixXd::Zero(3,num_dof);

  for (int i=0; i<num_bodies; i++) {
    set<int>::iterator robotnum_it = robotnum.find(bodies[i]->robotnum);
    if(robotnum_it != robotnum.end())
    {
      bm = bodies[i]->mass;
      if (bm>0) {
        forwardJac(i,bodies[i]->com,0,bJ);
        Jcom = (m*Jcom + bm*bJ)/(m+bm);
        m = m+bm;
      }
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMJacDot(MatrixBase<Derived> &Jcomdot, const std::set<int> &robotnum)
{
  double m = 0.0;
  double bm;
  Jcomdot = MatrixXd::Zero(3,num_dof);

  for (int i=0; i<num_bodies; i++) {
    set<int>::iterator robotnum_it = robotnum.find(bodies[i]->robotnum);
    if(robotnum_it != robotnum.end())
    {
      bm = bodies[i]->mass;
      if (bm>0) {
        forwardJacDot(i,bodies[i]->com,0,bJ);
        Jcomdot = (m*Jcomdot + bm*bJ)/(m+bm);
        m = m+bm;
      }
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getCOMdJac(MatrixBase<Derived> &dJcom, const std::set<int> &robotnum)
{
  double m = 0.0;
  double bm;
  dJcom = MatrixXd::Zero(3,num_dof*num_dof);

  for (int i=0; i<num_bodies; i++) {
    set<int>::iterator robotnum_it = robotnum.find(bodies[i]->robotnum);
    if(robotnum_it != robotnum.end())
    {
      bm = bodies[i]->mass;
      if (bm>0) {
        forwarddJac(i,bodies[i]->com,bdJ);
        dJcom = (m*dJcom + bm*bdJ)/(m+bm);
        m = m+bm;
      }
    }
  }
}

int RigidBodyManipulator::getNumContacts(const set<int> &body_idx)
{
  size_t n=0,nb=body_idx.size(),bi;
  if (nb==0) nb=num_bodies;
  set<int>::iterator iter = body_idx.begin();
  for (size_t i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    n += bodies[bi]->contact_pts.cols();
  }
  return static_cast<int>(n);
}


template <typename Derived>
void RigidBodyManipulator::getContactPositions(MatrixBase<Derived> &pos, const set<int> &body_idx)
{
  int n=0,nc,nb=static_cast<int>(body_idx.size()),bi;
  if (nb==0) nb=num_bodies;
  set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc>0) {
      // note: it's possible to pass pos.block in directly but requires such an ugly hack that I think it's not worth it:
      // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
      p.resize(3,nc);
      forwardKin(bi,bodies[bi]->contact_pts,0,p);
      pos.block(0,n,3,nc) = p;
      n += nc;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getContactPositionsJac(MatrixBase<Derived> &J, const set<int> &body_idx)
{
  int n=0,nc,nb=static_cast<int>(body_idx.size()),bi;
  if (nb==0) nb=num_bodies;
  set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc>0) {
      p.resize(3*nc,num_dof);
      forwardJac(bi,bodies[bi]->contact_pts,0,p);
      J.block(3*n,0,3*nc,num_dof) = p;
      n += nc;
    }
  }
}

template <typename Derived>
void RigidBodyManipulator::getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const set<int> &body_idx)
{
  int n=0,nc,nb=static_cast<int>(body_idx.size()),bi;
  if (nb==0) nb=num_bodies;
  set<int>::iterator iter = body_idx.begin();
  MatrixXd p;
  for (int i=0; i<nb; i++) {
    if (body_idx.size()==0) bi=i;
    else bi=*iter++;
    nc = static_cast<int>(bodies[bi]->contact_pts.cols());
    if (nc>0) {
      p.resize(3*nc,num_dof);
      forwardJacDot(bi,bodies[bi]->contact_pts,0,p);
      Jdot.block(3*n,0,3*nc,num_dof) = p;
      n += nc;
    }
  }
}

/* [body_ind,Tframe] = parseBodyOrFrameID(body_or_frame_id) */
int RigidBodyManipulator::parseBodyOrFrameID(const int body_or_frame_id, Matrix4d* Tframe)
{
  int body_ind=0;
  if (body_or_frame_id == -1) {
    cerr << "parseBodyOrFrameID got a -1, which should have been reserved for COM.  Shouldn't have gotten here." << endl;
  } else if (body_or_frame_id<0) {
    int frame_ind = -body_or_frame_id-2;
    body_ind = frames[frame_ind].body_ind;

    if (Tframe)
      (*Tframe) = frames[frame_ind].Ttree;
  } else {
    body_ind = body_or_frame_id;
    if (Tframe)
      (*Tframe) = Matrix4d::Identity();
  }
  return body_ind;
}

void RigidBodyManipulator::findAncestorBodies(std::vector<int>& ancestor_bodies, int body_idx)
{
  const RigidBody* current_body = bodies[body_idx].get();
  while (current_body->parent != -1)
  {
    ancestor_bodies.push_back(current_body->parent);
    current_body = bodies[current_body->parent].get();
  }
}

void RigidBodyManipulator::findKinematicPath(KinematicPath& path, int start_body_or_frame_idx, int end_body_or_frame_idx)
{
  // find all ancestors of start_body and end_body
  int start_body = parseBodyOrFrameID(start_body_or_frame_idx);

  std::vector<int> start_body_ancestors;
  start_body_ancestors.push_back(start_body);
  findAncestorBodies(start_body_ancestors, start_body);

  int end_body = parseBodyOrFrameID(end_body_or_frame_idx);
  std::vector<int> end_body_ancestors;
  end_body_ancestors.push_back(end_body);
  findAncestorBodies(end_body_ancestors, end_body);

  // find least common ancestor
  size_t common_size = std::min(start_body_ancestors.size(), end_body_ancestors.size());
  bool least_common_ancestor_found = false;
  std::vector<int>::iterator start_body_lca_it = start_body_ancestors.end() - common_size;
  std::vector<int>::iterator end_body_lca_it = end_body_ancestors.end() - common_size;

  for (size_t i = 0; i < common_size; i++) {
    if (*start_body_lca_it == *end_body_lca_it) {
      least_common_ancestor_found = true;
      break;
    }
    start_body_lca_it++;
    end_body_lca_it++;
  }

  if (!least_common_ancestor_found) {
    std::ostringstream stream;
    stream << "There is no path between " << bodies[start_body]->linkname << " and " << bodies[end_body]->linkname << ".";
    throw std::runtime_error(stream.str());
  }
  int least_common_ancestor = *start_body_lca_it;

  // compute path
  path.joint_path.clear();
  path.joint_direction_signs.clear();
  path.body_path.clear();

  std::vector<int>::iterator it = start_body_ancestors.begin();
  for ( ; it != start_body_lca_it; it++) {
    path.joint_path.push_back(*it);
    path.joint_direction_signs.push_back(-1);
    path.body_path.push_back(*it);
  }

  path.body_path.push_back(least_common_ancestor);

  std::vector<int>::reverse_iterator reverse_it(end_body_lca_it);
  for ( ; reverse_it != end_body_ancestors.rend(); reverse_it++) {
    path.joint_path.push_back(*reverse_it);
    path.joint_direction_signs.push_back(1);
    path.body_path.push_back(*reverse_it);
  }
}

/*
 * rotation_type  0, no rotation
 * 		  1, output Euler angles
 * 		  2, output quaternion [w,x,y,z], with w>=0
 */

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwardKin(const int body_or_frame_id, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x)
{
  int n_pts = static_cast<int>(pts.cols()); Matrix4d Tframe;
  int body_ind = parseBodyOrFrameID(body_or_frame_id, &Tframe);

  MatrixXd T = bodies[body_ind]->T.topLeftCorner(3,4)*Tframe;

  if (rotation_type == 0) {
    x = T*pts;
  } else if (rotation_type == 1) {
    Vector3d rpy;
    rpy << atan2(T(2,1),T(2,2)), atan2(-T(2,0),sqrt(T(2,1)*T(2,1) + T(2,2)*T(2,2))), atan2(T(1,0),T(0,0));
    // NOTE: we're assuming an X-Y-Z convention was used to construct T

    x = MatrixXd::Zero(6,n_pts);
    x.block(0,0,3,n_pts) = T*pts;
    x.block(3,0,3,n_pts) = rpy.replicate(1,n_pts);
  }
  else if(rotation_type == 2)
  {
    Vector4d quat, case_check;
    case_check << T(0,0)+T(1,1)+T(2,2), T(0,0)-T(1,1)-T(2,2), -T(0,0)+T(1,1)-T(2,2), -T(0,0)-T(1,1)+T(2,2);
    int ind; double val = case_check.maxCoeff(&ind);

    switch(ind) {
      case 0: { // val = trace(M)
        double w = sqrt(1+val)/2.0;
        double w4 = w*4;
        quat << w,
                (T(2,1)-T(1,2))/w4,
                (T(0,2)-T(2,0))/w4,
                (T(1,0)-T(0,1))/w4;
      } break;
      case 1: { // val = M(1,1) - M(2,2) - M(3,3)
        double s = 2*sqrt(1+val);
        quat << (T(2,1)-T(1,2))/s,
                0.25*s,
                (T(0,1)+T(1,0))/s,
                (T(0,2)+T(2,0))/s;
      } break;
      case 2: { // val = M(2,2) - M(1,1) - M(3,3)
        double s = 2*(sqrt(1+val));
        quat << (T(0,2)-T(2,0))/s,
                (T(0,1)+T(1,0))/s,
                0.25*s,
                (T(1,2)+T(2,1))/s;
      } break;
      default: { // val = M(3,3) - M(2,2) - M(1,1)
        double s = 2*(sqrt(1+val));
        quat << (T(1,0)-T(0,1))/s,
                (T(0,2)+T(2,0))/s,
                (T(1,2)+T(2,1))/s,
                0.25*s;
      } break;
    }

    x = MatrixXd::Zero(7,n_pts);
    x.block(0,0,3,n_pts) = T*pts;
    x.block(3,0,4,n_pts) = quat.replicate(1,n_pts);
  }
}

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
void RigidBodyManipulator::bodyKin(const int body_or_frame_id, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x, MatrixBase<DerivedC> *J, MatrixBase<DerivedD> *P)
{
  Matrix4d Tframe;
  int body_ind = parseBodyOrFrameID(body_or_frame_id, &Tframe);

  MatrixXd Tinv = (bodies[body_ind]->T*Tframe).inverse();
  x = Tinv.topLeftCorner(3,4)*pts;

  if (J) {
    int i;
    MatrixXd dTdq =  bodies[body_ind]->dTdq.topLeftCorner(3*num_dof,4)*Tframe;
    MatrixXd dTinvdq = dTdq*Tinv;
    for (i=0;i<num_dof;i++) {
      MatrixXd dTinvdqi = MatrixXd::Zero(4,4);
      dTinvdqi.row(0) = dTinvdq.row(i);
      dTinvdqi.row(1) = dTinvdq.row(num_dof+i);
      dTinvdqi.row(2) = dTinvdq.row(2*num_dof+i);
      dTinvdqi = -Tinv*dTinvdqi;
      MatrixXd dxdqi = dTinvdqi.topLeftCorner(3,4)*pts;
      dxdqi.resize(dxdqi.rows()*dxdqi.cols(),1);
      J->col(i) = dxdqi;
    }
  }
  if (P) {
    int i;
    for (i=0;i<pts.cols();i++) {
      P->block(i*3,i*3,3,3) = Tinv.topLeftCorner(3,3);
    }
  }

}

template<typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::geometricJacobian(
    int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot, std::vector<int>* v_or_qdot_indices)
{
  if (use_new_kinsol) {
    if (gradient_order > 1) {
      throw std::runtime_error("gradient order not supported");
    }

    KinematicPath kinematic_path;
    findKinematicPath(kinematic_path, base_body_or_frame_ind, end_effector_body_or_frame_ind);
    int nq = num_dof;

    int cols = 0;
    int body_index;
    for (int i = 0; i < kinematic_path.joint_path.size(); i++) {
      body_index = kinematic_path.joint_path[i];
      const std::unique_ptr<RigidBody>& body = bodies[body_index];
      const DrakeJoint& joint = body->getJoint();

      cols += in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
    }

    GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, cols, nq, gradient_order);
    auto& J = ret.value();

    DrakeJoint::MotionSubspaceType motion_subspace;
    if (v_or_qdot_indices != nullptr) {
      v_or_qdot_indices->clear();
      v_or_qdot_indices->reserve(cols);
    }

    int col_start = 0;
    int sign;
    for (int i = 0; i < kinematic_path.joint_path.size(); i++) {
      body_index = kinematic_path.joint_path[i];
      RigidBody& body = *bodies[body_index];
      const DrakeJoint& joint = body.getJoint();
      int ncols_block = in_terms_of_qdot ? joint.getNumPositions() : joint.getNumVelocities();
      sign = kinematic_path.joint_direction_signs[i];
      auto J_block = J.template block<TWIST_SIZE, Dynamic>(0, col_start, TWIST_SIZE, ncols_block);
      if (in_terms_of_qdot) {
        J_block.noalias() = sign * body.J * body.qdot_to_v;
      }
      else {
        J_block.noalias() = sign * body.J;
      }

      if (gradient_order > 0) {
        auto& dJ = ret.gradient().value();
        if (in_terms_of_qdot) {
          dJ.block(col_start * TWIST_SIZE, 0, J_block.size(), nq).noalias() = (sign * matGradMultMat(body.J, body.qdot_to_v, body.dJdq, body.dqdot_to_v_dq)).eval();
        }
        else {
          dJ.block(col_start * TWIST_SIZE, 0, J_block.size(), nq).noalias() = (sign * body.dJdq); //.eval();
        }
      }

      if (v_or_qdot_indices != nullptr) {
        int cols_block_start = in_terms_of_qdot ? body.dofnum : body.velocity_num_start;
        for (int j = 0; j < ncols_block; j++) {
          v_or_qdot_indices->push_back(cols_block_start + j);
        }
      }
      col_start += ncols_block;
    }

    if (expressed_in_body_or_frame_ind != 0) {
      auto T_world_to_frame_gradientvar = relativeTransform<double>(expressed_in_body_or_frame_ind, 0, gradient_order);
      Isometry3d T_world_to_frame(T_world_to_frame_gradientvar.value());

      if (gradient_order > 0) {
        auto& dJ = ret.gradient().value();
        dJ = (dTransformSpatialMotion(T_world_to_frame, J, T_world_to_frame_gradientvar.gradient().value(), dJ)).eval(); // eval to avoid aliasing
      }
      J = transformSpatialMotion(T_world_to_frame, J);
    }
    return ret;
  }
  else {
    if (gradient_order > 0) {
      throw std::runtime_error("gradient order not supported");
    }

    KinematicPath kinematic_path;
    findKinematicPath(kinematic_path, base_body_or_frame_ind, end_effector_body_or_frame_ind);

    int cols = 0;
    int body_index;
    for (int i = 0; i < kinematic_path.joint_path.size(); i++) {
      body_index = kinematic_path.joint_path[i];
      const std::unique_ptr<RigidBody>& body = bodies[body_index];
      const DrakeJoint& joint = body->getJoint();
      cols += joint.getNumVelocities();
    }

    Matrix4d Tframe;
    int expressed_in_body = parseBodyOrFrameID(expressed_in_body_or_frame_ind, &Tframe);
    Matrix4d T_world_to_frame = (bodies[expressed_in_body]->T * Tframe).inverse();

    GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> ret(TWIST_SIZE, cols);
    auto& J = ret.value();
    DrakeJoint::MotionSubspaceType motion_subspace;
    if (v_or_qdot_indices) {
      v_or_qdot_indices->clear();
      v_or_qdot_indices->reserve(cols);
    }

    int col_start = 0;
    int sign;
    for (int i = 0; i < kinematic_path.joint_path.size(); i++) {
      body_index = kinematic_path.joint_path[i];
      const std::unique_ptr<RigidBody>& body = bodies[body_index];
      const DrakeJoint& joint = body->getJoint();

      joint.motionSubspace(cached_q.data() + body->dofnum, motion_subspace); // TODO: should just let DrakeJoints work with VectorXds

      sign = kinematic_path.joint_direction_signs[i];
      auto block = J.template block<TWIST_SIZE, Dynamic>(0, col_start, TWIST_SIZE, joint.getNumVelocities());
      block.noalias() = sign * transformSpatialMotion(Isometry3d(body->T), motion_subspace);

      if (v_or_qdot_indices) {
        for (int j = 0; j < joint.getNumVelocities(); j++) {
          v_or_qdot_indices->push_back(body->dofnum + j); // assumes qd = v
        }
      }
      col_start = col_start + joint.getNumVelocities();
    }
    J = transformSpatialMotion(Isometry3d(T_world_to_frame), J);
    return ret;
  }
}

template<typename Scalar>
GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> RigidBodyManipulator::relativeTransform(int base_or_frame_ind, int body_or_frame_ind, int gradient_order)
{
  if (!use_new_kinsol)
    throw std::runtime_error("method requires new kinsol format");

  int nq = num_dof;
  GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> ret(SPACE_DIMENSION + 1, SPACE_DIMENSION + 1, nq, gradient_order);

  Matrix4d Tbase_frame;
  int base_ind = parseBodyOrFrameID(base_or_frame_ind, &Tbase_frame);
  Matrix4d Tbody_frame;
  int body_ind = parseBodyOrFrameID(body_or_frame_ind, &Tbody_frame);

  Isometry3d Tbaseframe_to_world = bodies[base_ind]->T_new * Isometry3d(Tbase_frame); // TODO: copy to Isometry3d
  Isometry3d Tworld_to_baseframe = Tbaseframe_to_world.inverse();
  Isometry3d Tbodyframe_to_world = bodies[body_ind]->T_new * Isometry3d(Tbody_frame); // TODO: copy to Isometry3d
  ret.value() = (Tworld_to_baseframe * Tbodyframe_to_world).matrix();

  if (gradient_order > 0) {
    auto dTbaseframe_to_world = matGradMult(bodies[base_ind]->dTdq_new, Tbase_frame);
    auto dTworld_to_baseframe = dHomogTransInv(Tbaseframe_to_world, dTbaseframe_to_world);
    auto dTbodyframe_to_world = matGradMult(bodies[body_ind]->dTdq_new, Tbody_frame);
    ret.gradient().value() = matGradMultMat(Tworld_to_baseframe.matrix(), Tbodyframe_to_world.matrix(), dTworld_to_baseframe, dTbodyframe_to_world);
  }
  if (gradient_order > 1) {
    throw std::runtime_error("gradient order > 1 not supported");
  }
  return ret;
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwardJac(const int body_or_frame_id, const MatrixBase<DerivedA> &pts, const int rotation_type, MatrixBase<DerivedB> &J)
{
  int n_pts = static_cast<int>(pts.cols()); Matrix4d Tframe;
  int body_ind = parseBodyOrFrameID(body_or_frame_id, &Tframe);

  MatrixXd dTdq =  bodies[body_ind]->dTdq.topLeftCorner(3*num_dof,4)*Tframe;
  MatrixXd tmp =dTdq*pts;
  MatrixXd Jt = Map<MatrixXd>(tmp.data(),num_dof,3*n_pts);
  J.topLeftCorner(3*n_pts,num_dof) = Jt.transpose();

  if (rotation_type == 1) {
    Matrix3d R = (bodies[body_ind]->T*Tframe).topLeftCorner(3,3);
    /*
     * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
     * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
     */

    VectorXd dR21_dq(num_dof),dR22_dq(num_dof),dR20_dq(num_dof),dR00_dq(num_dof),dR10_dq(num_dof);
    for (int i=0; i<num_dof; i++) {
      dR21_dq(i) = dTdq(2*num_dof+i,1);
      dR22_dq(i) = dTdq(2*num_dof+i,2);
      dR20_dq(i) = dTdq(2*num_dof+i,0);
      dR00_dq(i) = dTdq(i,0);
      dR10_dq(i) = dTdq(num_dof+i,0);
    }
    double sqterm1 = R(2,1)*R(2,1) + R(2,2)*R(2,2);
    double sqterm2 = R(0,0)*R(0,0) + R(1,0)*R(1,0);

    MatrixXd Jr = MatrixXd::Zero(3,num_dof);

    Jr.block(0,0,1,num_dof) = ((R(2,2)*dR21_dq - R(2,1)*dR22_dq)/sqterm1).transpose();
    Jr.block(1,0,1,num_dof) = ((-sqrt(sqterm1)*dR20_dq + R(2,0)/sqrt(sqterm1)*(R(2,1)*dR21_dq + R(2,2)*dR22_dq) )/(R(2,0)*R(2,0) + R(2,1)*R(2,1) + R(2,2)*R(2,2))).transpose();
    Jr.block(2,0,1,num_dof)= ((R(0,0)*dR10_dq - R(1,0)*dR00_dq)/sqterm2).transpose();

    MatrixXd Jfull = MatrixXd::Zero(2*3*n_pts,num_dof);
    for (int i=0; i<n_pts; i++) {
      Jfull.block(i*6,0,3,num_dof) = J.block(i*3,0,3,num_dof);
      Jfull.block(i*6+3,0,3,num_dof) = Jr;
    }
    J=Jfull;
  } else if(rotation_type == 2) {
    Matrix3d R = (bodies[body_ind]->T*Tframe).topLeftCorner(3,3);

    VectorXd dR21_dq(num_dof),dR22_dq(num_dof),dR20_dq(num_dof),dR00_dq(num_dof),dR10_dq(num_dof),dR01_dq(num_dof),dR02_dq(num_dof),dR11_dq(num_dof),dR12_dq(num_dof);
    for (int i=0; i<num_dof; i++) {
      dR21_dq(i) = dTdq(2*num_dof+i,1);
      dR22_dq(i) = dTdq(2*num_dof+i,2);
      dR20_dq(i) = dTdq(2*num_dof+i,0);
      dR00_dq(i) = dTdq(i,0);
      dR10_dq(i) = dTdq(num_dof+i,0);
      dR01_dq(i) = dTdq(i,1);
      dR02_dq(i) = dTdq(i,2);
      dR11_dq(i) = dTdq(num_dof+i,1);
      dR12_dq(i) = dTdq(num_dof+i,2);
    }

  	Vector4d case_check;
  	case_check << R(0,0)+R(1,1)+R(2,2), R(0,0)-R(1,1)-R(2,2), -R(0,0)+R(1,1)-R(2,2), -R(0,0)-R(1,1)+R(2,2);
  	int ind; double val = case_check.maxCoeff(&ind);

    MatrixXd Jq = MatrixXd::Zero(4,num_dof);
  	switch(ind) {
  	case 0: { // val = trace(M)
  			double qw = sqrt(1+val)/2.0;
  			VectorXd dqwdq = (dR00_dq+dR11_dq+dR22_dq)/(4*sqrt(1+val));
  			double wsquare4 = 4*qw*qw;
  			Jq.block(0,0,1,num_dof) = dqwdq.transpose();
  			Jq.block(1,0,1,num_dof) = (((dR21_dq-dR12_dq)*qw-(R(2,1)-R(1,2))*dqwdq).transpose())/wsquare4;
  			Jq.block(2,0,1,num_dof) = (((dR02_dq-dR20_dq)*qw-(R(0,2)-R(2,0))*dqwdq).transpose())/wsquare4;
  			Jq.block(3,0,1,num_dof) = (((dR10_dq-dR01_dq)*qw-(R(1,0)-R(0,1))*dqwdq).transpose())/wsquare4;
  		} break;
    case 1: { // val = M(1,1) - M(2,2) - M(3,3)
      	double s = 2*sqrt(1+val); double ssquare = s*s;
      	VectorXd dsdq = (dR00_dq - dR11_dq - dR22_dq)/sqrt(1+val);
      	Jq.block(0,0,1,num_dof) = (((dR21_dq-dR12_dq)*s - (R(2,1)-R(1,2))*dsdq).transpose())/ssquare;
      	Jq.block(1,0,1,num_dof) = .25*dsdq.transpose();
      	Jq.block(2,0,1,num_dof) = (((dR01_dq+dR10_dq)*s - (R(0,1)+R(1,0))*dsdq).transpose())/ssquare;
      	Jq.block(3,0,1,num_dof) = (((dR02_dq+dR20_dq)*s - (R(0,2)+R(2,0))*dsdq).transpose())/ssquare;
    	} break;
    case 2: { // val = M(2,2) - M(1,1) - M(3,3)
    		double s = 2*sqrt(1+val); double ssquare = s*s;
    		VectorXd dsdq = (-dR00_dq + dR11_dq - dR22_dq)/sqrt(1+val);
    		Jq.block(0,0,1,num_dof) = (((dR02_dq-dR20_dq)*s - (R(0,2)-R(2,0))*dsdq).transpose())/ssquare;
    		Jq.block(1,0,1,num_dof) = (((dR01_dq+dR10_dq)*s - (R(0,1)+R(1,0))*dsdq).transpose())/ssquare;
    		Jq.block(2,0,1,num_dof) = .25*dsdq.transpose();
    		Jq.block(3,0,1,num_dof) = (((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq).transpose())/ssquare;
    	} break;
    default: { // val = M(3,3) - M(2,2) - M(1,1)
    		double s = 2*sqrt(1+val); double ssquare = s*s;
    		VectorXd dsdq = (-dR00_dq - dR11_dq + dR22_dq)/sqrt(1+val);
    		Jq.block(0,0,1,num_dof) = (((dR10_dq-dR01_dq)*s - (R(1,0)-R(0,1))*dsdq).transpose())/ssquare;
    		Jq.block(1,0,1,num_dof) = (((dR02_dq+dR20_dq)*s - (R(0,2)+R(2,0))*dsdq).transpose())/ssquare;
    		Jq.block(2,0,1,num_dof) = (((dR12_dq+dR21_dq)*s - (R(1,2)+R(2,1))*dsdq).transpose())/ssquare;
    		Jq.block(3,0,1,num_dof) = .25*dsdq.transpose();
    	} break;
  	}

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
void RigidBodyManipulator::forwardJacDot(const int body_or_frame_id, const MatrixBase<DerivedA> &pts, const int rotation_type, MatrixBase<DerivedB>& Jdot)
{
  int n_pts = static_cast<int>(pts.cols()); Matrix4d Tframe;
  int body_ind = parseBodyOrFrameID(body_or_frame_id, &Tframe);

	MatrixXd tmp = bodies[body_ind]->dTdqdot*Tframe*pts;
	MatrixXd Jdott = Map<MatrixXd>(tmp.data(),num_dof,3*n_pts);
	Jdot = Jdott.transpose();

	if (rotation_type==1) {

		MatrixXd dTdqdot =  bodies[body_ind]->dTdqdot*Tframe;
		Matrix3d R = (bodies[body_ind]->T*Tframe).topLeftCorner(3,3);
		/*
		 * note the unusual format of dTdq(chosen for efficiently calculating jacobians from many pts)
		 * dTdq = [dT(1,:)dq1; dT(1,:)dq2; ...; dT(1,:)dqN; dT(2,dq1) ...]
		 */

		VectorXd dR21_dq(num_dof),dR22_dq(num_dof),dR20_dq(num_dof),dR00_dq(num_dof),dR10_dq(num_dof);
		for (int i=0; i<num_dof; i++) {
			dR21_dq(i) = dTdqdot(2*num_dof+i,1);
			dR22_dq(i) = dTdqdot(2*num_dof+i,2);
			dR20_dq(i) = dTdqdot(2*num_dof+i,0);
			dR00_dq(i) = dTdqdot(i,0);
			dR10_dq(i) = dTdqdot(num_dof+i,0);
		}
		double sqterm1 = R(2,1)*R(2,1) + R(2,2)*R(2,2);
		double sqterm2 = R(0,0)*R(0,0) + R(1,0)*R(1,0);

		MatrixXd Jr = MatrixXd::Zero(3,num_dof);

		Jr.block(0,0,1,num_dof) = ((R(2,2)*dR21_dq - R(2,1)*dR22_dq)/sqterm1).transpose();
		Jr.block(1,0,1,num_dof) = ((-sqrt(sqterm1)*dR20_dq + R(2,0)/sqrt(sqterm1)*(R(2,1)*dR21_dq + R(2,2)*dR22_dq) )/(R(2,0)*R(2,0) + R(2,1)*R(2,1) + R(2,2)*R(2,2))).transpose();
		Jr.block(2,0,1,num_dof)= ((R(0,0)*dR10_dq - R(1,0)*dR00_dq)/sqterm2).transpose();

		MatrixXd Jfull = MatrixXd::Zero(2*3*n_pts,num_dof);
		for (int i=0; i<n_pts; i++) {
			Jfull.block(i*6,0,3,num_dof) = Jdot.block(i*3,0,3,num_dof);
			Jfull.block(i*6+3,0,3,num_dof) = Jr;
		}
		Jdot=Jfull;
	}
}

template <typename DerivedA, typename DerivedB>
void RigidBodyManipulator::forwarddJac(const int body_or_frame_id, const MatrixBase<DerivedA> &pts, MatrixBase<DerivedB>& dJ)
{
  int n_pts = static_cast<int>(pts.cols()); Matrix4d Tframe;
  int body_ind = parseBodyOrFrameID(body_or_frame_id, &Tframe);

  int i,j;
  MatrixXd dJ_reshaped = MatrixXd(num_dof, 3*n_pts*num_dof);
  MatrixXd tmp = MatrixXd(3*num_dof,n_pts);
  for (i = 0; i < num_dof; i++) {
    tmp = bodies[body_ind]->ddTdqdq.block(i*num_dof*3,0,3*num_dof,4)*Tframe*pts;  //dim*num_dof x n_pts
    for (j = 0; j < n_pts; j++) {
      dJ_reshaped.block(i,j*3*num_dof,1,3*num_dof) = tmp.col(j).transpose();
    }
    //       dJ_reshaped.row(i) << tmp.col(0).transpose(), tmp.col(1).transpose();
  }
  MatrixXd dJ_t = Map<MatrixXd>(dJ_reshaped.data(), num_dof*num_dof, 3*n_pts);
  dJ = dJ_t.transpose();
}

template <typename DerivedPoints>
GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> RigidBodyManipulator::forwardKinNew(const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order)
{
  if (gradient_order > 2) {
    throw std::runtime_error("only first and second order gradients are available");
  }
  int x_gradient_order = std::min(gradient_order, 1);
  int J_gradient_order = gradient_order > 1 ? gradient_order - 1 : 0;

  int nq = num_dof;
  int npoints = static_cast<int>(points.cols());
  typedef typename DerivedPoints::Scalar Scalar;

  // compute rotation and translation
  GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> T = relativeTransform<Scalar>(new_body_or_frame_ind, current_body_or_frame_ind, x_gradient_order);

  GradientVar<Scalar, SPACE_DIMENSION, SPACE_DIMENSION> R(SPACE_DIMENSION, SPACE_DIMENSION, nq, x_gradient_order);
  R.value() = T.value().template topLeftCorner<SPACE_DIMENSION, SPACE_DIMENSION>();
  if (x_gradient_order > 0)
    R.gradient().value() = getSubMatrixGradient<Eigen::Dynamic>(T.gradient().value(), intRange<SPACE_DIMENSION>(0), intRange<SPACE_DIMENSION>(0), T.value().rows());

  GradientVar<Scalar, SPACE_DIMENSION, 1> p(SPACE_DIMENSION, 1, nq, x_gradient_order);
  p.value() = T.value().template topRightCorner<SPACE_DIMENSION, 1>();
  if (x_gradient_order > 0)
    p.gradient().value() = getSubMatrixGradient<Eigen::Dynamic>(T.gradient().value(), intRange<SPACE_DIMENSION>(0), intRange<1>(SPACE_DIMENSION), T.value().rows());

  // transform points to new frame
  GradientVar<Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> x(SPACE_DIMENSION + rotationRepresentationSize(rotation_type), npoints, nq, gradient_order);
  x.value().template topRows<SPACE_DIMENSION>().noalias() = R.value() * points;
  x.value().template topRows<SPACE_DIMENSION>().colwise() += p.value();

  // convert rotation representation
  GradientVar<Scalar, Eigen::Dynamic, 1> qrot = rotmat2Representation(R, rotation_type);
  x.value().bottomRows(qrot.value().rows()).colwise() = qrot.value();

  if (x_gradient_order > 0) {
    x.gradient().value().setZero();
    for (int i = 0; i < npoints; i++) {
      const auto& point = points.template middleCols<1>(i);
      auto point_gradient = matGradMult(R.gradient().value(), point).eval();
      point_gradient += p.gradient().value();

      // position rows
      for (int row = 0; row < SPACE_DIMENSION; row++) {
        setSubMatrixGradient<Eigen::Dynamic>(x.gradient().value(), getSubMatrixGradient<Eigen::Dynamic>(point_gradient, row, 0, point.rows()), row, i, x.value().rows());
      }

      // rotation rows
      for (int row = 0; row < qrot.value().rows(); row++) {
        setSubMatrixGradient<Eigen::Dynamic>(x.gradient().value(), qrot.gradient().value().row(row), row + SPACE_DIMENSION, i, x.value().rows());
      }
    }
  }

  if (gradient_order > 1) {
    auto J = forwardJacV(x, current_body_or_frame_ind, new_body_or_frame_ind, rotation_type, true, J_gradient_order);
    x.gradient().gradient().value() = J.gradient().value();
  }

  return x;
}

template <typename Scalar, int XRows, int XCols>
GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardJacV(const GradientVar<Scalar, XRows, XCols>& x, int body_or_frame_ind, int base_or_frame_ind, int rotation_type, bool compute_analytic_jacobian, int gradient_order)
{
  if (!use_new_kinsol)
    throw std::runtime_error("method requires new kinsol format");

  if (gradient_order > 1) {
    throw std::runtime_error("only first order gradient is available");
  }

  int nq = num_dof;
  int nv = num_velocities;
  int cols = compute_analytic_jacobian ? nq : nv;
  int npoints = static_cast<int>(x.value().cols());

  // compute geometric Jacobian
  int body_ind = parseBodyOrFrameID(body_or_frame_ind);
  int base_ind = parseBodyOrFrameID(base_or_frame_ind);
  int expressed_in = base_ind;
  std::vector<int> v_or_q_indices;
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> J_geometric = geometricJacobian<Scalar>(base_ind, body_ind, expressed_in, gradient_order, compute_analytic_jacobian, &v_or_q_indices);

  // split up into rotational and translational parts
  GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> Jomega(SPACE_DIMENSION, J_geometric.value().cols(), nq, gradient_order);
  Jomega.value() = J_geometric.value().template topRows<SPACE_DIMENSION>();

  GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> Jv(SPACE_DIMENSION, J_geometric.value().cols(), nq, gradient_order);
  Jv.value() = J_geometric.value().template bottomRows<SPACE_DIMENSION>();

  if (gradient_order > 0) {
    for (int col = 0; col < J_geometric.value().cols(); col++) {
      for (int row = 0; row < SPACE_DIMENSION; row++) {
        // Jomega
        setSubMatrixGradient<Eigen::Dynamic>(Jomega.gradient().value(), getSubMatrixGradient<Eigen::Dynamic>(J_geometric.gradient().value(), row, col, J_geometric.value().rows()), row, col, Jomega.value().rows());

        // Jv
        setSubMatrixGradient<Eigen::Dynamic>(Jv.gradient().value(), getSubMatrixGradient<Eigen::Dynamic>(J_geometric.gradient().value(), row + SPACE_DIMENSION, col, J_geometric.value().rows()), row, col, Jv.value().rows());
      }
    }
  }

  // compute rotation Jacobian
  int rotation_representation_size = rotationRepresentationSize(rotation_type);
  GradientVar<Scalar, Eigen::Dynamic, 1> qrot(rotation_representation_size, 1, nq, gradient_order);
  qrot.value() = x.value().template block<Eigen::Dynamic, 1>(SPACE_DIMENSION, 0, x.value().rows() - SPACE_DIMENSION, 1);
  if (gradient_order > 0) {
    for (int i = 0; i < rotation_representation_size; i++) {
      qrot.gradient().value().template middleRows<1>(i) = getSubMatrixGradient<Eigen::Dynamic>(x.gradient().value(), SPACE_DIMENSION + i, 0, x.value().rows());
    }
  }

  GradientVar<Scalar, Eigen::Dynamic, SPACE_DIMENSION> Phi = angularvel2RepresentationDotMatrix(rotation_type, qrot);
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jrot(Phi.value().rows(), Jomega.value().cols(), nq, gradient_order);
  Jrot.value() = Phi.value() * Jomega.value();
  if (gradient_order > 0) {
    Jrot.gradient().value() = matGradMultMat(Phi.value(), Jomega.value(), Phi.gradient().value(), Jomega.gradient().value());
  }

  // compute J
  GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> J(x.value().size(), cols, nq, gradient_order);
  J.value().setZero();
  if (gradient_order > 0)
    J.gradient().value().setZero();

  int row_start = 0;
  for (int i = 0; i < npoints; i++) {
    // translation part
    int col_start = 0;
    const auto& point = x.value().template block<SPACE_DIMENSION, 1>(0, i);
    for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
      J.value().template block<SPACE_DIMENSION, 1>(row_start, *it) = Jv.value().template middleCols<1>(col_start);
      const auto& Jomega_col = Jomega.value().template middleCols<1>(col_start);
      J.value().template block<SPACE_DIMENSION, 1>(row_start, *it).noalias() += Jomega_col.cross(point);

      if (gradient_order > 0) {
        auto rows = intRange<SPACE_DIMENSION>(0);
        auto dJpos_col = getSubMatrixGradient<Eigen::Dynamic>(Jv.gradient().value(), rows, intRange<1>(col_start), Jv.value().rows());
        auto dJomega_col = getSubMatrixGradient<Eigen::Dynamic>(Jomega.gradient().value(), rows, intRange<1>(col_start), Jv.value().rows());
        auto dpoint = getSubMatrixGradient<Eigen::Dynamic>(x.gradient().value(), rows, intRange<1>(i), x.value().rows());
        dJpos_col.noalias() += dcrossProduct(Jomega_col, point, dJomega_col, dpoint);
        setSubMatrixGradient<Eigen::Dynamic>(J.gradient().value(), dJpos_col, intRange<SPACE_DIMENSION>(row_start), intRange<1>(*it), J.value().rows());
      }
      col_start++;
    }
    row_start += SPACE_DIMENSION;

    // rotation part
    if (Jrot.value().rows() > 0) {
      col_start = 0;
      for (std::vector<int>::iterator it = v_or_q_indices.begin(); it != v_or_q_indices.end(); ++it) {
        J.value().template block<Eigen::Dynamic, 1>(row_start, *it, Jrot.value().rows(), 1) = Jrot.value().template middleCols<1>(col_start);

        if (gradient_order > 0) {
          for (int row = 0; row < rotation_representation_size; row++) {
            auto dJrot_element = getSubMatrixGradient<Eigen::Dynamic>(Jrot.gradient().value(), row, col_start, Jrot.value().rows());
            setSubMatrixGradient<Eigen::Dynamic>(J.gradient().value(), dJrot_element, row_start + row, *it, J.value().rows());
          }
        }
        col_start++;
      }
      row_start += rotation_representation_size;
    }
  }

  return J;
}

template <typename Scalar>
GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinPositionGradient(int npoints, int current_body_or_frame_ind, int new_body_or_frame_ind, int gradient_order)
{
  if (gradient_order > 1)
    throw std::runtime_error("Only first order gradients supported");

  int nq = num_dof;
  auto Tinv = relativeTransform<Scalar>(current_body_or_frame_ind, new_body_or_frame_ind, gradient_order);
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> ret(SPACE_DIMENSION * npoints, SPACE_DIMENSION * npoints, nq, gradient_order);
  ret.value().setZero();
  for (int i = 0; i < npoints; i++) {
    ret.value().template block<SPACE_DIMENSION, SPACE_DIMENSION>(SPACE_DIMENSION * i, SPACE_DIMENSION * i) = Tinv.value().template topLeftCorner<SPACE_DIMENSION, SPACE_DIMENSION>();
  }

  if (gradient_order > 0) {
    ret.gradient().value().setZero();
    std::vector<int> rows_cols;
    rows_cols.reserve(SPACE_DIMENSION);
    auto Rinv_gradient = getSubMatrixGradient<Eigen::Dynamic>(Tinv.gradient().value(), intRange<SPACE_DIMENSION>(0), intRange<SPACE_DIMENSION>(0), Tinv.value().rows());

    for (int i = 0; i < npoints; i++) {
      rows_cols.clear();
      for (int j = 0; j < SPACE_DIMENSION; j++)
        rows_cols.push_back(SPACE_DIMENSION * i + j);
      setSubMatrixGradient(ret.gradient().value(), Rinv_gradient, rows_cols, rows_cols, ret.value().rows());
    }
  }
  return ret;
}

template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE, typename DerivedF>
void RigidBodyManipulator::HandC(double * const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH, MatrixBase<DerivedE> *dC, MatrixBase<DerivedF> * const df_ext)
{
  H = MatrixXd::Zero(num_dof,num_dof);
  if (dH) *dH = MatrixXd::Zero(num_dof*num_dof,num_dof);
  // C gets overwritten completely in the algorithm below

  VectorXd vJ(6), fh(6), dfh(6), dvJdqd(6);
  MatrixXd XJ(6,6), dXJdq(6,6);
  int i,j,k,n,np,nk;

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
      if (df_ext) {
	dfvpdq[i] = dfvpdq[i] - df_ext->block(i*6,0,6,num_dof);
	dfvpdqd[i] = dfvpdqd[i] - df_ext->block(i*6,num_dof,6,num_dof);
      }

    }
  }

  for (i=(NB-1); i>=0; i--) {
    n = dofnum[i];
    C(n) = (S[i]).transpose() * fvp[i] + damping[i]*qd[n];

    if (qd[n] >= coulomb_window[i]) {
      C(n) += coulomb_friction[i];
    }
    else if (qd[n] <= -coulomb_window[i]) {
      C(n) -= coulomb_friction[i];
    }
    else {
      C(n) += qd[n]/coulomb_window[i] * coulomb_friction[i];
    }

    if (dC) {
      (*dC).block(n,0,1,NB) = S[i].transpose()*dfvpdq[i];
      (*dC).block(n,NB,1,NB) = S[i].transpose()*dfvpdqd[i];
      (*dC)(n,NB+n) += damping[i];

      if (qd[n]<0 && qd[n]>-coulomb_window[i]) {
        (*dC)(n,NB+n) -= 1/coulomb_window[i] * coulomb_friction[i];
      }
      else if (qd[n]>=0 && qd[n]<coulomb_window[i]) {
        (*dC)(n,NB+n) += 1/coulomb_window[i] * coulomb_friction[i];
      }
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
    for (k=0; k < NB; k++) {
      nk = dofnum[k];
      for (i=0; i < NB; i++) {
        n = dofnum[i];
        fh = IC[i] * S[i];
        dfh = dIC[i][nk] * S[i]; //dfh/dqk
        (*dH)(n + n*NB,nk) = S[i].transpose() * dfh;
        j = i;
        while (parent[j] >= 0) {
          if (j==k) {
            dfh = Xup[j].transpose() * dfh + dXupdq[j].transpose() * fh;
          } else {
            dfh = Xup[j].transpose() * dfh;
          }
          fh = Xup[j].transpose() * fh;

          j = parent[j];
          np = dofnum[j];
          (*dH)(n + (np)*NB,nk) = S[j].transpose() * dfh;
          (*dH)(np + (n)*NB,nk) = (*dH)(n + np*NB,nk);
        }
      }
    }
  }
}

int RigidBodyManipulator::findLinkId(string linkname, int robot)
{
  std::transform(linkname.begin(), linkname.end(), linkname.begin(), ::tolower); // convert to lower case

  //std::regex linkname_connector("[abc]");
  //cout<<"get linkname_connector"<<endl;
  //linkname = std::regex_replace(linkname,linkname_connector,string("_"));
  bool* name_match = new bool[this->num_bodies];
  for(int i = 0;i<this->num_bodies;i++)
  {
    string lower_linkname = this->bodies[i]->linkname;
    std::transform(lower_linkname.begin(), lower_linkname.end(), lower_linkname.begin(), ::tolower); // convert to lower case
    if(lower_linkname.find(linkname) != string::npos)
    {
      name_match[i] = true;
    }
    else
    {
      name_match[i] = false;
    }
  }
  if(robot != -1)
  {
    for(int i = 0;i<this->num_bodies;i++)
    {
      if(name_match[i])
      {
        name_match[i] = this->bodies[i]->robotnum == robot;
      }
    }
  }
  // Unlike the MATLAB implementation, I am not handling the fixed joints
  int num_match = 0;
  int ind_match = -1;
  for(int i = 0;i<this->num_bodies;i++)
  {
    if(name_match[i])
    {
      num_match++;
      ind_match = i;
    }
  }
  if(num_match != 1)
  {
    cerr<<"couldn't find unique link "<<linkname<<endl;
    return(EXIT_FAILURE);
  }
  else
  {
    return ind_match;
  }
  delete[] name_match;
}

std::string RigidBodyManipulator::getBodyOrFrameName(int body_or_frame_id)
{
  if (body_or_frame_id>=0) {
    return bodies[body_or_frame_id]->linkname;
  } else if (body_or_frame_id < -1) {
    return frames[-body_or_frame_id-2].name;
  } else {
    return "COM";
  }
}


// explicit instantiations (required for linking):
template DLLEXPORT_RBM void RigidBodyManipulator::getCMM(double * const, double * const, MatrixBase< Map<MatrixXd> > &, MatrixBase< Map<MatrixXd> > &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCMM(double * const, double * const, MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOM(MatrixBase< Map<Vector3d> > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOM(MatrixBase< Map<MatrixXd> > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMJac(MatrixBase< Map<MatrixXd> > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMdJac(MatrixBase< Map<MatrixXd> > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMJacDot(MatrixBase< Map<MatrixXd> > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOM(MatrixBase< Vector3d > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOM(MatrixBase< MatrixXd > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMJac(MatrixBase< MatrixXd > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMdJac(MatrixBase< MatrixXd > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getCOMJacDot(MatrixBase< MatrixXd > &,const set<int> &);

template DLLEXPORT_RBM void RigidBodyManipulator::getContactPositions(MatrixBase <MatrixXd > &, const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getContactPositionsJac(MatrixBase <MatrixXd > &,const set<int> &);
template DLLEXPORT_RBM void RigidBodyManipulator::getContactPositionsJacDot(MatrixBase <MatrixXd > &,const set<int> &);

template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, const MatrixBase< MatrixXd >&, const int, MatrixBase< Map<MatrixXd> > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, const MatrixBase< MatrixXd >&, const int, MatrixBase< MatrixXd > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Vector3d > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Matrix<double,6,1> > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, MatrixBase< Vector4d > const&, const int, MatrixBase< Matrix<double,7,1> > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, MatrixBase< Map<MatrixXd> > const&, const int, MatrixBase< MatrixXd > &);
//template DLLEXPORT_RBM void RigidBodyManipulator::forwardKin(const int, const MatrixBase< Vector4d >&, const int, MatrixBase< Vector3d > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJac(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< Map<MatrixXd> > &);
//template DLLEXPORT_RBM void RigidBodyManipulator::forwardJac(const int, MatrixBase< Map<MatrixXd> > const&, const int, MatrixBase< MatrixXd > &);
//template DLLEXPORT_RBM void RigidBodyManipulator::forwardJac(const int, MatrixBase< MatrixXd > const&, const int, MatrixBase< MatrixXd > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJac(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< MatrixXd > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJac(const int, const MatrixBase< Vector4d > &, const int, MatrixBase< MatrixXd > &);
template DLLEXPORT_RBM void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< MatrixXd > &, MatrixBase< Map<MatrixXd> >&);
template DLLEXPORT_RBM void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< MatrixXd > &, MatrixBase< MatrixXd >&);
//template DLLEXPORT_RBM void RigidBodyManipulator::forwarddJac(const int, const MatrixBase< Vector4d > &, MatrixBase< MatrixXd >&);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< Map<MatrixXd> >&);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< MatrixXd > &, const int, MatrixBase< MatrixXd >&);
template DLLEXPORT_RBM void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< Vector4d > &, const int, MatrixBase< MatrixXd >&);
//template DLLEXPORT_RBM void RigidBodyManipulator::forwardJacDot(const int, const MatrixBase< Vector4d > &, MatrixBase< MatrixXd >&);
template DLLEXPORT_RBM void RigidBodyManipulator::bodyKin(const int, const MatrixBase< MatrixXd >&, MatrixBase< Map<MatrixXd> > &, MatrixBase< Map<MatrixXd> > *, MatrixBase< Map<MatrixXd> > *);
template DLLEXPORT_RBM void RigidBodyManipulator::bodyKin(const int, const MatrixBase< MatrixXd >&, MatrixBase< MatrixXd > &, MatrixBase< MatrixXd > *, MatrixBase< MatrixXd > *);

template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::centroidalMomentumMatrix(int);
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION, 1> RigidBodyManipulator::centerOfMass(int, const std::set<int>&);
template DLLEXPORT_RBM GradientVar<double, TWIST_SIZE, Eigen::Dynamic> RigidBodyManipulator::geometricJacobian<double>(int, int, int, int, bool, std::vector<int>*);
template DLLEXPORT_RBM GradientVar<double, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> RigidBodyManipulator::relativeTransform(int, int, int);
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinNew(const MatrixBase< Matrix<double, 3, Eigen::Dynamic> >&, int, int, int, int);
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardJacV(const GradientVar<double, Eigen::Dynamic, Eigen::Dynamic>&, int, int, int, bool, int);
template DLLEXPORT_RBM GradientVar<double, Eigen::Dynamic, Eigen::Dynamic> RigidBodyManipulator::forwardKinPositionGradient(int, int, int, int);

template DLLEXPORT_RBM void RigidBodyManipulator::HandC(double* const, double * const, MatrixBase< Map<MatrixXd> > * const, MatrixBase< Map<MatrixXd> > &, MatrixBase< Map<VectorXd> > &, MatrixBase< Map<MatrixXd> > *, MatrixBase< Map<MatrixXd> > *, MatrixBase< Map<MatrixXd> > * const);
template DLLEXPORT_RBM void RigidBodyManipulator::HandC(double* const, double * const, MatrixBase< MatrixXd > * const, MatrixBase< MatrixXd > &, MatrixBase< VectorXd > &, MatrixBase< MatrixXd > *, MatrixBase< MatrixXd > *, MatrixBase< MatrixXd > * const);

