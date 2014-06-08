#include <mex.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>
//#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
//#include <Eigen/Sparse>

#ifdef USE_MAPS

#include "mexmaps/MapLib.hpp"
#include <maps/ViewBase.hpp>

#endif

//#define TEST_FAST_QP

#include "drake/fastQP.h"
#include "drake/gurobiQP.h"
#include "drake/RigidBodyManipulator.h"

const int m_surface_tangents = 2;  // number of faces in the friction cone approx
const double REG = 1e-8;

using namespace std;

struct QPControllerData {
  GRBenv *env;
  RigidBodyManipulator* r;
  void* multi_robot; // optional multi rigid body system
  double w; // objective function weight
  double slack_limit; // maximum absolute magnitude of acceleration slack variable values
  VectorXd umin,umax;
  void* map_ptr;
  double Kp_com,Kd_com,Kp_k; // COM-z and angular momentum (k) PD gains 
  set<int> active;

  // preallocate memory
  MatrixXd H, H_float, H_act;
  VectorXd C, C_float, C_act;
  MatrixXd B, B_act;
  MatrixXd J, Jdot;
  MatrixXd J_xy, Jdot_xy;
  MatrixXd Hqp;
  RowVectorXd fqp;
  
  // momentum controller-specific
  MatrixXd Ag, Agdot; // centroidal momentum matrix
  MatrixXd W_hdot; // quadratic cost for momentum control: (hdot_des - hdot)'*W*(hdot_des - hdot)
  VectorXd w_qdd; 
  double w_grf; 
  double w_slack; 
  int num_spatial_accel_constraints;
  double Kp, Kd; // COM-z PD gains, for momentum controller
  double mass; // total robot mass

  int *vbasis;
  int *cbasis;
  int vbasis_len;
  int cbasis_len;
};

// helper function for shuffling debugging data back into matlab
template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
  if (m.rows()*m.cols()>0)
  	memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
  return pm;
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetProperty(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","QPControllermex is trying to load object property '%s', but failed.", propname);
  return pm;
}

mxArray* myGetField(const mxArray* pobj, const char* propname)
{
  mxArray* pm = mxGetField(pobj,0,propname);
  if (!pm) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","QPControllermex is trying to load object field '%s', but failed.", propname);
  return pm;
}

typedef struct _support_state_element
{
  int body_idx;
  set<int> contact_pt_inds;
  int contact_surface;
} SupportStateElement;


bool inSupport(vector<SupportStateElement> supports, int body_idx) {
  for (int i=0; i<supports.size(); i++) {
    if (supports[i].body_idx == body_idx)
      return true;
  }
  return false;
}

template <typename DerivedA, typename DerivedB>
void getRows(set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (rows.size()==M.rows()) {
    Msub = M; 
    return;
  }
  
  int i=0;
  for (set<int>::iterator iter=rows.begin(); iter!=rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
void getCols(set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (cols.size()==M.cols()) {
    Msub = M;
    return;
  }
  int i=0;
  for (set<int>::iterator iter=cols.begin(); iter!=cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height)
{
  if (map_ptr) {
#ifdef USE_MAPS    
    Vector3f floatPos, floatNormal;
    auto state = static_cast<mexmaps::MapHandle*>(map_ptr);
    if (state != NULL) {
      auto view = state->getView();
      if (view != NULL) {
        if (view->getClosest(contact_pos.cast<float>(),floatPos,floatNormal)) {
          pos = floatPos.cast<double>();
          if (normal) *normal = floatNormal.cast<double>();
          return;
        }
      }
    }
#endif      
  } else {
//    mexPrintf("Warning: using 0,0,1 as normal\n");
    pos << contact_pos.topRows(2), terrain_height;
    if (normal) *normal << 0,0,1;
  }
}

void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1 - normal(2) < 10e-8) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1,0,0;
  } else { // now the general case
    t1 << normal(2), -normal(1), 0; // normal.cross([0;0;1])
    t1 /= sqrt(normal(1)*normal(1) + normal(2)*normal(2));
  }
      
  t2 = t1.cross(normal);
      
  for (int k=0; k<m_surface_tangents; k++) {
    theta = k*M_PI/m_surface_tangents;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}

int contactPhi(struct QPControllerData* pdata, SupportStateElement& supp, VectorXd &phi, double terrain_height)
{
  RigidBody* b = &(pdata->r->bodies[supp.body_idx]);
	int nc = supp.contact_pt_inds.size();
	phi.resize(nc);

	if (nc<1) return nc;

  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;

  int i=0;
  for (set<int>::iterator pt_iter=supp.contact_pt_inds.begin(); pt_iter!=supp.contact_pt_inds.end(); pt_iter++) {
  	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());

    #ifdef BULLET_COLLISION
    if (supp.contact_surface!=-1 && pdata->multi_robot) {
      // do bullet rigid body collision check

      auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);
      
      if (supp.contact_surface < -1) {
        // check entire world
        multi_robot->getPointCollision(supp.body_idx,*pt_iter,pos,posB,normal);
      }
      else {
        // do pairwise check with specified contact surface
        multi_robot->getPairwisePointCollision(supp.body_idx,supp.contact_surface,*pt_iter,pos,posB,normal);
      }
      
      pos = posB-pos; // now -rel_pos in matlab version
    }
    else {
    #endif
      tmp = b->contact_pts.col(*pt_iter);
      pdata->r->forwardKin(supp.body_idx,tmp,0,contact_pos);
      collisionDetect(pdata->map_ptr,contact_pos,pos,NULL,terrain_height);
      pos -= contact_pos;  // now -rel_pos in matlab version
    #ifdef BULLET_COLLISION
    }
    #endif
  
		phi(i) = pos.norm();
		if (pos.dot(normal)>0)
			phi(i)=-phi(i);
		i++;
  }
	return nc;
}

int contactConstraints(struct QPControllerData* pdata, int nc, vector<SupportStateElement>& supp, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height)
{
  int j, k=0, nq = pdata->r->num_dof;

  n.resize(nc,nq);
  D.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  
  for (vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[iter->body_idx]);
    if (nc>0) {
      for (set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
      	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        pdata->r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        pdata->r->forwardJac(iter->body_idx,tmp,0,J);

        #ifdef BULLET_COLLISION
        if (iter->contact_surface!=-1 && pdata->multi_robot) {
          // do bullet rigid body collision check
          auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);

          if (iter->contact_surface < -1) {
            // check entire world
            multi_robot->getPointCollision(iter->body_idx,(int)*pt_iter,pos,posB,normal);
          }
          else {
            // do pairwise check with specified contact surface
            multi_robot->getPairwisePointCollision(iter->body_idx,iter->contact_surface,*pt_iter,pos,posB,normal);
          }
        }
        else {
        #endif
          collisionDetect(pdata->map_ptr,contact_pos,pos,&normal,terrain_height);
        #ifdef BULLET_COLLISION
        }
        #endif

        surfaceTangents(normal,d);

        n.row(k) = normal.transpose()*J;
        for (j=0; j<m_surface_tangents; j++) {
          D.col(2*k*m_surface_tangents+j) = J.transpose()*d.col(j);
          D.col((2*k+1)*m_surface_tangents+j) = -D.col(2*k*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(iter->body_idx,tmp,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}


int contactConstraintsBV(struct QPControllerData* pdata, int nc, double mu, vector<SupportStateElement>& supp, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height)
{
  int j, k=0, nq = pdata->r->num_dof;

  B.resize(3,nc*2*m_surface_tangents);
  JB.resize(nq,nc*2*m_surface_tangents);
  Jp.resize(3*nc,nq);
  Jpdot.resize(3*nc,nq);
  
  Vector3d contact_pos,pos,posB,normal; Vector4d tmp;
  MatrixXd J(3,nq);
  Matrix<double,3,m_surface_tangents> d;
  double norm = sqrt(1+mu*mu); // because normals and ds are orthogonal, the norm has a simple form
  
  for (vector<SupportStateElement>::iterator iter = supp.begin(); iter!=supp.end(); iter++) {
    RigidBody* b = &(pdata->r->bodies[iter->body_idx]);
    if (nc>0) {
      for (set<int>::iterator pt_iter=iter->contact_pt_inds.begin(); pt_iter!=iter->contact_pt_inds.end(); pt_iter++) {
      	if (*pt_iter<0 || *pt_iter>=b->contact_pts.cols()) mexErrMsgIdAndTxt("DRC:QPControllermex:BadInput","requesting contact pt %d but body only has %d pts",*pt_iter,b->contact_pts.cols());
        tmp = b->contact_pts.col(*pt_iter);
        pdata->r->forwardKin(iter->body_idx,tmp,0,contact_pos);
        pdata->r->forwardJac(iter->body_idx,tmp,0,J);

        #ifdef BULLET_COLLISION
        if (iter->contact_surface!=-1 && pdata->multi_robot) {
          // do bullet rigid body collision check
          auto multi_robot = static_cast<RigidBodyManipulator*>(pdata->multi_robot);

          if (iter->contact_surface < -1) {
            // check entire world
            multi_robot->getPointCollision(iter->body_idx,(int)*pt_iter,pos,posB,normal);
          }
          else {
            // do pairwise check with specified contact surface
            multi_robot->getPairwisePointCollision(iter->body_idx,iter->contact_surface,*pt_iter,pos,posB,normal);
          }
        }
        else {
        #endif
          collisionDetect(pdata->map_ptr,contact_pos,pos,&normal,terrain_height);
        #ifdef BULLET_COLLISION
        }
        #endif

        surfaceTangents(normal,d);
        for (j=0; j<m_surface_tangents; j++) {
          B.col(2*k*m_surface_tangents+j) = (normal + mu*d.col(j)) / norm; 
          B.col((2*k+1)*m_surface_tangents+j) = (normal - mu*d.col(j)) / norm; 
    
          JB.col(2*k*m_surface_tangents+j) = J.transpose()*B.col(2*k*m_surface_tangents+j);
          JB.col((2*k+1)*m_surface_tangents+j) = J.transpose()*B.col((2*k+1)*m_surface_tangents+j);
        }

        // store away kin sols into Jp and Jpdot
        // NOTE: I'm cheating and using a slightly different ordering of J and Jdot here
        Jp.block(3*k,0,3,nq) = J;
        pdata->r->forwardJacDot(iter->body_idx,tmp,0,J);
        Jpdot.block(3*k,0,3,nq) = J;
        
        k++;
      }
    }
  }
  
  return k;
}