#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

//#define BULLET_COLLISION  // adds a bunch of dependencies, which are not necessary for all functionality

#include <Eigen/Dense>
#include <set>
#include <vector>

#ifdef BULLET_COLLISION
#include <btBulletCollisionCommon.h>
#endif


#define INF -2147483648
using namespace Eigen;

class RigidBody;

const std::set<int> emptyIntSet;

class RigidBodyManipulator 
{
public:
  RigidBodyManipulator(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1);
  ~RigidBodyManipulator(void);
  
  void compile(void);  // call me after the model is loaded
  void doKinematics(double* q, bool b_compute_second_derivatives=false, double* qd=NULL);

  template <typename Derived>
  void getCOM(MatrixBase<Derived> &com);

  template <typename Derived>
  void getCOMJac(MatrixBase<Derived> &J);

  template <typename Derived>
  void getCOMJacDot(MatrixBase<Derived> &Jdot);

  template <typename Derived>
  void getCOMdJac(MatrixBase<Derived> &dJ);

  int getNumContacts(const std::set<int> &body_idx = emptyIntSet);

  template <typename Derived>
  void getContactPositions(MatrixBase<Derived> &pos, const std::set<int> &body_idx = emptyIntSet);
  
  template <typename Derived>
  void getContactPositionsJac(MatrixBase<Derived> &J, const std::set<int> &body_idx = emptyIntSet);
  
  template <typename Derived>
  void getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const std::set<int> &body_idx = emptyIntSet);

  template <typename DerivedA, typename DerivedB>
  void forwardKin(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x);

  template <typename DerivedA, typename DerivedB>
  void forwardJacDot(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &Jdot);

  template <typename DerivedA, typename DerivedB>
  void forwardJac(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &J);

  template <typename DerivedA, typename DerivedB>
  void forwarddJac(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &dJ);

  template <typename DerivedA, typename DerivedB>
  void bodyKin(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x);

  void snoptIKfun( VectorXd q, VectorXd q0, VectorXd q_nom, MatrixXd Q, int narg, int* body_ind, VectorXd* pts, VectorXd* f, MatrixXd* G);

  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
  void HandC(double* const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH=NULL, MatrixBase<DerivedE> *dC=NULL);

public:
  int num_dof;
  double* joint_limit_min;
  double* joint_limit_max;

  // Rigid body objects
  int num_bodies;  // rigid body objects
  RigidBody* bodies;

  // featherstone data structure
  int NB;  // featherstone bodies
  int *pitch;
  int *parent;
  int *dofnum;
  double* damping;
  MatrixXd* Xtree;
  MatrixXd* I;
  VectorXd a_grav;

  double *cached_q, *cached_qd;  // these should be private

  // preallocate for approximateIK
  VectorXd qtmp;


private:
  // variables for featherstone dynamics
  VectorXd* S;
  MatrixXd* Xup;
  VectorXd* v;
  VectorXd* avp;
  VectorXd* fvp;
  MatrixXd* IC;

  //Variables for gradient calculations
  MatrixXd dTdTmult;
  MatrixXd* dXupdq;
  MatrixXd** dIC;

  MatrixXd* dvdq;
  MatrixXd* dvdqd;
  MatrixXd* davpdq;
  MatrixXd* davpdqd;
  MatrixXd* dfvpdq;
  MatrixXd* dfvpdqd;
  MatrixXd dvJdqd_mat;
  MatrixXd dcross;

  // preallocate for COM functions
  Vector3d bc;
  MatrixXd bJ;
  MatrixXd bdJ;

  int num_contact_pts;
  bool initialized;
  bool kinematicsInit;
  int secondDerivativesCached;

#ifdef BULLET_COLLISION
protected:
  btDefaultCollisionConfiguration bt_collision_configuration;
  btCollisionDispatcher bt_collision_dispatcher;
  btDbvtBroadphase bt_collision_broadphase;

public:
  btCollisionWorld bt_collision_world;

  void updateCollisionObjects(int body_ind);

  bool getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals);
#endif
};

#include "RigidBody.h"


#ifdef mex_h
  // helper function for shuffling debugging data back into matlab
  template <int Rows, int Cols>
  mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
  {
    mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
    if (m.rows()*m.cols()>0)
    	memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
    return pm;
  }
#endif

#ifdef _GUROBI_CPP_H

template <typename DerivedA,typename DerivedB>
int myGRBaddconstrs(GRBmodel *model, MatrixBase<DerivedA> const & A, MatrixBase<DerivedB> const & b, char sense, double sparseness_threshold = 1e-10)
{
  int i,j,nnz,error;
/*
  // todo: it seems like I should just be able to do something like this:
  SparseMatrix<double,RowMajor> sparseAeq(Aeq.sparseView());
  sparseAeq.makeCompressed();
  error = GRBaddconstrs(model,nq_con,sparseAeq.nonZeros(),sparseAeq.InnerIndices(),sparseAeq.OuterStarts(),sparseAeq.Values(),beq.data(),NULL);
*/

  int *cind = new int[A.cols()];
  double* cval = new double[A.cols()];
  for (i=0; i<A.rows(); i++) {
    nnz=0;
    for (j=0; j<A.cols(); j++) {
      if (abs(A(i,j))>sparseness_threshold) {
        cval[nnz] = A(i,j);
        cind[nnz++] = j;
      }
    }
    error = GRBaddconstr(model,nnz,cind,cval,sense,b(i),NULL);
    if (error) break;
  }

  delete[] cind;
  delete[] cval;
  return error;
}

#endif

#endif
