#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>

#define INF -2147483648
using namespace Eigen;

class RigidBody;

class RigidBodyManipulator 
{
public:
  int num_dof; 
  int NB;  // featherstone bodies
  int num_bodies;  // rigid body objects
  int *pitch;
  int *parent;
  int *dofnum;
  double* joint_limit_min;
  double* joint_limit_max;
  
  MatrixXd* Xtree;
  MatrixXd* I;
  VectorXd a_grav;
  
  VectorXd* S;
  MatrixXd* Xup;
  VectorXd* v;
  VectorXd* avp;
  VectorXd* fvp;
  MatrixXd* IC;
  
  //Variables for gradient calculations
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
      
  RigidBody* bodies;
  bool initialized;
  bool kinematicsInit;
  double *cached_q, *cached_qd;
  int secondDerivativesCached;

  RigidBodyManipulator(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1);
  ~RigidBodyManipulator(void);
  
  void compile(void);  // call me after the model is loaded
  void doKinematics(double* q, bool b_compute_second_derivatives=false, double* qd=NULL);

  template <typename Derived>
  void getCOM(MatrixBase<Derived> &com);

  template <typename Derived>
  void getCOMJac(MatrixBase<Derived> &J);

  template <typename Derived>
  void getCOMJacDot(MatrixBase<Derived>& Jdot);

  template <typename Derived>
  void getCOMdJac(MatrixBase<Derived> &dJ);

  template <typename DerivedA, typename DerivedB>
  void forwardKin(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x);

  template <typename DerivedA, typename DerivedB>
  void forwardJacDot(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &Jdot);

  template <typename DerivedA, typename DerivedB>
  void forwardJac(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &J);

  template <typename DerivedA, typename DerivedB>
  void forwarddJac(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &dJ);

  void snoptIKfun( VectorXd q, VectorXd q0, VectorXd q_nom, MatrixXd Q, int narg, int* body_ind, VectorXd* pts, VectorXd* f, MatrixXd* G);

  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
  void HandC(double* const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH, MatrixBase<DerivedE> *dC);

};

#include "RigidBody.h"

#endif
