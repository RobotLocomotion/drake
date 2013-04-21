#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>

#define INF -2147483648
using namespace Eigen;

class RigidBody;

class RigidBodyManipulator 
{
public:
  int NB;
  int *pitch;
  int *parent;
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
  
  // preallocate matrices used in doKinematics
  Matrix4d TJ;
  Matrix4d dTJ;
  Matrix4d ddTJ;
  Matrix4d Tbinv;
  Matrix4d Tb;
  Matrix4d Tmult;
  Matrix4d dTmult;
  Matrix4d dTdotmult;
  Matrix4d TdTmult;
  
  // preallocate for COM functions
  Vector3d com;
  Vector3d bc;
  MatrixXd Jcom;
  MatrixXd Jcomdot;
  MatrixXd bJ;
  MatrixXd dJcom;
  MatrixXd bdJ;
  
  RigidBody* bodies;
  bool initialized;
  bool kinematicsInit;
  double *cached_q, *cached_qd;
  int secondDerivativesCached;

  RigidBodyManipulator(int n);
  ~RigidBodyManipulator(void);
  
  void compile(void);  // call me after the model is loaded
  void doKinematics(double* q, bool b_compute_second_derivatives, double* qd=NULL);

  Vector3d getCOM(void);
  MatrixXd getCOMJac(void);
  MatrixXd getCOMJacDot(void);
  MatrixXd getCOMdJac(void);
  

  MatrixXd forwardKin(const int body_ind, const MatrixXd pts, const int rotation_type);
  MatrixXd forwardJac(const int body_ind, const MatrixXd pts, const int rotation_type);
  MatrixXd forwardJacDot(const int body_ind, const MatrixXd pts);
  MatrixXd forwarddJac(const int body_ind, const MatrixXd pts);

  void snoptIKfun( VectorXd q, VectorXd q0, VectorXd q_nom, MatrixXd Q, int narg, int* body_ind, VectorXd* pts, VectorXd* f, MatrixXd* G);

};

#include "RigidBody.h"

#endif