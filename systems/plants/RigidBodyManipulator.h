#include <Eigen/Dense>
#include "RigidBody.h"
using namespace Eigen;
//using namespace std;

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
  Matrix4d TdTmult;
  
  // preallocate for COM functions
  Vector3d com;
  Vector3d bc;
  MatrixXd Jcom;
  MatrixXd bJ;
  MatrixXd dJcom;
  MatrixXd bdJ;
  
  RigidBody* bodies;
  bool kinematicsInit;
  double *cached_q;
  int secondDerivativesCached;
  

  RigidBodyManipulator(int n);
  ~RigidBodyManipulator(void);
  
  void doKinematics(double* q, int b_compute_second_derivatives);

  Vector3d getCOM(void);
  MatrixXd getCOMJac(void);
  MatrixXd getCOMJacDot(void);
  
  MatrixXd forwardKin(const int body_ind, const MatrixXd pts, const bool include_rotations);
  MatrixXd forwardJac(const int body_ind, const MatrixXd pts, const bool include_rotations);
  MatrixXd forwardJacDot(const int body_ind, const MatrixXd pts, const bool include_rotations);

  void snoptIKfun( VectorXd q, VectorXd q0, VectorXd q_nom, MatrixXd Q, int narg, int* body_ind, VectorXd* pts, VectorXd* f, MatrixXd* G);

};
