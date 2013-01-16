#ifndef _PLANARRIGIDBODY
#define _PLANARRIGIDBODY
#include <Eigen/Dense>
using namespace Eigen;

class PlanarRigidBody {
public:
  int nq;
  MatrixXd T;
  MatrixXd dTdq;
  MatrixXd ddTdqdq;
  
  //Need to initialize these
  Matrix3d Ttree;
  int jsign;
  int dofnum;
  int jcode;
  int parent;

  PlanarRigidBody() {
    nq = 0;
  }
  
  void setN(int n) {    
    nq = n;
    T = Matrix3d::Zero();
    dTdq = MatrixXd::Zero(3*n,3);
    ddTdqdq = MatrixXd::Zero(3*n*n,3);
    Ttree = Matrix3d::Zero();
  }
  
  ~PlanarRigidBody() {
  }
  
};
#endif