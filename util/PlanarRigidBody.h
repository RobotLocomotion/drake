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
    this->nq = 0;
  }
  
  void setN(int n) {    
    this->nq = n;
    this->T = Matrix3d::Zero();
    this->dTdq = MatrixXd::Zero(3*n,3);
    this->ddTdqdq = MatrixXd::Zero(3*n*n,3);
    this->Ttree = Matrix3d::Zero();
  }
  
  ~PlanarRigidBody() {
  }
  
};
#endif