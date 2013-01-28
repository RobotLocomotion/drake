#ifndef _RIGIDBODY
#define _RIGIDBODY
#include <Eigen/Dense>
using namespace Eigen;

class RigidBody {
public:
  int nq;
  MatrixXd dTdq;
  MatrixXd ddTdqdq;
  Matrix4d T;
  Matrix4d T_body_to_joint;

  double mass;
  Vector4d com;  // this actually stores [com;1] (because that's what's needed in the kinematics functions)
  
  //Need to initialize these
  Matrix4d Ttree;
  int pitch;
  int dofnum;
  int parent;

  RigidBody() {
    this->nq = 0;
    this->mass = 0.0;
    this->com << Vector3d::Zero(), 1;
  }
  
  void setN(int n) {    
    this->nq = n;
    this->T = Matrix4d::Identity();
    this->dTdq = MatrixXd::Zero(4*n,4);
    this->ddTdqdq = MatrixXd::Zero(4*n*n,4);
    this->Ttree = Matrix4d::Zero();
    this->T_body_to_joint = Matrix4d::Identity();
  }
  
  ~RigidBody() {
  }
  
};
#endif
