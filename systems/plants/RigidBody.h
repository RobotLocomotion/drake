#ifndef _RIGIDBODY
#define _RIGIDBODY
#include <Eigen/Dense>
using namespace Eigen;

class RigidBody {
public:
  std::string linkname;
  std::string jointname;
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

  RigidBody() {
    mass = 0.0;
    com << Vector3d::Zero(), 1;
  }
  
  void setN(int n) {    
    T = Matrix4d::Identity();
    dTdq = MatrixXd::Zero(4*n,4);
    ddTdqdq = MatrixXd::Zero(4*n*n,4);
    Ttree = Matrix4d::Identity();
    T_body_to_joint = Matrix4d::Identity();
  }
  
  ~RigidBody() {
  }
  
};
#endif
