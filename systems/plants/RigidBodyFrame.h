#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

class DLLEXPORT_RBM RigidBodyFrame {
public:
  std::string name;
  int body_ind;
  Eigen::Matrix4d Ttree;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // _RIGIDBODYFRAME_H_
