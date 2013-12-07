#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

class RigidBodyFrame {
public:
  std::string name;
  int body_ind;
  Eigen::Matrix4d T;
};

#endif // _RIGIDBODYFRAME_H_
