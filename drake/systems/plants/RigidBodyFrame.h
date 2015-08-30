#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

class DLLEXPORT_RBM RigidBodyFrame {
public:
  RigidBodyFrame(const std::string& _name, const std::shared_ptr<RigidBody>& _body, const Eigen::Matrix4d& _Ttree)
   : name(_name), body(_body), Ttree(_Ttree), frame_index(0) {}
  RigidBodyFrame(const std::string& _name, const std::shared_ptr<RigidBody>& _body, const Eigen::Vector3d& xyz=Eigen::Vector3d::Zero(), const Eigen::Vector3d& rpy=Eigen::Vector3d::Zero())
          : name(_name), body(_body), Ttree(), frame_index(0) {
    Ttree << rpy2rotmat(rpy), Eigen::Vector3d::Zero(), 0,0,0,1;
  }
  RigidBodyFrame() : name(""), body(nullptr), Ttree(Eigen::Matrix4d::Identity()), frame_index(0) {}

  std::string name;
  std::shared_ptr<RigidBody> body;
  Eigen::Matrix4d Ttree;
  int frame_index;  // this will be negative, but will also be gone soon!

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // _RIGIDBODYFRAME_H_
