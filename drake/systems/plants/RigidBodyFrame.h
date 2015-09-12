#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

class DLLEXPORT_RBM RigidBodyFrame {
public:
  RigidBodyFrame(const std::string& _name, const std::shared_ptr<RigidBody>& _body, const Eigen::Matrix4d& _transform_to_body)
   : name(_name), body(_body), transform_to_body(_transform_to_body), frame_index(0) {}
  RigidBodyFrame(const std::string& _name, const std::shared_ptr<RigidBody>& _body, const Eigen::Vector3d& xyz=Eigen::Vector3d::Zero(), const Eigen::Vector3d& rpy=Eigen::Vector3d::Zero())
          : name(_name), body(_body), frame_index(0) {
    transform_to_body << rpy2rotmat(rpy), xyz, 0,0,0,1;
  }
  RigidBodyFrame() : name(""), body(nullptr), transform_to_body(Eigen::Matrix4d::Identity()), frame_index(0) {}

  std::string name;
  std::shared_ptr<RigidBody> body;
  Eigen::Matrix4d transform_to_body;  // todo: change this to Isometry3d
  int frame_index;  // this will be negative, but will also be gone soon!

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // _RIGIDBODYFRAME_H_
