#ifndef DRAKE_SYSTEMS_PLANTS_RIGIDBODYFRAME_H_
#define DRAKE_SYSTEMS_PLANTS_RIGIDBODYFRAME_H_

namespace tinyxml2 {
class XMLElement;
}
class RigidBodyTree;

class DRAKERBM_EXPORT RigidBodyFrame {
 public:
  /**
   * A constructor where the transform-to-body is specified using an
   * Eigen::Isometry3d matrix.
   */
  RigidBodyFrame(const std::string& _name, std::shared_ptr<RigidBody> _body,
                 const Eigen::Isometry3d& _transform_to_body)
      : name(_name),
        body(_body),
        transform_to_body(_transform_to_body),
        frame_index(0) {}

  /**
   * A constructor where the transform-to-body is specified using
   * Euler angles.
   */
  RigidBodyFrame(const std::string& _name, std::shared_ptr<RigidBody> _body,
                 const Eigen::Vector3d& xyz = Eigen::Vector3d::Zero(),
                 const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero())
      : name(_name), body(_body), frame_index(0) {
    transform_to_body.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  /**
   * A constructor based on a URDF specification.
   */
  RigidBodyFrame(RigidBodyTree* tree, tinyxml2::XMLElement* link_reference,
                 tinyxml2::XMLElement* pose = nullptr,
                 std::string name = "");  // parse from URDF

  /**
   * The default constructor.
   */
  RigidBodyFrame()
      : name(""),
        body(nullptr),
        transform_to_body(Eigen::Isometry3d::Identity()),
        frame_index(0) {}

  std::string name;
  std::shared_ptr<RigidBody> body;
  Eigen::Isometry3d transform_to_body;
  int frame_index;  // this will be negative, but will also be gone soon!

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // DRAKE_SYSTEMS_PLANTS_RIGIDBODYFRAME_H_
