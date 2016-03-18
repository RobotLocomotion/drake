#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

#include "drake/util/testUtil.h"

namespace tinyxml2 {
class XMLElement;
}
class RigidBodyTree;

class DRAKERBM_EXPORT RigidBodyFrame {
 public:
  RigidBodyFrame(const std::string& _name,
                 std::shared_ptr<RigidBody> _body,
                 const Eigen::Isometry3d& _transform_to_body)
      : name(_name),
        body(_body),
        transform_to_body(_transform_to_body),
        frame_index(0) {}
  RigidBodyFrame(const std::string& _name,
                 std::shared_ptr<RigidBody> _body,
                 const Eigen::Vector3d& xyz = Eigen::Vector3d::Zero(),
                 const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero())
      : name(_name), body(_body), frame_index(0) {
    transform_to_body.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }
  RigidBodyFrame(RigidBodyTree* tree, tinyxml2::XMLElement* link_reference,
                 tinyxml2::XMLElement* pose = nullptr,
                 std::string name = "");  // parse from URDF
  RigidBodyFrame()
      : name(""),
        body(nullptr),
        transform_to_body(Eigen::Isometry3d::Identity()),
        frame_index(0) {}

  #define PRINT_STMT(x) std::cout << "RigidBodyFrame: EQUALS: " << x << std::endl;

  /*!
   * Overload operator== to check whether two RigidBodyFrame objects are equal.
   */
  friend bool operator==(const RigidBodyFrame & rbf1, const RigidBodyFrame & rbf2) {
    bool result = true;

    if (rbf1.name.compare(rbf2.name) != 0) {
      PRINT_STMT("Names do not match: " << rbf1.name << " vs. " << rbf2.name)
      result = false;
    }

    if (result && *rbf1.body != *rbf2.body) {
      PRINT_STMT("Bodies do not match:\n"
              << "  - frame 1:\n" << *rbf1.body << "\n"
              << "  - frame 2:\n" << *rbf2.body)
      result = false;
    }

    if (result) {
       try {
        valuecheckMatrix(rbf1.transform_to_body.matrix(), rbf2.transform_to_body.matrix(), 1e-10);
      } catch(std::runtime_error re) {
        PRINT_STMT("Transform-to-body mismatch!" << std::endl
                    << "  - name: " << rbf1.name << "\n"
                    << "  - tree 1:\n" << rbf1.transform_to_body.matrix() << std::endl
                    << "  - tree 2:\n" << rbf2.transform_to_body.matrix() << std::endl
                    << "  - details:\n" << re.what())
        result = false;
      }
    }

    if (result && rbf1.frame_index != rbf2.frame_index) {
      PRINT_STMT("Frame indices do not match:\n"
              << "  - frame 1:\n" << rbf1.frame_index << "\n"
              << "  - frame 2:\n" << rbf2.frame_index)
      result = false;
    }

    return result;
  }
  #undef PRINT_STMT

  /*!
   * Overload operator!= to check whether two RigidBodyFrame objects are unequal.
   */
  friend bool operator!=(const RigidBodyFrame & rbf1, const RigidBodyFrame & rbf2) {
    return !operator==(rbf1, rbf2);
  }

  std::string name;
  std::shared_ptr<RigidBody> body;
  Eigen::Isometry3d transform_to_body;
  int frame_index;  // this will be negative, but will also be gone soon!

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // _RIGIDBODYFRAME_H_
