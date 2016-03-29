#ifndef _RIGIDBODYFRAME_H_
#define _RIGIDBODYFRAME_H_

#include "drake/util/testUtil.h"

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
  RigidBodyFrame(const std::string& _name,
                 std::shared_ptr<RigidBody> _body,
                 const Eigen::Isometry3d& _transform_to_body)
      : name(_name),
        body(_body),
        transform_to_body(_transform_to_body),
        frame_index(0) {}

  /**
   * A constructor where the transform-to-body is specified using
   * Euler angles.
   */
  RigidBodyFrame(const std::string& _name,
                 std::shared_ptr<RigidBody> _body,
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

  bool Compare(const RigidBodyFrame & rbf, std::string * explanation = nullptr) const {
    bool result = true;

    if (name.compare(rbf.name) != 0) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Names do not match: " << name << " vs. " << rbf.name;
        *explanation = ss.str();
      }
      result = false;
    }

    if (result && *body != *rbf.body) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Bodies do not match:\n"
              << "  - frame 1:\n" << *body << "\n"
              << "  - frame 2:\n" << *rbf.body;
        *explanation = ss.str();
      }
      result = false;
    }

    if (result) {
       try {
        valuecheckMatrix(transform_to_body.matrix(), rbf.transform_to_body.matrix(), std::numeric_limits<double>::epsilon());
      } catch(std::runtime_error re) {
        if (explanation != nullptr) {
          std::stringstream ss;
          ss << "Transform-to-body mismatch!" << std::endl
             << "  - name: " << name << "\n"
             << "  - tree 1:\n" << transform_to_body.matrix() << std::endl
             << "  - tree 2:\n" << rbf.transform_to_body.matrix() << std::endl
             << "  - details:\n" << re.what();
          *explanation = ss.str();
        }
        result = false;
      }
    }

    if (result && frame_index != rbf.frame_index) {
      if (explanation != nullptr) {
        std::stringstream ss;
        ss << "Frame indices do not match:\n"
           << "  - frame 1:\n" << frame_index << "\n"
           << "  - frame 2:\n" << rbf.frame_index;
        *explanation = ss.str();
      }
      result = false;
    }

    return result;
  }

  /*!
   * Overload operator== to check whether two RigidBodyFrame objects are equal.
   */
  friend DRAKERBM_EXPORT bool operator==(const RigidBodyFrame & rbf1, const RigidBodyFrame & rbf2) {
    return rbf1.Compare(rbf2);
  }

  /*!
   * Overload operator!= to check whether two RigidBodyFrame objects are unequal.
   */
  friend DRAKERBM_EXPORT bool operator!=(const RigidBodyFrame & rbf1, const RigidBodyFrame & rbf2) {
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
