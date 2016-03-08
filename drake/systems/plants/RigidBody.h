#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <map>
#include <Eigen/StdVector>
#include <memory>
#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/drakeRBM_export.h"

class DRAKERBM_EXPORT RigidBody {
 private:
  std::unique_ptr<DrakeJoint> joint;
  DrakeCollision::bitmask collision_filter_group;
  DrakeCollision::bitmask collision_filter_ignores;

 public:
  RigidBody();

  void setJoint(std::unique_ptr<DrakeJoint> joint);
  const DrakeJoint& getJoint() const;

  bool hasParent() const;

  void addVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& getVisualElements() const;

  void setCollisionFilter(const DrakeCollision::bitmask& group,
                          const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const {
    return collision_filter_group;
  };
  void setCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group = group;
  };

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const {
    return collision_filter_ignores;
  };
  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores) {
    this->collision_filter_ignores = ignores;
  };

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_group |= group;
  };
  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores |= group;
  };
  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group) {
    this->collision_filter_ignores &= ~group;
  };

  bool adjacentTo(const std::shared_ptr<RigidBody>& other) const {
    return ((parent == other && !(joint && joint->isFloating())) ||
            (other->parent.get() == this &&
             !(other->joint && other->joint->isFloating())));
  };

  bool collidesWith(const std::shared_ptr<RigidBody>& other) const {
    bool ignored =
        this == other.get() || adjacentTo(other) ||
        (collision_filter_group & other->getCollisionFilterIgnores()).any() ||
        (other->getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  };

  bool appendCollisionElementIdsFromThisBody(
      const std::string& group_name,
      std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(
      std::vector<DrakeCollision::ElementId>& ids) const;

 public:
  std::string linkname;
  std::string model_name;  // todo: replace robotnum w/ model_name
  int robotnum;            // uses 0-index. starts from 0
  // note: it's very ugly, but parent,dofnum,and pitch also exist currently
  // (independently) at the RigidBodyTree level to represent the featherstone
  // structure.  this version is for the kinematics.
  std::shared_ptr<RigidBody> parent;
  int body_index;
  int position_num_start;
  int velocity_num_start;

  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector<DrakeCollision::ElementId> collision_element_ids;
  std::map<std::string, std::vector<DrakeCollision::ElementId> >
      collision_element_groups;

  Eigen::Matrix3Xd contact_pts;

  double mass;
  Eigen::Vector3d com;
  Eigen::Matrix<double, TWIST_SIZE, TWIST_SIZE> I;

  friend std::ostream& operator<<(std::ostream& out, const RigidBody& b);

  // FIXME: move to a better place:
  class DRAKERBM_EXPORT CollisionElement : public DrakeCollision::Element {
   public:
    CollisionElement(const CollisionElement& other);
    CollisionElement(const Eigen::Isometry3d& T_element_to_link,
                     std::shared_ptr<RigidBody> body);
    CollisionElement(const DrakeShapes::Geometry& geometry,
                     const Eigen::Isometry3d& T_element_to_link,
                     std::shared_ptr<RigidBody> body);
    virtual ~CollisionElement(){};

    virtual CollisionElement* clone() const;

    const std::shared_ptr<RigidBody>& getBody() const;

    virtual bool collidesWith(const DrakeCollision::Element* other) const;

   protected:
    std::shared_ptr<RigidBody> body;

   public:
#ifndef SWIG
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
  };

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

#endif
