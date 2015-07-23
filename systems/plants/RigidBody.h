#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <Eigen/StdVector>
#include <memory>
#include "DrakeJoint.h"

class DLLEXPORT_RBM IndexRange {
 public:
  int start;
  int length;

  bool operator<(const IndexRange& other) const {
    return start<other.start;
  }
};

class RigidBodyManipulator;

using namespace Eigen;

class DLLEXPORT_RBM RigidBody {
private:
  std::unique_ptr<DrakeJoint> joint;
  typedef Matrix<double, 6,1> Vector6d;
  DrakeCollision::bitmask collision_filter_group;
  DrakeCollision::bitmask collision_filter_ignores;

public:
  RigidBody();

  void setN(int nq, int nv);
  void computeAncestorDOFs(RigidBodyManipulator* model);

  void setupOldKinematicTree(RigidBodyManipulator* model);

  void setJoint(std::unique_ptr<DrakeJoint> joint);
  const DrakeJoint& getJoint() const;

  bool hasParent() const;

  void addVisualElement(const DrakeShapes::VisualElement& elements);

  const DrakeShapes::VectorOfVisualElements& getVisualElements() const;

  void setCollisionFilter(const DrakeCollision::bitmask& group,
                                  const DrakeCollision::bitmask& ignores);

  const DrakeCollision::bitmask& getCollisionFilterGroup() const { return collision_filter_group; };
  void setCollisionFilterGroup(const DrakeCollision::bitmask& group) { this->collision_filter_group = group; };

  const DrakeCollision::bitmask& getCollisionFilterIgnores() const { return collision_filter_ignores; };
  void setCollisionFilterIgnores(const DrakeCollision::bitmask& ignores) { this->collision_filter_ignores = ignores; };

  void addToCollisionFilterGroup(const DrakeCollision::bitmask& group) { this->collision_filter_group |= group; }; 
  void ignoreCollisionFilterGroup(const DrakeCollision::bitmask& group) { this->collision_filter_ignores |= group; };
  void collideWithCollisionFilterGroup(const DrakeCollision::bitmask& group) { this->collision_filter_ignores &= ~group; };

  bool adjacentTo(const std::shared_ptr<RigidBody>& other) const
  {
      return ((parent==other && !(joint && joint->isFloating())) ||
          (other->parent.get() == this && !(other->joint && other->joint->isFloating())));
  };

  bool collidesWith(const std::shared_ptr<RigidBody>& other) const
  {
    bool ignored = this == other.get() || adjacentTo(other) || (collision_filter_group & other->getCollisionFilterIgnores()).any() || (other->getCollisionFilterGroup() & collision_filter_ignores).any();
    return !ignored;
  };

  bool appendCollisionElementIdsFromThisBody(const std::string& group_name, std::vector<DrakeCollision::ElementId>& ids) const;

  bool appendCollisionElementIdsFromThisBody(std::vector<DrakeCollision::ElementId>& ids) const;

public:
  std::string linkname;
  std::string jointname; // FLOATINGBASE TODO: remove
  int robotnum; // uses 0-index. starts from 0
  static const std::set<int> defaultRobotNumSet;
// note: it's very ugly, but parent,dofnum,and pitch also exist currently (independently) at the rigidbodymanipulator level to represent the featherstone structure.  this version is for the kinematics.
  std::shared_ptr<RigidBody> parent;
  int body_index; // index in RBM bodies vector (set in compile())
  int position_num_start; // interpreted as start of position_num from Matlab
  int velocity_num_start;
  int floating; // FLOATINGBASE TODO: remove
  int pitch; // FLOATINGBASE TODO: remove
  Matrix4d Ttree;  // floatingbase TODO: replace with Isometry3d?
  Matrix4d T_body_to_joint;  // floatingbase TODO: replace with Isometry3d?

  DrakeShapes::VectorOfVisualElements visual_elements;

  std::vector< DrakeCollision::ElementId > collision_element_ids;
  std::map< std::string, std::vector<DrakeCollision::ElementId> > collision_element_groups;

  Matrix3Xd contact_pts;

  std::set<int> ancestor_dofs;
  std::set<int> ddTdqdq_nonzero_rows;
  std::set<IndexRange> ddTdqdq_nonzero_rows_grouped;

  Matrix4d T;    // floatingbase TODO: replace with Isometry3d?
  MatrixXd dTdq; // floatingbase TODO: replace with dTdq_new
  MatrixXd dTdqdot;
  Matrix4d Tdot; // floatingbase TODO: replace with Isometry3d?
  MatrixXd ddTdqdq;

  double mass;
  Vector3d com; 
  Matrix<double, TWIST_SIZE, TWIST_SIZE> I;

  DrakeJoint::MotionSubspaceType S;
  MatrixXd dSdqi;
  DrakeJoint::MotionSubspaceType J;
  MatrixXd dJdq;

  MatrixXd qdot_to_v;
  MatrixXd dqdot_to_v_dqi;
  MatrixXd dqdot_to_v_dq;
  MatrixXd v_to_qdot;
  MatrixXd dv_to_qdot_dqi;
  MatrixXd dv_to_qdot_dq;

  Vector6d twist;
  Gradient<Vector6d, Eigen::Dynamic>::type dtwistdq;

  Vector6d SdotV;
  Gradient<Vector6d, Eigen::Dynamic>::type dSdotVdqi;
  Gradient<Vector6d, Eigen::Dynamic>::type dSdotVdvi;

  Vector6d JdotV;
  Gradient<Vector6d, Eigen::Dynamic>::type dJdotVdq;
  Gradient<Vector6d, Eigen::Dynamic>::type dJdotVdv;

  Isometry3d T_new;
  Gradient<Isometry3d::MatrixType, Eigen::Dynamic>::type dTdq_new;

  friend std::ostream& operator<<( std::ostream &out, const RigidBody &b);

  class DLLEXPORT_RBM CollisionElement : public DrakeCollision::Element
  {
    public:
      CollisionElement(const CollisionElement& other);
      CollisionElement(const Matrix4d& T_element_to_link, std::shared_ptr<RigidBody> body);
      CollisionElement(const DrakeShapes::Geometry& geometry,
                       const Matrix4d& T_element_to_link, std::shared_ptr<RigidBody> body);
      virtual ~CollisionElement(){};

      virtual CollisionElement* clone() const;

      const std::shared_ptr<RigidBody>& getBody() const;

      virtual bool collidesWith( const DrakeCollision::Element* other) const;
    protected:
      std::shared_ptr<RigidBody> body;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif
