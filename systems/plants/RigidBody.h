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

public:
  RigidBody();

  void setN(int n);
  void computeAncestorDOFs(RigidBodyManipulator* model);

  void setJoint(std::unique_ptr<DrakeJoint> joint);
  const DrakeJoint& getJoint() const;

  bool hasParent() const;

public:
  std::string linkname;
  std::string jointname; // FLOATINGBASE TODO: remove
  int robotnum; // uses 0-index. starts from 0
  static const std::set<int> defaultRobotNumSet;
// note: it's very ugly, but parent,dofnum,and pitch also exist currently (independently) at the rigidbodymanipulator level to represent the featherstone structure.  this version is for the kinematics.
  int parent;
  int dofnum; // interpreted as start of position_num from Matlab
  int velocity_num_start;
  int floating; // FLOATINGBASE TODO: remove
  int pitch; // FLOATINGBASE TODO: remove
  MatrixXd contact_pts;
  Matrix4d Ttree;
  Matrix4d T_body_to_joint;

  std::set<int> ancestor_dofs;
  std::set<int> ddTdqdq_nonzero_rows;
  std::set<IndexRange> ddTdqdq_nonzero_rows_grouped;

  Matrix4d T;
  MatrixXd dTdq; // floatingbase TODO: replace with dTdq_new
  MatrixXd dTdqdot;
  Matrix4d Tdot;
  MatrixXd ddTdqdq;

  double mass;
  Vector4d com;  // this actually stores [com;1] (because that's what's needed in the kinematics functions)
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

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif
