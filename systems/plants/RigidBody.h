#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <Eigen/StdVector>

class IndexRange {
 public:
  int start;
  int length;
  
  bool operator<(const IndexRange& other) const {
    return start<other.start;
  }
};

class RigidBodyManipulator;

class RigidBody {
public:
  RigidBody();

  void setN(int n);
  void computeAncestorDOFs(RigidBodyManipulator* model);

public:
  std::string linkname;
  std::string jointname;
  int robotnum; // uses 0-index. starts from 0
  static const std::set<int> defaultRobotNumSet;
// note: it's very ugly, but parent,dofnum,and pitch also exist currently (independently) at the rigidbodymanipulator level to represent the featherstone structure.  this version is for the kinematics.
  int parent;
  int dofnum;
  int floating;
  int pitch;
  Eigen::MatrixXd contact_pts;
  Eigen::Matrix4d Ttree;
  Eigen::Matrix4d T_body_to_joint;

  std::set<int> ancestor_dofs;
  std::set<int> ddTdqdq_nonzero_rows;
  std::set<IndexRange> ddTdqdq_nonzero_rows_grouped;

  Eigen::Matrix4d T;
  Eigen::MatrixXd dTdq;
  Eigen::MatrixXd dTdqdot;
  Eigen::Matrix4d Tdot;
  Eigen::MatrixXd ddTdqdq;

  double mass;
  Eigen::Vector4d com;  // this actually stores [com;1] (because that's what's needed in the kinematics functions)

  friend std::ostream& operator<<( std::ostream &out, const RigidBody &b);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif
