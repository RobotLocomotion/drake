#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <Eigen/Dense>
#include <iostream>
#include <set>
#include <Eigen/StdVector>
#include <memory>

class DLLEXPORT IndexRange {
 public:
  int start;
  int length;
  
  bool operator<(const IndexRange& other) const {
    return start<other.start;
  }
};

class RigidBodyManipulator;

using namespace Eigen;

class DLLEXPORT RigidBody {
#if !defined(_WIN32) && !defined(_WIN64)
private:
  std::unique_ptr<DrakeJoint> joint;
#endif

public:
  RigidBody();

  void setN(int n);
  void computeAncestorDOFs(RigidBodyManipulator* model);

#if !defined(_WIN32) && !defined(_WIN64)
  void setJoint(std::unique_ptr<DrakeJoint> joint);
  const DrakeJoint& getJoint() const;
#endif

public:
  std::string linkname;
  std::string jointname; // FLOATINGBASE TODO: remove
  int robotnum; // uses 0-index. starts from 0
  static const std::set<int> defaultRobotNumSet;
// note: it's very ugly, but parent,dofnum,and pitch also exist currently (independently) at the rigidbodymanipulator level to represent the featherstone structure.  this version is for the kinematics.
  int parent;
  int dofnum;
  int floating; // FLOATINGBASE TODO: remove
  int pitch; // FLOATINGBASE TODO: remove
  MatrixXd contact_pts;
  Matrix4d Ttree;
  Matrix4d T_body_to_joint;

  std::set<int> ancestor_dofs;
  std::set<int> ddTdqdq_nonzero_rows;
  std::set<IndexRange> ddTdqdq_nonzero_rows_grouped;

  Matrix4d T;
  MatrixXd dTdq;
  MatrixXd dTdqdot;
  Matrix4d Tdot;
  MatrixXd ddTdqdq;

  double mass;
  Vector4d com;  // this actually stores [com;1] (because that's what's needed in the kinematics functions)

  friend std::ostream& operator<<( std::ostream &out, const RigidBody &b);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif
