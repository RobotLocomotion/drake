#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <set>

class IndexRange {
 public:
  int start;
  int length;
  
  bool operator<(const IndexRange& other) const {
    return start<other.start;
  }
};

class RigidBody {
public:
  std::string linkname;
  std::string jointname;
  MatrixXd dTdq;
  MatrixXd ddTdqdq;
  std::set<int> ancestor_dofs;
  std::set<int> ddTdqdq_nonzero_rows;
  std::set<IndexRange> ddTdqdq_nonzero_rows_grouped;
  Matrix4d T;
  Matrix4d T_body_to_joint;

  double mass;
  Vector4d com;  // this actually stores [com;1] (because that's what's needed in the kinematics functions)
  
  //Need to initialize these
  Matrix4d Ttree;
  int dofnum;

  RigidBody() {
    mass = 0.0;
    com << Vector3d::Zero(), 1;
  }
  
  ~RigidBody() {
  }

  void setN(int n) {    
    T = Matrix4d::Identity();
    dTdq = MatrixXd::Zero(4*n,4);
    ddTdqdq = MatrixXd::Zero(4*n*n,4);
    Ttree = Matrix4d::Identity();
    T_body_to_joint = Matrix4d::Identity();
  }

  void computeAncestorDOFs(RigidBodyManipulator* model)
  {
    if (dofnum>=0) {
      int i;
      ancestor_dofs = model->bodies[model->parent[dofnum]].ancestor_dofs;
      ancestor_dofs.insert(dofnum);
      ddTdqdq_nonzero_rows = model->bodies[model->parent[dofnum]].ddTdqdq_nonzero_rows;
      for (i=0; i<4*model->NB; i++) {
	ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum);
	ddTdqdq_nonzero_rows.insert(4*model->NB*dofnum + i);
      }
      IndexRange ind;  ind.start=-1; ind.length=0;
      for (i=0; i<4*model->NB*4*model->NB; i++) {
	if (ddTdqdq_nonzero_rows.find(i)!=ddTdqdq_nonzero_rows.end()) {
	  if (ind.start<0) ind.start=i;
	} else {
	  if (ind.start>=0) {
	    ind.length = i-ind.start;
	    ddTdqdq_nonzero_rows_grouped.insert(ind);
	    ind.start = -1;
	  }
	}
      }
      if (ind.start>=0) {
	ind.length = i-ind.start;
	ddTdqdq_nonzero_rows_grouped.insert(ind);
      }
    }
  }
  
};
#endif
