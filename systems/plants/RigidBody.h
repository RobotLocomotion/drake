#ifndef _RIGIDBODY
#define _RIGIDBODY

#include <set>
#include <vector>

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
  RigidBody() {
    dofnum=0;
    mass = 0.0;
    floating = 0;
    com << Vector3d::Zero(), 1;
    Ttree = Matrix4d::Identity();
    T_body_to_joint = Matrix4d::Identity();
  }
  
  ~RigidBody() {
  	// NOTE: (I'm playing with fire here, but...)
  	// The default operator= is being used in myrealloc to resize RigidBodyManipulator.
  	// This should be suffiicent *except* for the collision object pointers, but since they are being
  	// created and deleted at a level above, I am currently getting away with it.  (yuck!)
  }

  void setN(int n) {    
    T = Matrix4d::Identity();
    dTdq = MatrixXd::Zero(3*n,4);
    dTdqdot = MatrixXd::Zero(3*n,4);
    Tdot = Matrix4d::Zero();
    ddTdqdq = MatrixXd::Zero(3*n*n,4);
  }

  void computeAncestorDOFs(RigidBodyManipulator* model)
  {
    if (dofnum>=0) {
      int i,j;
      if (parent>=0) {
        ancestor_dofs = model->bodies[parent].ancestor_dofs;
        ddTdqdq_nonzero_rows = model->bodies[parent].ddTdqdq_nonzero_rows;
      }
      
      if (floating==1) {
      	for (j=0; j<6; j++) {
      		ancestor_dofs.insert(dofnum+j);
      		for (i=0; i<3*model->NB; i++) {
      			ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum + j);
      			ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i + j);
      		}
      	}
      } else if (floating==2) {
      	for (j=0; j<7; j++) {
      		ancestor_dofs.insert(dofnum+j);
      		for (i=0; i<3*model->NB; i++) {
      			ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum + j);
      			ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i + j);
      		}
      	}
      }
      else {
      	ancestor_dofs.insert(dofnum);
      	for (i=0; i<3*model->NB; i++) {
      		ddTdqdq_nonzero_rows.insert(i*model->NB + dofnum);
      		ddTdqdq_nonzero_rows.insert(3*model->NB*dofnum + i);
      	}
      }


      // compute matrix blocks
      IndexRange ind;  ind.start=-1; ind.length=0;
      for (i=0; i<3*model->NB*model->NB; i++) {
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
  
public:
  std::string linkname;
  std::string jointname;
// note: it's very ugly, but parent,dofnum,and pitch also exist currently (independently) at the rigidbodymanipulator level to represent the featherstone structure.  this version is for the kinematics.
  int parent;
  int dofnum;
  int floating;
  int pitch;
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

#ifdef BULLET_COLLISION
  class CollisionObject {
  public:
	  btCollisionObject* bt_obj;
	  btCollisionShape* bt_shape;
	  Matrix4d T;
  };

  std::vector< CollisionObject > collision_objects;  // note: should be an unordered_set (but linker didn't find on my first try)
#endif
};
#endif
