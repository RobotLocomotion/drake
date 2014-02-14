#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>
#include <set>
#include <vector>

#include "collision/Model.h"

#include "RigidBody.h"
#include "RigidBodyFrame.h"

#define INF -2147483648
using namespace Eigen;

extern std::set<int> emptyIntSet;  // was const std:set<int> emptyIntSet, but valgrind said I was leaking memory

class RigidBodyManipulator 
{
public:
  RigidBodyManipulator(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);
  ~RigidBodyManipulator(void);
  
  void resize(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);

  void compile(void);  // call me after the model is loaded
  void doKinematics(double* q, bool b_compute_second_derivatives=false, double* qd=NULL);

  template <typename Derived>
  void getCMM(double* const q, double* const qd, MatrixBase<Derived> &A, MatrixBase<Derived> &Adot);

  template <typename Derived>
  void getCOM(MatrixBase<Derived> &com,const std::set<int> &robotnum = RigidBody::defaultRobotNumSet);

  template <typename Derived>
  void getCOMJac(MatrixBase<Derived> &J,const std::set<int> &robotnum = RigidBody::defaultRobotNumSet);

  template <typename Derived>
  void getCOMJacDot(MatrixBase<Derived> &Jdot,const std::set<int> &robotnum = RigidBody::defaultRobotNumSet);

  template <typename Derived>
  void getCOMdJac(MatrixBase<Derived> &dJ, const std::set<int> &robotnum = RigidBody::defaultRobotNumSet);

  int getNumContacts(const std::set<int> &body_idx = emptyIntSet);

  template <typename Derived>
  void getContactPositions(MatrixBase<Derived> &pos, const std::set<int> &body_idx = emptyIntSet);
  
  template <typename Derived>
  void getContactPositionsJac(MatrixBase<Derived> &J, const std::set<int> &body_idx = emptyIntSet);
  
  template <typename Derived>
  void getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const std::set<int> &body_idx = emptyIntSet);

  template <typename DerivedA, typename DerivedB>
  void forwardKin(const int body_or_frame_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x);

  template <typename DerivedA, typename DerivedB>
  void forwardJacDot(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &Jdot);

  template <typename DerivedA, typename DerivedB>
  void forwardJac(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &J);

  template <typename DerivedA, typename DerivedB>
  void forwarddJac(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &dJ);

  template <typename DerivedA, typename DerivedB>
  void bodyKin(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x);


  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE>
  void HandC(double* const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH=NULL, MatrixBase<DerivedE> *dC=NULL);

  void addCollisionElement(const int body_ind, Matrix4d T_elem_to_lnk, DrakeCollision::Shape shape, std::vector<double> params);

  void updateCollisionElements(const int body_ind);

  bool setCollisionFilter(const int body_ind, const uint16_t group, 
                          const uint16_t mask);

  bool getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals);

  bool getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);

  bool getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal);

  bool getPairwiseClosestPoint(const int body_indA, const int body_indB, Vector3d &ptA, Vector3d &ptB, Vector3d &normal, double &distance);

  bool closestPointsAllBodies(std::vector<int>& bodyA_idx, 
                                   std::vector<int>& bodyB_idx, 
                                   MatrixXd& ptsA, 
                                   MatrixXd& ptsB,
                                   MatrixXd& normal, 
                                   VectorXd& distance,
                                   MatrixXd& JA, 
                                   MatrixXd& JB,
                                   MatrixXd& Jd);

  bool closestDistanceAllBodies(VectorXd& distance, MatrixXd& Jd);
  
  int findLinkInd(std::string linkname, int robot = -1);
  //@param robot   the index of the robot. robot = -1 means to look at all the robots
public:
  std::vector<std::string> robot_name;

  int num_dof;
  double* joint_limit_min;
  double* joint_limit_max;

  // Rigid body objects
  int num_bodies;  // rigid body objects
  RigidBody* bodies;

  // Rigid body frames
  int num_frames;
  RigidBodyFrame* frames;

  // featherstone data structure
  int NB;  // featherstone bodies
  int *pitch;
  int *parent;
  int *dofnum;
  double* damping;
  double* coulomb_friction;
  double* coulomb_window;
  double* static_friction;
  MatrixXd* Xtree;
  MatrixXd* I;
  VectorXd a_grav;

  double *cached_q, *cached_qd;  // these should be private


private:
  int parseBodyOrFrameID(const int body_or_frame_id, Matrix4d& Tframe);

  // variables for featherstone dynamics
  VectorXd* S;
  MatrixXd* Xup;
  VectorXd* v;
  VectorXd* avp;
  VectorXd* fvp;
  MatrixXd* IC;

  //Variables for gradient calculations
  MatrixXd dTdTmult;
  MatrixXd* dXupdq;
  MatrixXd** dIC;

  MatrixXd* dvdq;
  MatrixXd* dvdqd;
  MatrixXd* davpdq;
  MatrixXd* davpdqd;
  MatrixXd* dfvpdq;
  MatrixXd* dfvpdqd;
  MatrixXd dvJdqd_mat;
  MatrixXd dcross;

  // preallocate for COM functions
  Vector3d bc;
  MatrixXd bJ;
  MatrixXd bdJ;

  // preallocate for CMM function
  MatrixXd P; // spatial incidence matrix 
  MatrixXd dP; // dP_dq * qd 
  MatrixXd Pinv;
  MatrixXd Phi; // joint axis matrix
  MatrixXd Is; // system inertia matrix
  MatrixXd Xg; // spatial centroidal projection matrix
  MatrixXd *Xworld; // spatial transforms from world to each body
  MatrixXd dXg;  // dXg_dq * qd
  MatrixXd *dXworld; // dXworld_dq * qd
  MatrixXd *dXup; // dXup_dq * qd
  MatrixXd Xcom; // spatial transform from centroid to world
  MatrixXd Jcom; 
  MatrixXd dXcom;
  MatrixXd Xi;
  MatrixXd dXidq;
  VectorXd s;
  MatrixXd Js;
  MatrixXd Jdot;

  int num_contact_pts;
  bool initialized;
  bool kinematicsInit;
  int secondDerivativesCached;

  std::shared_ptr< DrakeCollision::Model > collision_model;
};

template<typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
};
#endif
