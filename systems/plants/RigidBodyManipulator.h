#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>
#include <Eigen/LU>
#include <set>
#include <Eigen/StdVector> //#include <vector>

#include "collision/DrakeCollision.h"

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeRBM_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
  #include "DrakeJoint.h"  // todo: move this out of here
#endif

#include "RigidBody.h"
#include "RigidBodyFrame.h"

#define INF -2147483648
using namespace Eigen;

//extern std::set<int> emptyIntSet;  // was const std:set<int> emptyIntSet, but valgrind said I was leaking memory

class DLLEXPORT RigidBodyManipulator 
{
public:
  RigidBodyManipulator(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);
  virtual ~RigidBodyManipulator(void);

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

  int getNumContacts(const std::set<int> &body_idx);// = emptyIntSet);

  template <typename Derived>
    void getContactPositions(MatrixBase<Derived> &pos, const std::set<int> &body_idx);// = emptyIntSet);
  
  template <typename Derived>
    void getContactPositionsJac(MatrixBase<Derived> &J, const std::set<int> &body_idx);// = emptyIntSet);
  
  template <typename Derived>
    void getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const std::set<int> &body_idx);// = emptyIntSet);

  template <typename DerivedA, typename DerivedB>
  void forwardKin(const int body_or_frame_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x);

  template <typename DerivedA, typename DerivedB>
  void forwardJacDot(const int body_ind, const MatrixBase<DerivedA>& pts, const int, MatrixBase<DerivedB> &Jdot);

  template <typename DerivedA, typename DerivedB>
  void forwardJac(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &J);

  template <typename DerivedA, typename DerivedB>
  void forwarddJac(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &dJ);

  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
  void bodyKin(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x, MatrixBase<DerivedC> *J=NULL, MatrixBase<DerivedD> *P=NULL);


  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE, typename DerivedF>
  void HandC(double* const q, double * const qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH=NULL, MatrixBase<DerivedE> *dC=NULL, MatrixBase<DerivedF> * const df_ext=NULL);

  void addCollisionElement(const int body_ind, const Matrix4d &T_elem_to_lnk, DrakeCollision::Shape shape, std::vector<double> params, std::string group_name = "default");

  void updateCollisionElements(const int body_ind);

  bool setCollisionFilter(const int body_ind, const uint16_t group, 
                          const uint16_t mask);

  bool getPairwiseCollision(const int body_indA, const int body_indB, MatrixXd &ptsA, MatrixXd &ptsB, MatrixXd &normals, bool use_margins=true);

  bool getPairwisePointCollision(const int body_indA, const int body_indB, const int body_collision_indA, Vector3d &ptA, Vector3d &ptB, Vector3d &normal, bool use_margins=true);

  bool getPointCollision(const int body_ind, const int body_collision_ind, Vector3d &ptA, Vector3d &ptB, Vector3d &normal, bool use_margins=true);

  bool getPairwiseClosestPoint(const int body_indA, const int body_indB, Vector3d &ptA, Vector3d &ptB, Vector3d &normal, double &distance, bool use_margins=true);
  
  bool collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances, bool use_margins=false);

  //bool closestPointsAllBodies( MatrixXd& ptsA, MatrixXd& ptsB,
                               //MatrixXd& normal, VectorXd& distance,
                               //std::vector<int>& bodyA_idx, 
                               //std::vector<int>& bodyB_idx);

  bool collisionDetect( VectorXd& phi, MatrixXd& normal, 
                        MatrixXd& xA, MatrixXd& xB, 
                        std::vector<int>& bodyA_idx, 
                        std::vector<int>& bodyB_idx,
                        const std::vector<int>& bodies_idx,
                        const std::set<std::string>& active_element_groups,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, MatrixXd& normal, 
                        MatrixXd& xA, MatrixXd& xB, 
                        std::vector<int>& bodyA_idx, 
                        std::vector<int>& bodyB_idx,
                        const std::vector<int>& bodies_idx,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, MatrixXd& normal, 
                        MatrixXd& xA, MatrixXd& xB, 
                        std::vector<int>& bodyA_idx, 
                        std::vector<int>& bodyB_idx,
                        const std::set<std::string>& active_element_groups,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, MatrixXd& normal, 
                        MatrixXd& xA, MatrixXd& xB, 
                        std::vector<int>& bodyA_idx, 
                        std::vector<int>& bodyB_idx,
                        bool use_margins = true);


  bool allCollisions(std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx, 
                     MatrixXd& ptsA, MatrixXd& ptsB,
                        bool use_margins = true);

  //bool closestDistanceAllBodies(VectorXd& distance, MatrixXd& Jd);
  
  int findLinkInd(std::string linkname, int robot = -1);
  //@param robot   the index of the robot. robot = -1 means to look at all the robots
  
  std::string getBodyOrFrameName(int body_or_frame_id);
  //@param body_or_frame_id   the index of the body or the id of the frame. 
public:
  std::vector<std::string> robot_name;

  int num_dof;
  VectorXd joint_limit_min;
  VectorXd joint_limit_max;

  // Rigid body objects
  int num_bodies;  // rigid body objects
  std::vector<std::unique_ptr<RigidBody>> bodies;

  // Rigid body frames
  int num_frames;
  std::vector<RigidBodyFrame,Eigen::aligned_allocator<RigidBodyFrame> > frames;

  // featherstone data structure
  int NB;  // featherstone bodies
  VectorXi pitch;
  VectorXi parent;
  VectorXi dofnum;
  VectorXd damping;
  VectorXd coulomb_friction;
  VectorXd coulomb_window;
  VectorXd static_friction;
  std::vector<MatrixXd> Xtree;
  std::vector<MatrixXd> I;
  VectorXd a_grav;

  VectorXd cached_q, cached_qd;  // these should be private


private:
  int parseBodyOrFrameID(const int body_or_frame_id, Matrix4d& Tframe);

  // variables for featherstone dynamics
  std::vector<VectorXd> S;
  std::vector<MatrixXd> Xup;
  std::vector<VectorXd> v;
  std::vector<VectorXd> avp;
  std::vector<VectorXd> fvp;
  std::vector<MatrixXd> IC;

  //Variables for gradient calculations
  MatrixXd dTdTmult;
  std::vector<MatrixXd> dXupdq;
  std::vector<std::vector<MatrixXd>> dIC;

  std::vector<MatrixXd> dvdq;
  std::vector<MatrixXd> dvdqd;
  std::vector<MatrixXd> davpdq;
  std::vector<MatrixXd> davpdqd;
  std::vector<MatrixXd> dfvpdq;
  std::vector<MatrixXd> dfvpdqd;
  MatrixXd dvJdqd_mat;
  MatrixXd dcross;

  // preallocate for COM functions
  Vector3d bc;
  MatrixXd bJ;
  MatrixXd bdJ;

  // preallocate for CMM function
  MatrixXd Xg; // spatial centroidal projection matrix
  MatrixXd dXg;  // dXg_dq * qd  
  std::vector<MatrixXd> Ic; // body spatial inertias
  std::vector<MatrixXd> dIc; // derivative of body spatial inertias
  std::vector<VectorXd> phi; // joint axis vectors
  std::vector<MatrixXd> Xworld; // spatial transforms from world to each body
  std::vector<MatrixXd> dXworld; // dXworld_dq * qd
  std::vector<MatrixXd> dXup; // dXup_dq * qd 
  MatrixXd Xcom; // spatial transform from centroid to world
  MatrixXd Jcom; 
  MatrixXd dXcom;
  MatrixXd Xi;
  MatrixXd dXidq;

  int num_contact_pts;
  bool initialized;
  bool kinematicsInit;
  int secondDerivativesCached;

  std::shared_ptr< DrakeCollision::Model > collision_model;
  std::shared_ptr< DrakeCollision::Model > collision_model_no_margins;  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// The following was required for building w/ DLLEXPORT on windows (due to the unique_ptrs).  See
// http://stackoverflow.com/questions/8716824/cannot-access-private-member-error-only-when-class-has-export-linkage
private:
  RigidBodyManipulator(const RigidBodyManipulator&) {}
  RigidBodyManipulator& operator=(const RigidBodyManipulator&) { return *this; }
};

/*
template<typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
};
*/
#endif
