#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>
#include <Eigen/LU>
#include <set>
#include <map>
#include <Eigen/StdVector> //#include <vector>

#include "collision/DrakeCollision.h"
#include "KinematicPath.h"
#include "GradientVar.h"

#undef DLLEXPORT_RBM
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeRBM_EXPORTS)
    #define DLLEXPORT_RBM __declspec( dllexport )
  #else
    #define DLLEXPORT_RBM __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT_RBM
#endif


#include "RigidBody.h"
#include "RigidBodyFrame.h"

#define INF -2147483648    // this number is only used for checking the pitch to see if it's a revolute joint or a helical joint, and is set to match the value handed to us for inf from matlab.

using namespace Eigen;

//extern std::set<int> emptyIntSet;  // was const std:set<int> emptyIntSet, but valgrind said I was leaking memory

class DLLEXPORT_RBM RigidBodyActuator
{
public:
  RigidBodyActuator(std::string _name, std::shared_ptr<RigidBody> _body, double _reduction = 1.0) :
    name(_name), body(_body), reduction(_reduction) {};

  std::string name;
  std::shared_ptr<RigidBody> body;
  double reduction;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DLLEXPORT_RBM RigidBodyLoop
{
public:
  RigidBodyLoop(std::shared_ptr<RigidBody> _bodyA, Vector3d _ptA, std::shared_ptr<RigidBody> _bodyB, Vector3d _ptB) :
    bodyA(_bodyA), bodyB(_bodyB), ptA(_ptA), ptB(_ptB) {};

  std::shared_ptr<RigidBody> bodyA, bodyB;
  Vector3d ptA, ptB;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DLLEXPORT_RBM RigidBodyManipulator
{
public:
  RigidBodyManipulator(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);
  RigidBodyManipulator(const std::string &urdf_filename);
  virtual ~RigidBodyManipulator(void);

  bool addRobotFromURDFString(const std::string &xml_string, const std::string &root_dir = ".");
  bool addRobotFromURDF(const std::string &urdf_filename);

  void resize(int num_dof, int num_featherstone_bodies=-1, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);

  void compile(void);  // call me after the model is loaded
  
  template <typename Derived>
  void doKinematics(MatrixBase<Derived> & q, bool b_compute_second_derivatives = false);

  template <typename DerivedA, typename DerivedB>
  void doKinematics(MatrixBase<DerivedA> & q, bool b_compute_second_derivatives, MatrixBase<DerivedB> & v);

  template <typename DerivedQ, typename DerivedV>
  void doKinematicsNew(const MatrixBase<DerivedQ>& q, const MatrixBase<DerivedV>& v, bool compute_gradients = false, bool compute_JdotV = false);

  void updateCompositeRigidBodyInertias(int gradient_order);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> centroidalMomentumMatrix(int gradient_order);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> centroidalMomentumMatrixDotTimesV(int gradient_order);

  template <typename DerivedA, typename DerivedB>
  void getCMM(MatrixBase<DerivedA> const & q, MatrixBase<DerivedA> const & qd, MatrixBase<DerivedB> &A, MatrixBase<DerivedB> &Adot);

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMass(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMassJacobianDotTimesV(int gradient_order);

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

  void findAncestorBodies(std::vector<int>& ancestor_bodies, int body);

  void findKinematicPath(KinematicPath& path, int start_body_or_frame_idx, int end_body_or_frame_idx);

  template <typename DerivedA, typename DerivedB>
  void forwardKin(const int body_or_frame_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &x);

  template <typename DerivedA, typename DerivedB>
  void forwardJacDot(const int body_ind, const MatrixBase<DerivedA>& pts, const int, MatrixBase<DerivedB> &Jdot);

  template <typename DerivedA, typename DerivedB>
  void forwardJac(const int body_ind, const MatrixBase<DerivedA>& pts, const int rotation_type, MatrixBase<DerivedB> &J);

  template <typename DerivedA, typename DerivedB>
  void forwarddJac(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &dJ);

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> massMatrix(int gradient_order = 0);

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, 1> inverseDynamics(std::map<int, std::unique_ptr< GradientVar<Scalar, TWIST_SIZE, 1> > >& f_ext, GradientVar<Scalar, Eigen::Dynamic, 1>* vd = nullptr, int gradient_order = 0);

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> forwardKinNew(const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order);

  template <typename Scalar, int XRows, int XCols>
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> forwardJacV(const GradientVar<Scalar, XRows, XCols>& x, int body_or_frame_ind, int base_or_frame_ind, int rotation_type, bool compute_analytic_jacobian, int gradient_order);

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> forwardKinPositionGradient(int npoints, int current_body_or_frame_ind, int new_body_or_frame_ind, int gradient_order);

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, 1> forwardJacDotTimesV(const MatrixBase<DerivedPoints>& points, int body_or_frame_ind, int base_or_frame_ind, int rotation_type, int gradient_order);

  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD>
  void bodyKin(const int body_ind, const MatrixBase<DerivedA>& pts, MatrixBase<DerivedB> &x, MatrixBase<DerivedC> *J=NULL, MatrixBase<DerivedD> *P=NULL);

  template<typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobian(int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot = false, std::vector<int>* v_indices = nullptr);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> geometricJacobianDotTimesV(int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> relativeTwist(int base_or_frame_ind, int body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> transformSpatialAcceleration(const GradientVar<Scalar, TWIST_SIZE, 1>& spatial_acceleration, int base_or_frame_ind, int body_or_frame_ind, int old_body_or_frame_ind, int new_body_or_frame_ind);

  template<typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> relativeTransform(int base_or_frame_ind, int body_or_frame_ind, int gradient_order);

  template <typename DerivedA, typename DerivedB, typename DerivedC, typename DerivedD, typename DerivedE, typename DerivedF, typename DerivedG>
  void HandC(MatrixBase<DerivedG> const & q, MatrixBase<DerivedG> const & qd, MatrixBase<DerivedA> * const f_ext, MatrixBase<DerivedB> &H, MatrixBase<DerivedC> &C, MatrixBase<DerivedD> *dH=NULL, MatrixBase<DerivedE> *dC=NULL, MatrixBase<DerivedF> * const df_ext=NULL);

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

  int findLinkId(std::string linkname, int robot = -1);
  //@param robot   the index of the robot. robot = -1 means to look at all the robots

  std::string getBodyOrFrameName(int body_or_frame_id);
  //@param body_or_frame_id   the index of the body or the id of the frame.

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, 1> positionConstraints(int gradient_order);

public:
  std::vector<std::string> robot_name;

  int num_positions; // treated as nq now; TODO: rename to nq
  int num_velocities;
  VectorXd joint_limit_min;
  VectorXd joint_limit_max;

  // Rigid body objects
  int num_bodies;  // rigid body objects
  std::vector<std::shared_ptr<RigidBody> > bodies;

  // Rigid body frames
  int num_frames;
  std::vector<RigidBodyFrame,Eigen::aligned_allocator<RigidBodyFrame> > frames;

  // Rigid body actuators
  std::vector<RigidBodyActuator,Eigen::aligned_allocator<RigidBodyActuator> > actuators;

  // Rigid body loops
  std::vector<RigidBodyLoop,Eigen::aligned_allocator<RigidBodyLoop> > loops;

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
  Matrix<double,TWIST_SIZE,1> a_grav;
  MatrixXd B;  // the B matrix maps inputs into joint-space forces

  VectorXd cached_q, cached_v;  // these should be private

  bool use_new_kinsol;

private:
  void doKinematics(double* q, bool b_compute_second_derivatives=false, double* qd=NULL);
  int parseBodyOrFrameID(const int body_or_frame_id, Matrix4d* Tframe = nullptr);

  // variables for featherstone dynamics
  std::vector<VectorXd> S;
  std::vector<MatrixXd> Xup;
  std::vector<VectorXd> v;
  std::vector<VectorXd> avp;
  std::vector<VectorXd> fvp;
  std::vector<MatrixXd> IC;
  std::vector<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::aligned_allocator< Matrix<double, TWIST_SIZE, TWIST_SIZE> > > I_world;
  std::vector<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::aligned_allocator< Matrix<double, TWIST_SIZE, TWIST_SIZE> > > Ic_new;


  //Variables for gradient calculations
  MatrixXd dTdTmult;
  std::vector<MatrixXd> dXupdq;
  std::vector<std::vector<MatrixXd> > dIC;
    std::vector<Gradient<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::Dynamic>::type> dI_world;
  std::vector<Gradient<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::Dynamic>::type> dIc_new;

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
  std::vector<MatrixXd> Ic; // composite rigid body inertias
  std::vector<MatrixXd> dIc; // derivative of composite rigid body inertias
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
  bool gradients_cached;
  bool velocity_kinematics_cached;
  bool jdotV_cached;
  int cached_inertia_gradients_order;

  // collision_model and collision_model_no_margins both maintain
  // a collection of the collision geometry in the RBM for use in
  // collision detection of different kinds. collision_model has
  // small margins applied to all collision geometry when that
  // geometry is added, to improve the numerical stability of
  // contact gradients taken using the model. collision_model_no_margins
  // does not apply these margins, such that it can be used for
  // precise raycasting, e.g. for simulating a laser scanner
  // These models are switched between with the use_margins flag
  // to collision-relevant methods of the RBM.
  std::shared_ptr< DrakeCollision::Model > collision_model;
  std::shared_ptr< DrakeCollision::Model > collision_model_no_margins;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// The following was required for building w/ DLLEXPORT_RBM on windows (due to the unique_ptrs).  See
// http://stackoverflow.com/questions/8716824/cannot-access-private-member-error-only-when-class-has-export-linkage
private:
  RigidBodyManipulator(const RigidBodyManipulator&) {}
  RigidBodyManipulator& operator=(const RigidBodyManipulator&) { return *this; }
};


#endif
