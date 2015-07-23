#ifndef __RigidBodyManipulator_H__
#define __RigidBodyManipulator_H__

#include <Eigen/Dense>
#include <Eigen/LU>
#include <set>
#include <map>
#include <Eigen/StdVector>

#include "collision/DrakeCollision.h"
#include "shapes/DrakeShapes.h"
#include "KinematicPath.h"
#include "ForceTorqueMeasurement.h"
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

#define BASIS_VECTOR_HALF_COUNT 2  //number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8
#define MIN_RADIUS 1e-7

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3xd;

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
	friend std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj);
};

class DLLEXPORT_RBM RigidBodyManipulator
{
public:
  RigidBodyManipulator(int num_dof, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);
  RigidBodyManipulator(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  RigidBodyManipulator(void);
  virtual ~RigidBodyManipulator(void);

  bool addRobotFromURDFString(const std::string &xml_string, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDFString(const std::string &xml_string, std::map<std::string,std::string>& package_map, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDF(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDF(const std::string &urdf_filename, std::map<std::string,std::string>& package_map, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);

  void addFrame(const RigidBodyFrame& frame);

  std::map<std::string, int> computePositionNameToIndexMap() const;

  void surfaceTangents(Eigen::Map<Matrix3xd> const & normals, std::vector< Map<Matrix3xd> > & tangents);

  void resize(int num_dof, int num_rigid_body_objects=-1, int num_rigid_body_frames=0);

  void compile(void);  // call me after the model is loaded

  void getRandomConfiguration(Eigen::VectorXd& q, std::default_random_engine& generator) const;

  // akin to the coordinateframe names in matlab
  std::string getPositionName(int position_num) const;
  std::string getVelocityName(int velocity_num) const;
  std::string getStateName(int state_num) const;

  template <typename Derived>
  void doKinematics(MatrixBase<Derived> & q, bool b_compute_second_derivatives = false);

  template <typename DerivedA, typename DerivedB>
  void doKinematics(MatrixBase<DerivedA> & q, bool b_compute_second_derivatives, MatrixBase<DerivedB> & v);

  template <typename DerivedQ, typename DerivedV>
  void doKinematicsNew(const MatrixBase<DerivedQ>& q, const MatrixBase<DerivedV>& v, bool compute_gradients = false, bool compute_JdotV = true);

  bool isBodyPartOfRobot(const RigidBody& body, const std::set<int>& robotnum);

  double getMass(const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMass(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> worldMomentumMatrix(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> worldMomentumMatrixDotTimesV(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> centroidalMomentumMatrix(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false);

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> centroidalMomentumMatrixDotTimesV(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> centerOfMassJacobian(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false);

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMassJacobianDotTimesV(int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet);

  template <typename Derived>
  void getCOM(MatrixBase<Derived> &com,const std::set<int> &robotnum = RigidBody::defaultRobotNumSet);

  template <typename DerivedA, typename DerivedB, typename DerivedC>
  void jointLimitConstraints(MatrixBase<DerivedA> const & q, MatrixBase<DerivedB> &phi, MatrixBase<DerivedC> &J) const;

  size_t getNumJointLimitConstraints() const;

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


  /**
   * Computes CoP in world frame. Normal and point on contact plane should be in world frame too.
   */
  template <typename DerivedNormal, typename DerivedPoint>
  std::pair<Eigen::Vector3d, double> resolveCenterOfPressure( const std::vector< ForceTorqueMeasurement > & force_torque_measurements, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane);

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

  template <typename DerivedV>
  GradientVar<typename DerivedV::Scalar, Dynamic, 1> frictionTorques(Eigen::MatrixBase<DerivedV> const & v, int gradient_order = 0);

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> forwardKinNew(const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order);

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, Eigen::Dynamic> forwardJacV(const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, bool in_terms_of_qdot, int gradient_order);

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

  void computeContactJacobians(VectorXi const & idxA, VectorXi const & idxB, Map<Matrix3xd> const & xA, Map<Matrix3xd> const & xB, const bool compute_second_derivatives, MatrixXd & J, MatrixXd & dJ);

  DrakeCollision::ElementId addCollisionElement(const RigidBody::CollisionElement& element, const std::shared_ptr<RigidBody>& body, std::string group_name);

  void updateCollisionElements(const std::shared_ptr<RigidBody>& body);
  void getTerrainContactPoints(const RigidBody& body, Eigen::Matrix3Xd &terrain_points) const;

  bool collisionRaycast(const Matrix3Xd &origins, const Matrix3Xd &ray_endpoints, VectorXd &distances, bool use_margins=false);

  bool collisionDetect( VectorXd& phi,
                        Matrix3Xd& normal,
                        Matrix3Xd& xA,
                        Matrix3Xd& xB,
                        std::vector<int>& bodyA_idx,
                        std::vector<int>& bodyB_idx,
                        const std::vector<DrakeCollision::ElementId>& ids_to_check,
                        bool use_margins);

  bool collisionDetect( VectorXd& phi, Matrix3Xd& normal,
                        Matrix3Xd& xA, Matrix3Xd& xB,
                        std::vector<int>& bodyA_idx,
                        std::vector<int>& bodyB_idx,
                        const std::vector<int>& bodies_idx,
                        const std::set<std::string>& active_element_groups,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, Matrix3Xd& normal,
                        Matrix3Xd& xA, Matrix3Xd& xB,
                        std::vector<int>& bodyA_idx,
                        std::vector<int>& bodyB_idx,
                        const std::vector<int>& bodies_idx,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, Matrix3Xd& normal,
                        Matrix3Xd& xA, Matrix3Xd& xB,
                        std::vector<int>& bodyA_idx,
                        std::vector<int>& bodyB_idx,
                        const std::set<std::string>& active_element_groups,
                        bool use_margins = true);

  bool collisionDetect( VectorXd& phi, Matrix3Xd& normal,
                        Matrix3Xd& xA, Matrix3Xd& xB,
                        std::vector<int>& bodyA_idx,
                        std::vector<int>& bodyB_idx,
                        bool use_margins = true);


  bool allCollisions(std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                     Matrix3Xd& ptsA, Matrix3Xd& ptsB,
                        bool use_margins = true);

  void potentialCollisions(Eigen::VectorXd& phi,
                           Eigen::Matrix3Xd& normal,
                           Eigen::Matrix3Xd& xA,
                           Eigen::Matrix3Xd& xB,
                           std::vector<int>& bodyA_idx,
                           std::vector<int>& bodyB_idx,
                           bool use_margins = true);
  //bool closestDistanceAllBodies(VectorXd& distance, MatrixXd& Jd);

  virtual std::vector<size_t> collidingPoints(
        const std::vector<Eigen::Vector3d>& points,
        double collision_threshold);

  void warnOnce(const std::string& id, const std::string& msg);

  std::shared_ptr<RigidBody> findLink(std::string linkname, int robot=-1);
  int findLinkId(std::string linkname, int robot = -1);
  std::shared_ptr<RigidBody> findJoint(std::string jointname, int robot=-1);
  int findJointId(std::string linkname, int robot = -1);
  //@param robot   the index of the robot. robot = -1 means to look at all the robots

  std::string getBodyOrFrameName(int body_or_frame_id);
  //@param body_or_frame_id   the index of the body or the id of the frame.

  int parseBodyOrFrameID(const int body_or_frame_id, Matrix4d* Tframe = nullptr);

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, 1> positionConstraintsNew(int gradient_order);

  template <typename DerivedA, typename DerivedB>
  void positionConstraints(Eigen::MatrixBase<DerivedA> & phi, Eigen::MatrixBase<DerivedB> & J);

  size_t getNumPositionConstraints() const;

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> transformVelocityMappingToPositionDotMapping(
      const Eigen::MatrixBase<Derived>& mat);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> compactToFull(
    const Eigen::MatrixBase<Derived>& compact, const std::vector<int>& joint_path, bool in_terms_of_qdot) {
  /*
   * This method is used after calling geometric Jacobian, where compact is the Jacobian on the joints that are on the kinematic path; if we want to reconstruct the full Jacobian on all joints, then we should call this method.
   */
  int ncols = in_terms_of_qdot ? num_positions : num_velocities;
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> full(compact.rows(), ncols);
  full.setZero();
  int compact_col_start = 0;
  for (std::vector<int>::const_iterator it = joint_path.begin(); it != joint_path.end(); ++it) {
    RigidBody& body = *bodies[*it];
    int ncols_joint = in_terms_of_qdot ? body.getJoint().getNumPositions() : body.getJoint().getNumVelocities();
    int col_start = in_terms_of_qdot ? body.position_num_start : body.velocity_num_start;
    full.middleCols(col_start, ncols_joint) = compact.middleCols(compact_col_start, ncols_joint);
    compact_col_start += ncols_joint;
  }
  return full;
};
public:
  std::vector<std::string> robot_name;

  int num_positions;
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

  Matrix<double,TWIST_SIZE,1> a_grav;
  MatrixXd B;  // the B matrix maps inputs into joint-space forces

  /*
   * Temporary solution, as we're switching from old kinematics to new.
   * Had to separate these so that it's possible to know specifically whether the old doKinematics and/or new doKinematics
   * has been cached with a given q and v. There was a place outside of RigidBodyManipulator that used cached_q and should continue
   * to work for both new and old doKinematics, so we kept cached_q and cached_v as well (set by whatever doKinematics method
   * was called last).
   */
  VectorXd cached_q, cached_v;  // these should be private
  VectorXd cached_q_old, cached_v_old;  // these should be private

  void setUseNewKinsol(bool tf) { use_new_kinsol=tf; kinematicsInit=false; }
  bool getUseNewKinsol(void) { return use_new_kinsol; }

private:
  VectorXd cached_q_new, cached_v_new;
  bool use_new_kinsol;

  void doKinematics(double* q, bool b_compute_second_derivatives=false, double* qd=NULL);

  //helper functions for contactConstraints
  void accumulateContactJacobian(const int bodyInd, Matrix3Xd const & bodyPoints, std::vector<size_t> const & cindA, std::vector<size_t> const & cindB, MatrixXd & J);
  void accumulateSecondOrderContactJacobian(const int bodyInd, Matrix3Xd const & bodyPoints, std::vector<size_t> const & cindA, std::vector<size_t> const & cindB, MatrixXd & dJ);

  void updateCompositeRigidBodyInertias(int gradient_order);

  void checkCachedKinematicsSettings(bool kinematics_gradients_required, bool velocity_kinematics_required, bool jdot_times_v_required, const std::string& method_name);

  // variables for featherstone dynamics
  std::vector<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::aligned_allocator< Matrix<double, TWIST_SIZE, TWIST_SIZE> > > I_world;
  std::vector<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::aligned_allocator< Matrix<double, TWIST_SIZE, TWIST_SIZE> > > Ic_new;


  //Variables for gradient calculations
  MatrixXd dTdTmult;
  std::vector<Gradient<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::Dynamic>::type> dI_world;
  std::vector<Gradient<Matrix<double, TWIST_SIZE, TWIST_SIZE>, Eigen::Dynamic>::type> dIc_new;

  // preallocate for COM functions
  Vector3d bc;
  MatrixXd bJ;
  MatrixXd bdJ;

  int num_contact_pts;
  bool initialized;
  bool kinematicsInit;
  int secondDerivativesCached;
  bool position_kinematics_cached;
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
  std::unique_ptr< DrakeCollision::Model > collision_model;
  //std::shared_ptr< DrakeCollision::Model > collision_model_no_margins;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// The following was required for building w/ DLLEXPORT_RBM on windows (due to the unique_ptrs).  See
// http://stackoverflow.com/questions/8716824/cannot-access-private-member-error-only-when-class-has-export-linkage
private:
  RigidBodyManipulator(const RigidBodyManipulator&);
  RigidBodyManipulator& operator=(const RigidBodyManipulator&) { return *this; }

  std::set<std::string> already_printed_warnings;
};


#endif
