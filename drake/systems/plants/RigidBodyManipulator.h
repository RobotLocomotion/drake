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
#include <stdexcept>


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
#include "KinematicsCache.h"

#define BASIS_VECTOR_HALF_COUNT 2  //number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8
#define MIN_RADIUS 1e-7

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;

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
  RigidBodyLoop(const std::shared_ptr<RigidBodyFrame>& _frameA, const std::shared_ptr<RigidBodyFrame>& _frameB, const Eigen::Vector3d& _axis) :
    frameA(_frameA), frameB(_frameB), axis(_axis) {};

  std::shared_ptr<RigidBodyFrame> frameA, frameB;
  Eigen::Vector3d axis;

  friend std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class DLLEXPORT_RBM RigidBodyManipulator
{
public:
  RigidBodyManipulator(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  RigidBodyManipulator(void);
  virtual ~RigidBodyManipulator(void);

  bool addRobotFromURDFString(const std::string &xml_string, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDFString(const std::string &xml_string, std::map<std::string,std::string>& package_map, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDF(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
  bool addRobotFromURDF(const std::string &urdf_filename, std::map<std::string,std::string>& package_map, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);

  void addFrame(const std::shared_ptr<RigidBodyFrame>& frame);

  std::map<std::string, int> computePositionNameToIndexMap() const;

  void surfaceTangents(Eigen::Map<Eigen::Matrix3Xd> const & normals, std::vector< Eigen::Map<Eigen::Matrix3Xd> > & tangents) const;

  void compile(void);  // call me after the model is loaded

  void getRandomConfiguration(Eigen::VectorXd& q, std::default_random_engine& generator) const;

  // akin to the coordinateframe names in matlab
  std::string getPositionName(int position_num) const;
  std::string getVelocityName(int velocity_num) const;
  std::string getStateName(int state_num) const;

  template <typename DerivedQ>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(const Eigen::MatrixBase<DerivedQ>& q, int gradient_order) {
    KinematicsCache<typename DerivedQ::Scalar> ret(bodies, gradient_order);
    ret.initialize(q);
    doKinematics(ret);
    return ret;
  }

  template <typename DerivedQ, typename DerivedV>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedV>& v, int gradient_order = 0, bool compute_JdotV = true) {
    KinematicsCache<typename DerivedQ::Scalar> ret(bodies, gradient_order);
    ret.initialize(q, v);
    doKinematics(ret, compute_JdotV);
    return ret;
  }

  template <typename Scalar>
  void doKinematics(KinematicsCache<Scalar>& cache, bool compute_JdotV = false) const {
    using namespace std;
    using namespace Eigen;

    const auto& q = cache.getQ();
    if (!initialized)
      throw runtime_error("RigidBodyManipulator::doKinematics: call compile first.");

    int nq = num_positions;
    int gradient_order = cache.getGradientOrder();
    bool compute_gradients = gradient_order > 0;

    compute_JdotV = compute_JdotV && cache.hasV(); // no sense in computing Jdot times v if v is not passed in

    cache.setPositionKinematicsCached(); // doing this here because there is a geometricJacobian call within doKinematics below which checks for this

    for (int i = 0; i < bodies.size(); i++) {
      RigidBody& body = *bodies[i];
      KinematicsCacheElement<Scalar>& element = cache.getElement(body);

      if (body.hasParent()) {
        const KinematicsCacheElement<Scalar>& parent_element = cache.getElement(*body.parent);
        const DrakeJoint& joint = body.getJoint();
        auto q_body = q.middleRows(body.position_num_start, joint.getNumPositions());

        // transform
        auto T_body_to_parent = joint.getTransformToParentBody().cast<Scalar>() * joint.jointTransform(q_body);
        element.transform_to_world = parent_element.transform_to_world * T_body_to_parent;

        // motion subspace in body frame
        Matrix<Scalar, Dynamic, Dynamic>* dSdq = compute_gradients ? &(element.motion_subspace_in_body.gradient().value()) : nullptr;
        joint.motionSubspace(q_body, element.motion_subspace_in_body.value(), dSdq);

        // motion subspace in world frame
        element.motion_subspace_in_world.value() = transformSpatialMotion(element.transform_to_world, element.motion_subspace_in_body.value());

        // qdot to v, v to qdot
        if (compute_gradients) {
          // TODO: make DrakeJoint::v2qdot and qdot2v accept Refs instead, pass in blocks of element.v_to_qdot.gradient().value() and element.qdot_to_v.gradient().value()
          Matrix<Scalar, Dynamic, Dynamic> dqdot_to_vdqi(element.qdot_to_v.value().size(), joint.getNumPositions());
          joint.qdot2v(q_body, element.qdot_to_v.value(), &dqdot_to_vdqi);
          auto& dqdot_to_v = element.qdot_to_v.gradient().value();
          dqdot_to_v.setZero();
          dqdot_to_v.middleCols(body.position_num_start, joint.getNumPositions()) = dqdot_to_vdqi;

          Matrix<Scalar, Dynamic, Dynamic> dv_to_qdotdqi(element.v_to_qdot.value().size(), joint.getNumPositions());
          joint.v2qdot(q_body, element.v_to_qdot.value(), &dv_to_qdotdqi);
          auto& dv_to_qdot = element.v_to_qdot.gradient().value();
          dv_to_qdot.setZero();
          dv_to_qdot.middleCols(body.position_num_start, joint.getNumPositions()) = dv_to_qdotdqi;
        }
        else {
          joint.qdot2v(q_body, element.qdot_to_v.value(), nullptr);
          joint.v2qdot(q_body, element.v_to_qdot.value(), nullptr);
        }

        if (compute_gradients) {
          // gradient of transform
          auto dT_body_to_parentdqi = dHomogTrans(T_body_to_parent, element.motion_subspace_in_body.value(), element.qdot_to_v.value()).eval();
          typename Gradient<typename Transform<Scalar, 3, Isometry>::MatrixType, Eigen::Dynamic>::type dT_body_to_parentdq(HOMOGENEOUS_TRANSFORM_SIZE, nq);
          dT_body_to_parentdq.setZero();
          dT_body_to_parentdq.middleCols(body.position_num_start, joint.getNumPositions()) = dT_body_to_parentdqi;
          element.dtransform_to_world_dq = matGradMultMat(parent_element.transform_to_world.matrix(), T_body_to_parent.matrix(), parent_element.dtransform_to_world_dq, dT_body_to_parentdq);

          // gradient of motion subspace in world
          Matrix<Scalar, Dynamic, Dynamic> dSdq(element.motion_subspace_in_body.value().size(), nq);
          dSdq.setZero();
          dSdq.middleCols(body.position_num_start, joint.getNumPositions()) = element.motion_subspace_in_body.gradient().value();
          element.motion_subspace_in_world.gradient().value() = dTransformSpatialMotion(element.transform_to_world, element.motion_subspace_in_body.value(), element.dtransform_to_world_dq, dSdq);
        }

        if (cache.hasV()) {
          const auto& v = cache.getV();
          if (joint.getNumVelocities()==0) { // for fixed joints
            element.twist_in_world.value() = parent_element.twist_in_world.value();
            if (compute_gradients) element.twist_in_world.gradient().value() = parent_element.twist_in_world.gradient().value();
            if (compute_JdotV) {
              element.motion_subspace_in_world_dot_times_v.value() = parent_element.motion_subspace_in_world_dot_times_v.value();
              if (compute_gradients) {
                element.motion_subspace_in_world_dot_times_v.gradient().value() = parent_element.motion_subspace_in_world_dot_times_v.gradient().value();
              }
            }
          } else {
            // twist
            auto v_body = v.middleRows(body.velocity_num_start, joint.getNumVelocities());

            GradientVar<Scalar, TWIST_SIZE, 1> joint_twist(TWIST_SIZE, 1, nq, gradient_order);
            joint_twist.value().noalias() = element.motion_subspace_in_world.value() * v_body;
            element.twist_in_world.value() = parent_element.twist_in_world.value() + joint_twist.value();

            if (compute_gradients) {
              // dtwistdq
              joint_twist.gradient().value() = matGradMult(element.motion_subspace_in_world.gradient().value(), v_body);
              element.twist_in_world.gradient().value() = parent_element.twist_in_world.gradient().value() + joint_twist.gradient().value();
            }

            if (compute_JdotV) {
              // Sdotv
              if (compute_gradients) {
                Matrix<Scalar, 6, Dynamic> dSdotVdqi(6, joint.getNumPositions()); // TODO: use block of dSdotv instead
                Matrix<Scalar, 6, Dynamic> dSdotVdvi(6, joint.getNumVelocities()); // TODO: use block of dSdotv instead
                joint.motionSubspaceDotTimesV(q_body, v_body, element.motion_subspace_in_body_dot_times_v.value(), &dSdotVdqi, &dSdotVdvi);
                auto& dSdotv = element.motion_subspace_in_body_dot_times_v.gradient().value();
                dSdotv.leftCols(joint.getNumPositions()) = dSdotVdqi;
                dSdotv.rightCols(joint.getNumVelocities()) = dSdotVdvi;
              }
              else {
                joint.motionSubspaceDotTimesV(q_body, v_body, element.motion_subspace_in_body_dot_times_v.value(), nullptr, nullptr);
              }

              // Jdotv
              auto joint_accel = crossSpatialMotion(element.twist_in_world.value(), joint_twist.value());
              joint_accel += transformSpatialMotion(element.transform_to_world, element.motion_subspace_in_body_dot_times_v.value());
              element.motion_subspace_in_world_dot_times_v.value() = parent_element.motion_subspace_in_world_dot_times_v.value() + joint_accel;

              if (compute_gradients) {
                // dJdotvdq
                // TODO: exploit sparsity better
                Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> dSdotVdq(TWIST_SIZE, nq);
                dSdotVdq.setZero();
                auto& dSdotv = element.motion_subspace_in_body_dot_times_v.gradient().value();
                dSdotVdq.middleCols(body.position_num_start, joint.getNumPositions()) = dSdotv.leftCols(joint.getNumPositions());
                auto dcrm_twist_joint_twistdq = dCrossSpatialMotion(element.twist_in_world.value(), joint_twist.value(), element.twist_in_world.gradient().value(), joint_twist.gradient().value());

                auto dJdotVdq = element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions);
                dJdotVdq = parent_element.motion_subspace_in_world_dot_times_v.gradient().value().leftCols(num_positions)
                           + dcrm_twist_joint_twistdq
                           + dTransformSpatialMotion(element.transform_to_world, element.motion_subspace_in_body_dot_times_v.value(), element.dtransform_to_world_dq, dSdotVdq);

                // dJdotvdv
                int nv_joint = joint.getNumVelocities();
                std::vector<int> v_indices;
                auto dtwistdv = geometricJacobian(cache, 0, i, 0, 0, false, &v_indices);
                int nv_branch = static_cast<int>(v_indices.size());

                Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> djoint_twistdv(TWIST_SIZE, nv_branch);
                djoint_twistdv.setZero();
                djoint_twistdv.rightCols(nv_joint) = element.motion_subspace_in_world.value();

                Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> djoint_acceldv(TWIST_SIZE, nv_branch);
                djoint_acceldv = dCrossSpatialMotion(element.twist_in_world.value(), joint_twist.value(), dtwistdv.value(), djoint_twistdv); // TODO: can probably exploit sparsity better
                auto dSdotVdvi = dSdotv.rightCols(joint.getNumVelocities());
                djoint_acceldv.rightCols(nv_joint) += transformSpatialMotion(element.transform_to_world, dSdotVdvi);

                auto dJdotVdv = element.motion_subspace_in_world_dot_times_v.gradient().value().rightCols(num_velocities);
                auto dJdotVdv_parent = parent_element.motion_subspace_in_world_dot_times_v.gradient().value().rightCols(num_velocities);
                dJdotVdv.setZero();
                for (int j = 0; j < nv_branch; j++) {
                  int v_index = v_indices[j];
                  dJdotVdv.col(v_index) = dJdotVdv_parent.col(v_index) + djoint_acceldv.col(j);
                }
              }
            }
          }
        }
      }
      else {
        element.transform_to_world.setIdentity();
        // motion subspace in body frame is empty
        // motion subspace in world frame is empty
        // qdot to v is empty
        // v to qdot is empty
        if (compute_gradients) {
          // gradient of transform
          element.dtransform_to_world_dq.setZero();
          // gradient of motion subspace in world is empty
        }
        if (cache.hasV()) {
          element.twist_in_world.value().setZero();
          element.motion_subspace_in_body.value().setZero();
          element.motion_subspace_in_world.value().setZero();
          element.qdot_to_v.value().setZero();
          element.v_to_qdot.value().setZero();

          if (compute_gradients) {
            element.twist_in_world.gradient().value().setZero();
            element.motion_subspace_in_body.gradient().value().setZero();
            element.motion_subspace_in_world.gradient().value().setZero();
            element.qdot_to_v.gradient().value().setZero();
            element.v_to_qdot.gradient().value().setZero();
          }
          if (compute_JdotV) {
            element.motion_subspace_in_body_dot_times_v.value().setZero();
            element.motion_subspace_in_world_dot_times_v.value().setZero();
            if (compute_gradients) {
              element.motion_subspace_in_body_dot_times_v.gradient().value().setZero();
              element.motion_subspace_in_world_dot_times_v.gradient().value().setZero();
            }
          }
        }
      }
    }

    cache.setJdotVCached(compute_JdotV && cache.hasV());
  };

  bool isBodyPartOfRobot(const RigidBody& body, const std::set<int>& robotnum) const;

  double getMass(const std::set<int>& robotnum = RigidBody::defaultRobotNumSet) const;

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMass(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> worldMomentumMatrix(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> worldMomentumMatrixDotTimesV(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> centroidalMomentumMatrix(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> centroidalMomentumMatrixDotTimesV(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet) const;

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, Eigen::Dynamic> centerOfMassJacobian(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet, bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION, 1> centerOfMassJacobianDotTimesV(KinematicsCache<Scalar>& cache, int gradient_order, const std::set<int>& robotnum = RigidBody::defaultRobotNumSet) const;

  template <typename DerivedA, typename DerivedB, typename DerivedC>
  void jointLimitConstraints(Eigen::MatrixBase<DerivedA> const & q, Eigen::MatrixBase<DerivedB> &phi, Eigen::MatrixBase<DerivedC> &J) const;

  size_t getNumJointLimitConstraints() const;

  int getNumContacts(const std::set<int> &body_idx) const;// = emptyIntSet);

  template <typename Derived>
  void getContactPositions(const KinematicsCache<typename Derived::Scalar>& cache, Eigen::MatrixBase<Derived> &pos, const std::set<int> &body_idx) const;// = emptyIntSet);

  template <typename Derived>
  void getContactPositionsJac(const KinematicsCache<typename Derived::Scalar>& cache, Eigen::MatrixBase<Derived> &J, const std::set<int> &body_idx) const;// = emptyIntSet);

//  template <typename Derived>
//  void getContactPositionsJacDot(MatrixBase<Derived> &Jdot, const std::set<int> &body_idx);// = emptyIntSet);
//

  /**
   * Computes CoP in world frame. Normal and point on contact plane should be in world frame too.
   */
  template <typename DerivedNormal, typename DerivedPoint>
  std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(const KinematicsCache<double>& cache, const std::vector< ForceTorqueMeasurement > & force_torque_measurements, const Eigen::MatrixBase<DerivedNormal> & normal, const Eigen::MatrixBase<DerivedPoint> & point_on_contact_plane) const;

  void findAncestorBodies(std::vector<int>& ancestor_bodies, int body) const;

  KinematicPath findKinematicPath(int start_body_or_frame_idx, int end_body_or_frame_idx) const;

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> massMatrix(KinematicsCache<Scalar>& cache, int gradient_order = 0) const;

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, 1> inverseDynamics(KinematicsCache<Scalar>& cache, std::map<int, std::unique_ptr< GradientVar<Scalar, TWIST_SIZE, 1> > >& f_ext, GradientVar<Scalar, Eigen::Dynamic, 1>* vd = nullptr, int gradient_order = 0) const;

  template <typename DerivedV>
  GradientVar<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorques(Eigen::MatrixBase<DerivedV> const & v, int gradient_order = 0) const;

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, DerivedPoints::ColsAtCompileTime> forwardKin(const KinematicsCache<typename DerivedPoints::Scalar>& cache, const Eigen::MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, int gradient_order) const;

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, Eigen::Dynamic> forwardKinJacobian(const KinematicsCache<typename DerivedPoints::Scalar>& cache, const Eigen::MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind, int new_body_or_frame_ind, int rotation_type, bool in_terms_of_qdot, int gradient_order) const;

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, Eigen::Dynamic> forwardKinPositionGradient(const KinematicsCache<Scalar>& cache, int npoints, int current_body_or_frame_ind, int new_body_or_frame_ind, int gradient_order) const;

  template <typename DerivedPoints>
  GradientVar<typename DerivedPoints::Scalar, Eigen::Dynamic, 1> forwardJacDotTimesV(const KinematicsCache<typename DerivedPoints::Scalar>& cache, const Eigen::MatrixBase<DerivedPoints>& points, int body_or_frame_ind, int base_or_frame_ind, int rotation_type, int gradient_order) const;

  template<typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobian(const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot = false, std::vector<int>* v_indices = nullptr) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> geometricJacobianDotTimesV(const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> relativeTwist(const KinematicsCache<Scalar>& cache, int base_or_frame_ind, int body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order) const;

  template <typename Scalar>
  GradientVar<Scalar, TWIST_SIZE, 1> transformSpatialAcceleration(const KinematicsCache<Scalar>& cache, const GradientVar<Scalar, TWIST_SIZE, 1>& spatial_acceleration, int base_or_frame_ind, int body_or_frame_ind, int old_body_or_frame_ind, int new_body_or_frame_ind) const;

  template<typename Scalar>
  GradientVar<Scalar, SPACE_DIMENSION + 1, SPACE_DIMENSION + 1> relativeTransform(const KinematicsCache<Scalar>& cache, int base_or_frame_ind, int body_or_frame_ind, int gradient_order) const;

  void computeContactJacobians(const KinematicsCache<double>& cache, Eigen::VectorXi const & idxA, Eigen::VectorXi const & idxB, Eigen::Map<Eigen::Matrix3Xd> const & xA, Eigen::Map<Eigen::Matrix3Xd> const & xB, const bool compute_second_derivatives, Eigen::MatrixXd & J, Eigen::MatrixXd & dJ) const;

  DrakeCollision::ElementId addCollisionElement(const RigidBody::CollisionElement& element, const std::shared_ptr<RigidBody>& body, std::string group_name);

  void updateCollisionElements(const RigidBody& body, const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world);

  void updateStaticCollisionElements();

  void updateDynamicCollisionElements(const KinematicsCache<double>& kin_cache);

  void getTerrainContactPoints(const RigidBody& body, Eigen::Matrix3Xd &terrain_points) const;

  bool collisionRaycast(const KinematicsCache<double>& cache, const Eigen::Matrix3Xd &origins, const Eigen::Matrix3Xd &ray_endpoints, Eigen::VectorXd &distances, bool use_margins=false);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi,
                       Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA,
                       Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx,
                       std::vector<int>& bodyB_idx,
                       const std::vector<DrakeCollision::ElementId>& ids_to_check,
                       bool use_margins);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi,
                       Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx,
                       std::vector<int>& bodyB_idx,
                       const std::vector<int>& bodies_idx,
                       const std::set<std::string>& active_element_groups,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx,
                       std::vector<int>& bodyB_idx,
                       const std::vector<int>& bodies_idx,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx,
                       std::vector<int>& bodyB_idx,
                       const std::set<std::string>& active_element_groups,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx,
                       std::vector<int>& bodyB_idx,
                        bool use_margins = true);


  bool allCollisions(const KinematicsCache<double>& cache,
                     std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                     Eigen::Matrix3Xd& ptsA, Eigen::Matrix3Xd& ptsB,
                     bool use_margins = true);

  void potentialCollisions(const KinematicsCache<double>& cache,
                           Eigen::VectorXd& phi,
                           Eigen::Matrix3Xd& normal,
                           Eigen::Matrix3Xd& xA,
                           Eigen::Matrix3Xd& xB,
                           std::vector<int>& bodyA_idx,
                           std::vector<int>& bodyB_idx,
                           bool use_margins = true);
  //bool closestDistanceAllBodies(VectorXd& distance, MatrixXd& Jd);

  virtual std::vector<size_t> collidingPoints(const KinematicsCache<double>& cache,
                                              const std::vector<Eigen::Vector3d>& points,
        double collision_threshold);

  void warnOnce(const std::string& id, const std::string& msg);

  std::shared_ptr<RigidBody> findLink(std::string linkname, int robot=-1) const;
  int findLinkId(const std::string& linkname, int robot = -1) const;
  std::shared_ptr<RigidBody> findJoint(std::string jointname, int robot=-1) const;
  int findJointId(const std::string& linkname, int robot = -1) const;
  //@param robot   the index of the robot. robot = -1 means to look at all the robots

  std::string getBodyOrFrameName(int body_or_frame_id) const;
  //@param body_or_frame_id   the index of the body or the id of the frame.

  // TODO: remove parseBodyOrFrameID methods
  template <typename Scalar>
  int parseBodyOrFrameID(const int body_or_frame_id, Eigen::Transform<Scalar, 3, Eigen::Isometry>* Tframe) const;
  int parseBodyOrFrameID(const int body_or_frame_id) const;

  template <typename Scalar>
  GradientVar<Scalar, Eigen::Dynamic, 1> positionConstraints(const KinematicsCache<Scalar>& cache, int gradient_order) const;

  size_t getNumPositionConstraints() const;

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> transformVelocityMappingToPositionDotMapping(
      const KinematicsCache<typename Derived::Scalar>& cache, const Eigen::MatrixBase<Derived>& mat) const;
  
  /*
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> transformPositionDotMappingToVelocityMapping(
      const KinematicsCache<typename Derived::Scalar>& cache, const Eigen::MatrixBase<Derived>& mat) const;
  */
  
  template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> transformPositionDotMappingToVelocityMapping(
    const KinematicsCache<typename Derived::Scalar>& cache, const Eigen::MatrixBase<Derived>& mat) const
{
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> ret(mat.rows(), num_velocities);
  int ret_col_start = 0;
  int mat_col_start = 0;
  for (auto it = bodies.begin(); it != bodies.end(); ++it) {
    RigidBody& body = **it;
    if (body.hasParent()) {
      const DrakeJoint& joint = body.getJoint();
      const auto& element = cache.getElement(body);
      ret.middleCols(ret_col_start, joint.getNumVelocities()).noalias() = mat.middleCols(mat_col_start, joint.getNumPositions()) * element.v_to_qdot.value();
      ret_col_start += joint.getNumVelocities();
      mat_col_start += joint.getNumPositions();
    }
  }
  return ret;
};

  template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Eigen::Dynamic> compactToFull(
    const Eigen::MatrixBase<Derived>& compact, const std::vector<int>& joint_path, bool in_terms_of_qdot) const {
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
  Eigen::VectorXd joint_limit_min;
  Eigen::VectorXd joint_limit_max;

  // Rigid body objects
  std::vector<std::shared_ptr<RigidBody> > bodies;

  // Rigid body frames
  std::vector<std::shared_ptr<RigidBodyFrame> > frames;

  // Rigid body actuators
  std::vector<RigidBodyActuator,Eigen::aligned_allocator<RigidBodyActuator> > actuators;

  // Rigid body loops
  std::vector<RigidBodyLoop,Eigen::aligned_allocator<RigidBodyLoop> > loops;

  Eigen::Matrix<double,TWIST_SIZE,1> a_grav;
  Eigen::MatrixXd B;  // the B matrix maps inputs into joint-space forces

private:
  //helper functions for contactConstraints
  void accumulateContactJacobian(const KinematicsCache<double>& cache, const int bodyInd, Eigen::Matrix3Xd const & bodyPoints, std::vector<size_t> const & cindA, std::vector<size_t> const & cindB, Eigen::MatrixXd & J) const;
  void accumulateSecondOrderContactJacobian(const KinematicsCache<double>& cache, const int bodyInd, Eigen::Matrix3Xd const & bodyPoints, std::vector<size_t> const & cindA, std::vector<size_t> const & cindB, Eigen::MatrixXd & dJ) const;

  template <typename Scalar>
  void updateCompositeRigidBodyInertias(KinematicsCache<Scalar>& cache, int gradient_order) const;

  bool initialized;


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
