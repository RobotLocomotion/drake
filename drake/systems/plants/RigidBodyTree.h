#pragma once

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <set>
#include <stdexcept>
#include <unordered_map>

#include "drake/common/drake_deprecated.h"
#include "drake/math/rotation_matrix.h"
#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/plants/KinematicPath.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/DrakeJoint.h"
#include "drake/systems/plants/pose_map.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"
#include "drake/util/drakeUtil.h"

#define BASIS_VECTOR_HALF_COUNT \
  2  // number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;

class DRAKERBM_EXPORT RigidBodyActuator {
 public:
  RigidBodyActuator(
      const std::string& name, const RigidBody* body, double reduction = 1.0,
      double effort_limit_min = -std::numeric_limits<double>::infinity(),
      double effort_limit_max = std::numeric_limits<double>::infinity())
      : name(name),
        body(body),
        reduction(reduction),
        effort_limit_min(effort_limit_min),
        effort_limit_max(effort_limit_max) {}

  const std::string name;
  const RigidBody* const body;
  const double reduction;
  const double effort_limit_min;
  const double effort_limit_max;
};

class DRAKERBM_EXPORT RigidBodyLoop {
 public:
  //
  // Constructs a RigidBodyLoop between two frames. Is this the correct API?
  // TODO(amcastro-tri): review the correctness of this API
  RigidBodyLoop(std::shared_ptr<RigidBodyFrame> _frameA,
                std::shared_ptr<RigidBodyFrame> _frameB,
                const Eigen::Vector3d& _axis)
      : frameA(_frameA), frameB(_frameB), axis(_axis) {}

  const std::shared_ptr<RigidBodyFrame> frameA, frameB;
  const Eigen::Vector3d axis;

  friend std::ostream& operator<<(std::ostream& os, const RigidBodyLoop& obj);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

class DRAKERBM_EXPORT RigidBodyTree {
 public:
  /**
   * Defines the name of the rigid body within a rigid body tree that represents
   * the world.
   */
  // TODO(amcastro-tri): Move the concept of world to an actual world
  // abstraction. See issue #2318.
  static const char* const kWorldLinkName;

  RigidBodyTree(const std::string& urdf_filename,
                const DrakeJoint::FloatingBaseType floating_base_type =
                    DrakeJoint::ROLLPITCHYAW);
  RigidBodyTree(void);
  virtual ~RigidBodyTree(void);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use drake::parsers::urdf::AddRobotFromURDFString.")
#endif
  void addRobotFromURDFString(
      const std::string& xml_string, const std::string& root_dir = ".",
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::ROLLPITCHYAW,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use drake::parsers::urdf::AddRobotFromURDFString.")
#endif
  void addRobotFromURDFString(
      const std::string& xml_string,
      std::map<std::string, std::string>& package_map,
      const std::string& root_dir = ".",
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::ROLLPITCHYAW,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use drake::parsers::urdf::AddRobotFromURDF.")
#endif
  void addRobotFromURDF(
      const std::string& urdf_filename,
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::ROLLPITCHYAW,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use drake::parsers::urdf::AddRobotFromURDF.")
#endif
  void addRobotFromURDF(
      const std::string& urdf_filename,
      std::map<std::string, std::string>& package_map,
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::ROLLPITCHYAW,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use drake::parsers::sdf::AddRobotFromSDF.")
#endif
  void addRobotFromSDF(const std::string& sdf_filename,
                       const DrakeJoint::FloatingBaseType floating_base_type =
                           DrakeJoint::QUATERNION,
                       std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  /**
   * Returns an integer that can be used to uniquely identify a model
   * within this rigid body tree. Note that this method is not thread safe!
   */
  int get_next_model_id() { return next_model_id_++; }

  /**
   * Returns an integer that will be used as a unique ID for the next model
   * to be added within the rigid body tree.
   */
  int get_current_model_id() { return next_model_id_; }

  void addFrame(std::shared_ptr<RigidBodyFrame> frame);

  std::map<std::string, int> computePositionNameToIndexMap() const;

  void surfaceTangents(
      Eigen::Map<Eigen::Matrix3Xd> const& normals,
      std::vector<Eigen::Map<Eigen::Matrix3Xd>>& tangents) const;

  /*!
   * Updates the frame of collision elements to be equal to the joint's frame.
   *
   * @param eid The ID of the collision element to update.
   * @param transform_body_to_joint The transform from the model's
   * body frame to the joint frame.
   * @return true if the collision element was successfully updated.
   * False can be returned if a collision element with the specified eid
   * cannot be found.
   */
  bool transformCollisionFrame(
      const DrakeCollision::ElementId& eid,
      const Eigen::Isometry3d& transform_body_to_joint);

  void compile(void);  // call me after the model is loaded

  Eigen::VectorXd getZeroConfiguration() const;

  Eigen::VectorXd getRandomConfiguration(
      std::default_random_engine& generator) const;

  // akin to the coordinateframe names in matlab
  std::string getPositionName(int position_num) const;
  std::string getVelocityName(int velocity_num) const;
  std::string getStateName(int state_num) const;

  void drawKinematicTree(std::string graphviz_dotfile_filename) const;

  /// Initializes a `KinematicsCache` with the given configuration @p q,
  /// computes the kinematics, and returns the cache.
  ///
  /// This method is explicitly instantiated in RigidBodyTree.cpp for a
  /// small set of supported `DerivedQ`.
  template <typename DerivedQ>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(
      const Eigen::MatrixBase<DerivedQ>& q) const;

  /// Initializes a `KinematicsCache` with the given configuration @p q
  /// and velocity @p v, computes the kinematics, and returns the cache.
  ///
  /// This method is explicitly instantiated in RigidBodyTree.cpp for a
  /// small set of supported `DerivedQ` and `DerivedV`.
  template <typename DerivedQ, typename DerivedV>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v, bool compute_JdotV = true) const;

  /// Computes the kinematics on the given @p cache.
  ///
  /// This method is explicitly instantiated in RigidBodyTree.cpp for a
  /// small set of supported Scalar types.
  template <typename Scalar>
  void doKinematics(KinematicsCache<Scalar>& cache,
                    bool compute_JdotV = false) const;

  bool isBodyPartOfRobot(const RigidBody& body,
                         const std::set<int>& robotnum) const;

  double getMass(const std::set<int>& robotnum = default_robot_num_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> centerOfMass(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> worldMomentumMatrix(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> worldMomentumMatrixDotTimesV(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> centroidalMomentumMatrix(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> centroidalMomentumMatrixDotTimesV(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, SPACE_DIMENSION, Eigen::Dynamic> centerOfMassJacobian(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, SPACE_DIMENSION, 1> centerOfMassJacobianDotTimesV(
      KinematicsCache<Scalar>& cache,
      const std::set<int>& robotnum = default_robot_num_set) const;

  template <typename DerivedA, typename DerivedB, typename DerivedC>
  void jointLimitConstraints(Eigen::MatrixBase<DerivedA> const& q,
                             Eigen::MatrixBase<DerivedB>& phi,
                             Eigen::MatrixBase<DerivedC>& J) const;

  size_t getNumJointLimitConstraints() const;

  int getNumContacts(const std::set<int>& body_idx) const;  // = emptyIntSet);

  /**
   * Computes CoP in world frame. Normal and point on contact plane should be in
   * world frame too.
   */
  template <typename DerivedNormal, typename DerivedPoint>
  std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(
      const KinematicsCache<double>& cache,
      const std::vector<ForceTorqueMeasurement>& force_torque_measurements,
      const Eigen::MatrixBase<DerivedNormal>& normal,
      const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane) const;

  void findAncestorBodies(std::vector<int>& ancestor_bodies, int body) const;

  KinematicPath findKinematicPath(int start_body_or_frame_idx,
                                  int end_body_or_frame_idx) const;

  /** \brief Compute the positive definite mass (configuration space) matrix \f$
   *H(q) \f$, defined by \f$T = \frac{1}{2} v^T H(q) v \f$, where \f$ T \f$ is
   *kinetic energy.
   *
   * The mass matrix also appears in the manipulator equations
   *  \f[
   *  H(q) \dot{v} + C(q, v, f_\text{ext}) = B(q) u
   * \f]
   *
   * \param cache a KinematicsCache constructed given \f$ q \f$
   * \return the mass matrix \f$ H(q) \f$
   */
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> massMatrix(
      KinematicsCache<Scalar>& cache) const;

  /** \brief Compute the term \f$ C(q, v, f_\text{ext}) \f$ in the manipulator
  *equations
  *  \f[
  *  H(q) \dot{v} + C(q, v, f_\text{ext}) = B(q) u
  * \f]
  *
  * Convenience method that calls inverseDynamics with \f$ \dot{v} = 0 \f$. See
  *inverseDynamics for argument descriptions.
  * \see inverseDynamics
  */
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> dynamicsBiasTerm(
      KinematicsCache<Scalar>& cache,
      const eigen_aligned_unordered_map<RigidBody const*,
                                        drake::TwistVector<Scalar>>& f_ext,
      bool include_velocity_terms = true) const;

  /** \brief Compute
  * \f[
  *  H(q) \dot{v} + C(q, v, f_\text{ext})
  * \f]
  * that is, the left hand side of the manipulator equations
  *  \f[
  *  H(q) \dot{v} + C(q, v, f_\text{ext}) = B(q) u
  * \f]
  *
  * Note that the 'dynamics bias term' \f$ C(q, v, f_\text{ext}) \f$ can be
  *computed by simply setting \f$ \dot{v} = 0\f$.
  * Note also that if only the gravitational terms contained in \f$ C(q, v,
  *f_\text{ext}) \f$ are required, one can set \a include_velocity_terms to
  *false.
  * Alternatively, one can pass in a KinematicsCache created with \f$ v = 0\f$
  *or without specifying the velocity vector.
  *
  * Algorithm: recursive Newton-Euler. Does not explicitly compute mass matrix.
  * \param cache a KinematicsCache constructed given \f$ q \f$ and \f$ v \f$
  * \param f_ext external wrenches exerted upon bodies. Expressed in body frame.
  * \param vd \f$ \dot{v} \f$
  * \param include_velocity_terms whether to include velocity-dependent terms in
  *\f$ C(q, v, f_\text{ext}) \f$. Setting \a include_velocity_terms to false is
  *Equivalent to setting \f$ v = 0 \f$
  * \return \f$ H(q) \dot{v} + C(q, v, f_\text{ext}) \f$
  */
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inverseDynamics(
      KinematicsCache<Scalar>& cache,
      const eigen_aligned_unordered_map<RigidBody const*,
                                        drake::TwistVector<Scalar>>& f_ext,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vd,
      bool include_velocity_terms = true) const;

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorques(
      Eigen::MatrixBase<DerivedV> const& v) const;

  template <
      typename Scalar,
      typename DerivedPoints>  // not necessarily any relation between the two;
  // a major use case is having an AutoDiff
  // KinematicsCache, but double points matrix
  Eigen::Matrix<Scalar, 3, DerivedPoints::ColsAtCompileTime>
  transformPoints(const KinematicsCache<Scalar>& cache,
                  const Eigen::MatrixBase<DerivedPoints>& points,
                  int from_body_or_frame_ind, int to_body_or_frame_ind) const {
    static_assert(DerivedPoints::RowsAtCompileTime == 3 ||
                      DerivedPoints::RowsAtCompileTime == Eigen::Dynamic,
                  "points argument has wrong number of rows");
    auto T =
        relativeTransform(cache, to_body_or_frame_ind, from_body_or_frame_ind);
    return T * points.template cast<Scalar>();
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, 4, 1> relativeQuaternion(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind) const {
    return drake::math::rotmat2quat(
        relativeTransform(cache, to_body_or_frame_ind, from_body_or_frame_ind)
            .linear());
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, 3, 1> relativeRollPitchYaw(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind) const {
    return drake::math::rotmat2rpy(
        relativeTransform(cache, to_body_or_frame_ind, from_body_or_frame_ind)
            .linear());
  }

  template <typename Scalar, typename DerivedPoints>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> transformPointsJacobian(
      const KinematicsCache<Scalar>& cache,
      const Eigen::MatrixBase<DerivedPoints>& points,
      int from_body_or_frame_ind, int to_body_or_frame_ind,
      bool in_terms_of_qdot) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, QUAT_SIZE, Eigen::Dynamic> relativeQuaternionJacobian(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind, bool in_terms_of_qdot) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, RPY_SIZE, Eigen::Dynamic> relativeRollPitchYawJacobian(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind, bool in_terms_of_qdot) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
  forwardKinPositionGradient(const KinematicsCache<Scalar>& cache, int npoints,
                             int from_body_or_frame_ind,
                             int to_body_or_frame_ind) const;

  template <typename Scalar, typename DerivedPoints>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> transformPointsJacobianDotTimesV(
      const KinematicsCache<Scalar>& cache,
      const Eigen::MatrixBase<DerivedPoints>& points,
      int from_body_or_frame_ind, int to_body_or_frame_ind) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> relativeQuaternionJacobianDotTimesV(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
  relativeRollPitchYawJacobianDotTimesV(const KinematicsCache<Scalar>& cache,
                                        int from_body_or_frame_ind,
                                        int to_body_or_frame_ind) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> geometricJacobian(
      const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
      int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind,
      bool in_terms_of_qdot = false,
      std::vector<int>* v_indices = nullptr) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> geometricJacobianDotTimesV(
      const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
      int end_effector_body_or_frame_ind,
      int expressed_in_body_or_frame_ind) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> relativeTwist(
      const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
      int body_or_frame_ind, int expressed_in_body_or_frame_ind) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> transformSpatialAcceleration(
      const KinematicsCache<Scalar>& cache,
      const drake::TwistVector<Scalar>& spatial_acceleration,
      int base_or_frame_ind, int body_or_frame_ind, int old_body_or_frame_ind,
      int new_body_or_frame_ind) const;

  template <typename Scalar>
  Eigen::Transform<Scalar, SPACE_DIMENSION, Eigen::Isometry> relativeTransform(
      const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
      int body_or_frame_ind) const;

  /** computeContactJacobians
   * @brief Computes the jacobian for many points in the format currently used
   * by matlab.  (possibly should be scheduled for deletion, taking
   * accumulateContactJacobians with it)
   */
  template <typename Scalar>
  void computeContactJacobians(
      const KinematicsCache<Scalar>& cache,
      Eigen::Ref<const Eigen::VectorXi> const& idxA,
      Eigen::Ref<const Eigen::VectorXi> const& idxB,
      Eigen::Ref<const Eigen::Matrix3Xd> const& xA,
      Eigen::Ref<const Eigen::Matrix3Xd> const& xB,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& J) const;

  DrakeCollision::ElementId addCollisionElement(
      const RigidBody::CollisionElement& element, RigidBody& body,
      const std::string& group_name);

  template <class UnaryPredicate>
  void removeCollisionGroupsIf(UnaryPredicate test) {
    for (const auto& body_ptr : bodies) {
      std::vector<std::string> names_of_groups_to_delete;
      for (const auto& group : body_ptr->collision_element_groups) {
        const std::string& group_name = group.first;
        if (test(group_name)) {
          auto& ids = body_ptr->collision_element_ids;
          for (const auto& id : group.second) {
            ids.erase(std::find(ids.begin(), ids.end(), id));
            collision_model->removeElement(id);
          }
          names_of_groups_to_delete.push_back(group_name);
        }
      }
      for (const auto& group_name : names_of_groups_to_delete) {
        body_ptr->collision_element_groups.erase(group_name);
      }
    }
  }

  void updateCollisionElements(
      const RigidBody& body,
      const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world);

  void updateStaticCollisionElements();

  void updateDynamicCollisionElements(const KinematicsCache<double>& kin_cache);

  void getTerrainContactPoints(const RigidBody& body,
                               Eigen::Matrix3Xd& terrain_points) const;

  bool collisionRaycast(const KinematicsCache<double>& cache,
                        const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints,
                        Eigen::VectorXd& distances, bool use_margins = false);
  bool collisionRaycast(const KinematicsCache<double>& cache,
                        const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints,
                        Eigen::VectorXd& distances, Eigen::Matrix3Xd& normals,
                        bool use_margins = false);

  /** collisionDetectFromPoints
   * @brief Computes the (signed) distance from the given points to the nearest
   * body in the RigidBodyTree.
   */
  void collisionDetectFromPoints(const KinematicsCache<double>& cache,
                                 const Eigen::Matrix3Xd& points,
                                 Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                                 Eigen::Matrix3Xd& x, Eigen::Matrix3Xd& body_x,
                                 std::vector<int>& body_idx, bool use_margins);

  bool collisionDetect(
      const KinematicsCache<double>& cache, Eigen::VectorXd& phi,
      Eigen::Matrix3Xd& normal, Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
      std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
      const std::vector<DrakeCollision::ElementId>& ids_to_check,
      bool use_margins);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                       const std::vector<int>& bodies_idx,
                       const std::set<std::string>& active_element_groups,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                       const std::vector<int>& bodies_idx,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                       const std::set<std::string>& active_element_groups,
                       bool use_margins = true);

  bool collisionDetect(const KinematicsCache<double>& cache,
                       Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                       Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                       std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                       bool use_margins = true);

  bool allCollisions(const KinematicsCache<double>& cache,
                     std::vector<int>& bodyA_idx, std::vector<int>& bodyB_idx,
                     Eigen::Matrix3Xd& ptsA, Eigen::Matrix3Xd& ptsB,
                     bool use_margins = true);

  void potentialCollisions(const KinematicsCache<double>& cache,
                           Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
                           Eigen::Matrix3Xd& xA, Eigen::Matrix3Xd& xB,
                           std::vector<int>& bodyA_idx,
                           std::vector<int>& bodyB_idx,
                           bool use_margins = true);

  /** Computes the point of closest approach between bodies in the
   RigidBodyTree that are in contact.

   @param cache[in] a KinematicsCache constructed by RigidBodyTree::doKinematics
   given `q` and `v`.

   Collision points are returned as a vector of PointPair's.
   See the documentation for PointPair for details. The collision point on the
   surface of each body is stored in the PointPair structure in the frame of the
   corresponding body.

   @param use_margins[in] If `true` the model uses the representation with
   margins. If `false`, the representation without margins is used instead.
   **/
  std::vector<DrakeCollision::PointPair> ComputeMaximumDepthCollisionPoints(
      const KinematicsCache<double>& cache, bool use_margins = true);

  virtual bool collidingPointsCheckOnly(
      const KinematicsCache<double>& cache,
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  virtual std::vector<size_t> collidingPoints(
      const KinematicsCache<double>& cache,
      const std::vector<Eigen::Vector3d>& points, double collision_threshold);

  /**
   * Finds a body with the specified \p body_name belonging to a model
   * with the specified \p model_name and \p model_id. Note that if
   * \p model_name is the empty string and \p model_id is -1, every model is
   * searched. If \p model_name and \p model_id are inconsistent, no body
   * will be found and an exception will be thrown.
   *
   * @param[in] body_name The name of the body to find.
   * @param[in] model_name The name of the model to which the body belongs. If
   * this value is an empty string, every model is searched.
   * @param[in] model_id The ID of the model to which the body belongs. If this
   * value is -1, every model is searched.
   * @throws std::logic_error if multiple matching bodies are found or no
   * matching bodies are found.
   */
  RigidBody* FindBody(const std::string& body_name,
                      const std::string& model_name = "",
                      int model_id = -1) const;

/**
 * This is a deprecated version of `FindBody(...)`. Please use `FindBody(...)`
 * instead.
 */
#ifndef SWIG
  DRAKE_DEPRECATED("Please use RigidBodyTree::FindBody instead.")
#endif
  RigidBody* findLink(const std::string& link_name,
                      const std::string& model_name = "",
                      int model_id = -1) const;

  /**
   * Obtains the index of a rigid body within this rigid body tree. The rigid
   * body tree maintains a vector of pointers to all rigid bodies that are part
   * of the rigid body tree. The index of a rigid body is the index within this
   * vector at which a pointer to the rigid body is stored.
   *
   * @param[in] body_name The body whose index we want to find. It should
   * be unique within the searched models, otherwise an exception will be
   * thrown.
   * @param[in] model_id The ID of the model. This parameter is optional. If
   * supplied, only the model with the specified ID is searched; otherwise, all
   * models are searched.
   * @return The index of the specified rigid body.
   * @throws std::logic_error if no rigid body with the specified \p body_name
   * and \p model_id was found or if multiple matching rigid bodies were found.
   */
  int FindBodyIndex(const std::string& body_name, int model_id = -1) const;

/**
 * This is a deprecated version of `FindBodyIndex(...)`. Please use
 * `FindBodyIndex(...)` instead.
 */
#ifndef SWIG
  DRAKE_DEPRECATED("Pleasse use RigidBodyTree::FindBodyIndex instead.")
#endif
  int findLinkId(const std::string& link_name, int model_id = -1) const;

  // TODO(amcastro-tri): The name of this method is misleading.
  // It returns a RigidBody when the user seems to request a joint.
  /**
   * Obtains a pointer to the rigid body whose parent joint is named
   * \p joint_name and is part of a model with ID \p model_id.
   *
   * @param[in] joint_name The name of the joint to find.
   * @param[in] model_id The ID of the model that contains the joint. This
   * parameter is optional. If supplied, only that model is searched; otherwise,
   * all models are searched.
   * @return A pointer to the rigid body whose joint is the one being searched
   * for.
   * @throws std::logic_error if no joint is found with the given name within
   * the searched model(s).
   */
  RigidBody* findJoint(const std::string& joint_name, int model_id = -1) const;

  int findJointId(const std::string& joint_name, int model_id = -1) const;

  /**
   * Finds a frame of the specified \p frame_name belonging to a model with the
   * specified \p model_id.
   *
   * @param[in] frame_name The name of the frame to find.
   * @param[in] model_id The ID of the model to which the frame belongs. If this
   * value is -1, search all models.
   * @throws std::logic_error if multiple matching frames are found.
   */
  std::shared_ptr<RigidBodyFrame> findFrame(const std::string& frame_name,
                                            int model_id = -1) const;

  std::string getBodyOrFrameName(int body_or_frame_id) const;
  // @param body_or_frame_id the index of the body or the id of the frame.

  /**
   * Obtains a rigid body actuator from this rigid body tree. The actuator is
   * selected based on its name.
   *
   * @param name The name of the rigid body actuator to get.
   * @returns A const reference to the rigid body actuator with name @p name.
   * @throws std::invalid_argument if no rigid body actuator with name @p name
   * exists.
   */
  const RigidBodyActuator& GetActuator(const std::string& name) const;

  // TODO(tkoolen): remove parseBodyOrFrameID methods
  template <typename Scalar>
  int parseBodyOrFrameID(
      const int body_or_frame_id,
      Eigen::Transform<Scalar, 3, Eigen::Isometry>* Tframe) const;
  int parseBodyOrFrameID(const int body_or_frame_id) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> positionConstraints(
      const KinematicsCache<Scalar>& cache) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
  positionConstraintsJacobian(const KinematicsCache<Scalar>& cache,
                              bool in_terms_of_qdot = true) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> positionConstraintsJacDotTimesV(
      const KinematicsCache<Scalar>& cache) const;

  size_t getNumPositionConstraints() const;

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
  compactToFull(const Eigen::MatrixBase<Derived>& compact,
                const std::vector<int>& joint_path,
                bool in_terms_of_qdot) const {
    /*
     * This method is used after calling geometric Jacobian, where compact is
     * the Jacobian on the joints that are on the kinematic path; if we want to
     * reconstruct the full Jacobian on all joints, then we should call this
     * method.
     */
    int ncols = in_terms_of_qdot ? num_positions_ : num_velocities_;
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                  Eigen::Dynamic> full(compact.rows(), ncols);
    full.setZero();
    int compact_col_start = 0;
    for (std::vector<int>::const_iterator it = joint_path.begin();
         it != joint_path.end(); ++it) {
      RigidBody& body = *bodies[*it];
      int ncols_joint = in_terms_of_qdot ? body.getJoint().getNumPositions()
                                         : body.getJoint().getNumVelocities();
      int col_start =
          in_terms_of_qdot ? body.position_num_start : body.velocity_num_start;
      full.middleCols(col_start, ncols_joint) =
          compact.middleCols(compact_col_start, ncols_joint);
      compact_col_start += ncols_joint;
    }
    return full;
  }

  /**
   * A toString method for this class.
   */
  friend DRAKERBM_EXPORT std::ostream& operator<<(std::ostream&,
                                                  const RigidBodyTree&);

  /**
   * @brief Adds and takes ownership of a rigid body.
   *
   * A RigidBodyTree is the sole owner and manager of the RigidBody's in it.
   * A body is assigned a unique id (RigidBody::id()) when added to a
   * RigidBodyTree. This unique id can be use to access a body with the accessor
   * RigidBodyTree::body.
   *
   * @param[in] body The rigid body to add to this rigid body tree.
   */
  void add_rigid_body(std::unique_ptr<RigidBody> body);

  /**
   * Adds one floating joint to each link specified in the list of link indicies
   * that does not already have a parent. Typically, the list of link indices is
   * created while calling add_rigid_body(). The purpose of the floating joint
   * is to connect the links and of their child branches to the rigid body tree.
   *
   * @param floating_base_type The floating joint's type.
   * @param link_indices A list of link indexes to check. A floating joint is
   * added to any link in this list that does not have a parent joint.
   * @param weld_to_frame The frame to which the floating joint should attach
   * the parent-less non-world links. This parameter may be nullptr, in which
   * case the link is welded to the world with zero offset.
   * @param pose_map A mapping where the key is the link's name and the value
   * is the transform from the frame of the link to the frame of the model
   * to which the link belongs.
   * @return The number of floating joint added to this rigid body tree.
   * @throws A std::runtime_error if the floating_base_type is unrecognized or
   * zero floating joints were added to the model.
   */
  int AddFloatingJoint(
      DrakeJoint::FloatingBaseType floating_base_type,
      const std::vector<int>& link_indices,
      const std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr,
      const PoseMap* pose_map = nullptr);

  /**
   * @brief Returns a mutable reference to the RigidBody associated with the
   * world in the model. This is the root of the RigidBodyTree.
   */
  RigidBody& world() { return *bodies[0]; }

  /**
   * @brief Returns a const reference to the RigidBody associated with the
   * world in the model. This is the root of the RigidBodyTree.
   */
  const RigidBody& world() const { return *bodies[0]; }

  /**
   * An accessor to the number of position states outputted by this rigid body
   * system.
   */
  int number_of_positions() const { return num_positions_; }

  /**
   * An accessor to the number of velocity states outputted by this rigid body
   * system.
   */
  int number_of_velocities() const { return num_velocities_; }

 public:
  static const std::set<int> default_robot_num_set;

  Eigen::VectorXd joint_limit_min;
  Eigen::VectorXd joint_limit_max;

  // Rigid body objects
  // TODO(amcastro-tri): make private and start using accessors body(int).
  // TODO(amcastro-tri): rename to bodies_ to follow Google's style guide once.
  // accessors are used throughout the code.
  std::vector<std::unique_ptr<RigidBody>> bodies;

  // Rigid body frames
  std::vector<std::shared_ptr<RigidBodyFrame>> frames;

  // Rigid body actuators
  std::vector<RigidBodyActuator, Eigen::aligned_allocator<RigidBodyActuator>>
      actuators;

  // Rigid body loops
  std::vector<RigidBodyLoop, Eigen::aligned_allocator<RigidBodyLoop>> loops;

  drake::TwistVector<double> a_grav;
  Eigen::MatrixXd B;  // the B matrix maps inputs into joint-space forces

 private:
  // The number of position states in this rigid body tree.
  int num_positions_{};

  // The number of velocity states in this rigid body tree.
  int num_velocities_{};

  // Remembers the ID that should be assigned to the next model added to this
  // rigid body tree.
  int next_model_id_{};

  // helper functions for contactConstraints
  template <typename Scalar>
  void accumulateContactJacobian(
      const KinematicsCache<Scalar>& cache, const int bodyInd,
      Eigen::Matrix3Xd const& bodyPoints, std::vector<size_t> const& cindA,
      std::vector<size_t> const& cindB,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& J) const;

  template <typename Scalar>
  void updateCompositeRigidBodyInertias(KinematicsCache<Scalar>& cache) const;

  // Reorder body list to make sure parents are before children in
  // the list RigidBodyTree::bodies.
  //
  // See RigidBodyTree::compile
  void SortTree();

  // collision_model maintains a collection of the collision geometry in the
  // RBM for use in collision detection of different kinds. Small margins are
  // applied to all collision geometry when that geometry is added, to improve
  // the numerical stability of contact gradients taken using the model.
  std::unique_ptr<DrakeCollision::Model> collision_model;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  // The following was required for building w/ DRAKERBM_EXPORT on windows (due
  // to the unique_ptrs).  See
  // http://stackoverflow.com/questions/8716824/cannot-access-private-member-error-only-when-class-has-export-linkage
 private:
  RigidBodyTree(const RigidBodyTree&);
  RigidBodyTree& operator=(const RigidBodyTree&) { return *this; }

  std::set<std::string> already_printed_warnings;
  bool initialized_{false};
};
