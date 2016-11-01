#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "drake/common/constants.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/drake_export.h"
#include "drake/systems/plants/ForceTorqueMeasurement.h"
#include "drake/systems/plants/KinematicPath.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/collision/DrakeCollision.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/pose_map.h"
#include "drake/systems/plants/rigid_body_actuator.h"
#include "drake/systems/plants/rigid_body_loop.h"
#include "drake/systems/plants/shapes/DrakeShapes.h"
#include "drake/util/drakeGeometryUtil.h"

#define BASIS_VECTOR_HALF_COUNT \
  2  // number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;

/**
 * Maintains a vector of RigidBody objects that are arranged into a kinematic
 * tree via DrakeJoint objects. It provides various utility methods for
 * computing kinematic and dynamics properties of the RigidBodyTree.
 *
 * The internal organization of a RigidBodyTree's generalized state vector is as
 * follows:
 *
 * <pre>
 * [model instance 1's generalized coordinate vector]
 * [model instance 2's generalized coordinate vector]
 * ...
 * [model instance 1's generalized velocity vector]
 * [model instance 2's generalized velocity vector]
 * ...
 * </pre>
 *
 * Each RigidBody maintains for its joint that connects to its parent the
 * indices of the joint's generalized coordinate vector and generalized velocity
 * vector in the RigidBodyTree's generalized state vector.
 *
 * The starting index of the joint's generalized coordinate vector in the
 * RigidBodyTree's generalized state vector can be obtained by executing
 * RigidBody::get_position_start_index().
 *
 * The starting index of the joint's generalized velocity vector in the
 * RigidBodyTree's generalized state vector can be computed as
 * follows: RigidBodyTree::get_num_positions() +
 * RigidBody::get_velocity_start_index().
 *
 * Note that the velocity index starts at the beginning of the velocity state
 * variables and not at the beginning of the full state of the RigidBodyTree.
 * This is why the total number of positions needs to be added to the velocity
 * index to get its index in the RigidBodyTree's full state vector.
 */
class DRAKE_EXPORT RigidBodyTree {
 public:
  /**
   * Defines the name of the rigid body within a rigid body tree that represents
   * the world.
   */
  static const char* const kWorldName;

  /**
   * Defines the index of the body that represents the world within a
   * RigidBodyTree.
   */
  static const int kWorldBodyIndex;

  RigidBodyTree(
      const std::string& urdf_filename,
      const drake::systems::plants::joints::FloatingBaseType
          floating_base_type = drake::systems::plants::joints::kRollPitchYaw);
  RigidBodyTree(void);
  virtual ~RigidBodyTree(void);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromURDFString.")
#endif
  void addRobotFromURDFString(
      const std::string& xml_string, const std::string& root_dir = ".",
      const drake::systems::plants::joints::FloatingBaseType
          floating_base_type = drake::systems::plants::joints::kRollPitchYaw,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromURDFString.")
#endif
  void addRobotFromURDFString(
      const std::string& xml_string,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::map<std::string, std::string>& ros_package_map,
      const std::string& root_dir = ".",
      const drake::systems::plants::joints::FloatingBaseType
          floating_base_type = drake::systems::plants::joints::kRollPitchYaw,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromURDF.")
#endif
  void addRobotFromURDF(
      const std::string& urdf_filename,
      const drake::systems::plants::joints::FloatingBaseType
          floating_base_type = drake::systems::plants::joints::kRollPitchYaw,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstanceFromURDF.")
#endif
  void addRobotFromURDF(
      const std::string& urdf_filename,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::map<std::string, std::string>& ros_package_map,
      const drake::systems::plants::joints::FloatingBaseType
          floating_base_type = drake::systems::plants::joints::kRollPitchYaw,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use AddModelInstancesFromSdfFile.")
#endif
  void addRobotFromSDF(const std::string& sdf_filename,
                       const drake::systems::plants::joints::FloatingBaseType
                           floating_base_type =
                               drake::systems::plants::joints::kQuaternion,
                       std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  /**
   * Adds a new model instance to this `RigidBodyTree`. The model instance is
   * identified by a unique model instance ID, which is the return value of
   * this method.
   */
  // This method is not thread safe!
  int add_model_instance();

  // This method is not thread safe.
  int get_next_clique_id() { return next_available_clique_++; }

  /**
   * Returns the number of model instances in the tree.
   */
  int get_num_model_instances() const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_model_instances().")
#endif
  int get_number_of_model_instances() const;

  void addFrame(std::shared_ptr<RigidBodyFrame> frame);

  std::map<std::string, int> computePositionNameToIndexMap() const;

  void surfaceTangents(
      Eigen::Map<Eigen::Matrix3Xd> const& normals,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::default_random_engine& generator) const;

  /**
   * Returns the name of the position state at index @p position_num
   * within this `RigidBodyTree`'s state vector.
   *
   * @param[in] position_num An index value between zero and
   * number_of_positions().
   *
   * @return The name of the position value at index @p position_num.
   */
  std::string get_position_name(int position_num) const;

  /**
   * Returns the name of the velocity state at index @p velocity_num
   * within this `RigidBodyTree`'s state vector.
   *
   * @param[in] velocity_num An index value between number_of_positions() and
   * number_of_veocities().
   *
   * @return The name of the velocity value at index @p velocity_num.
   */
  std::string get_velocity_name(int velocity_num) const;

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_position_name.")
#endif
  std::string getPositionName(int position_num) const;

// TODO(liang.fok) Remove this deprecated method prior to release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_velocity_name.")
#endif
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
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void doKinematics(KinematicsCache<Scalar>& cache,
                    bool compute_JdotV = false) const;

  /**
   * Returns true if @p body is part of a model instance whose ID is in
   * @p model_instance_id_set.
   */
  bool is_part_of_model_instances(const RigidBody& body,
      const std::set<int>& model_instance_id_set) const;

  /**
   * Computes the total combined mass of a set of model instances.
   *
   * @param[in] model_instance_id_set A set of model instance ID values
   * corresponding to the model instances whose masses should be included in the
   * returned value.
   *
   * @returns The total combined mass of the model instances in
   * @p model_instance_id_set.
   */
  double getMass(const std::set<int>& model_instance_id_set =
      default_model_instance_id_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1> centerOfMass(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> worldMomentumMatrix(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> worldMomentumMatrixDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> centroidalMomentumMatrix(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> centroidalMomentumMatrixDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, Eigen::Dynamic>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  centerOfMassJacobian(KinematicsCache<Scalar>& cache,
                       const std::set<int>& model_instance_id_set =
                           default_model_instance_id_set,
                       bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1>
  centerOfMassJacobianDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          default_model_instance_id_set) const;

  template <typename DerivedA, typename DerivedB, typename DerivedC>
  void jointLimitConstraints(
      Eigen::MatrixBase<DerivedA> const& q,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::MatrixBase<DerivedB>& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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

  /**
   * Finds the ancestors of a body. The ancestors include the body's parent,
   * the parent's parent, etc., all the way to the root of this RigidBodyTree,
   * which represents the world.
   *
   * @param[in] body_index The index of the body in this RigidBodyTree for which
   * the ancestors of the body are found. Ancestors are returned in a vector of
   * body indexes.
   *
   * @return A vector of body indexes of the ancestor bodies of the body with
   * index @p body_index.
   */
  std::vector<int> FindAncestorBodies(int body_index) const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use RigidBodyTree::FindAncestorBodies().")
#endif
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache) const;

#ifndef SWIG
  /// Convenience alias for rigid body to external wrench map, for use with
  /// inverseDynamics and dynamicsBiasTerm.
  template <typename Scalar>
  using BodyToWrenchMap = drake::eigen_aligned_std_unordered_map<
    RigidBody const*, drake::WrenchVector<Scalar>>;
#endif

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
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const drake::eigen_aligned_std_unordered_map<
          RigidBody const*, drake::WrenchVector<Scalar>>& external_wrenches,
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
  * \param external_wrenches external wrenches exerted upon bodies
  * (\f$ f_\text{ext} \f$). Expressed in body frame.
  * \param vd \f$ \dot{v} \f$
  * \param include_velocity_terms whether to include velocity-dependent terms in
  *\f$ C(q, v, f_\text{ext}) \f$. Setting \a include_velocity_terms to false is
  *Equivalent to setting \f$ v = 0 \f$
  * \return \f$ H(q) \dot{v} + C(q, v, f_\text{ext}) \f$
  */
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inverseDynamics(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const drake::eigen_aligned_std_unordered_map<
          RigidBody const*, drake::WrenchVector<Scalar>>& external_wrenches,
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
  Eigen::Matrix<Scalar, drake::kQuaternionSize, Eigen::Dynamic>
  relativeQuaternionJacobian(const KinematicsCache<Scalar>& cache,
                             int from_body_or_frame_ind,
                             int to_body_or_frame_ind,
                             bool in_terms_of_qdot) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kRpySize, Eigen::Dynamic>
  relativeRollPitchYawJacobian(const KinematicsCache<Scalar>& cache,
                               int from_body_or_frame_ind,
                               int to_body_or_frame_ind,
                               bool in_terms_of_qdot) const;

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
  Eigen::Transform<Scalar, drake::kSpaceDimension, Eigen::Isometry>
  relativeTransform(const KinematicsCache<Scalar>& cache, int base_or_frame_ind,
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
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& J) const;

  DrakeCollision::ElementId addCollisionElement(
      const DrakeCollision::Element& element,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      RigidBody& body,
      const std::string& group_name);

  template <class UnaryPredicate>
  void removeCollisionGroupsIf(UnaryPredicate test) {
    for (const auto& body_ptr : bodies) {
      std::vector<std::string> names_of_groups_to_delete;
      for (const auto& group : body_ptr->get_group_to_collision_ids_map()) {
        const std::string& group_name = group.first;
        if (test(group_name)) {
          auto& ids = body_ptr->get_mutable_collision_element_ids();
          for (const auto& id : group.second) {
            ids.erase(std::find(ids.begin(), ids.end(), id));
            collision_model_->removeElement(id);
          }
          names_of_groups_to_delete.push_back(group_name);
        }
      }
      for (const auto& group_name : names_of_groups_to_delete) {
        body_ptr->get_mutable_group_to_collision_ids_map().erase(group_name);
      }
    }
  }

  void updateCollisionElements(
      const RigidBody& body,
      const Eigen::Transform<double, 3, Eigen::Isometry>& transform_to_world);

  void updateStaticCollisionElements();

  void updateDynamicCollisionElements(const KinematicsCache<double>& kin_cache);

  /**
   * Gets the contact points defined by a body's collision elements.
   *
   * @param[in] body The body who's collision elements are searched.
   *
   * @param[out] terrain_points Contact points are added to this matrix.
   *
   * @param[in] group_name If a group name was given, use it to look up the
   * subset of collision elements that belong to that collision group.
   * Otherwise, uses the full set of collision elements that belong to the body.
   *
   * @throws std::runtime_error if an invalid group name is given.
   */
  void getTerrainContactPoints(const RigidBody& body,
                               Eigen::Matrix3Xd* terrain_points,
                               const std::string& group_name = "") const;

  bool collisionRaycast(const KinematicsCache<double>& cache,
                        const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                        Eigen::VectorXd& distances,
                        bool use_margins = false);
  bool collisionRaycast(const KinematicsCache<double>& cache,
                        const Eigen::Matrix3Xd& origins,
                        const Eigen::Matrix3Xd& ray_endpoints,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                        Eigen::VectorXd& distances, Eigen::Matrix3Xd& normals,
                        bool use_margins = false);

  /** collisionDetectFromPoints
   * @brief Computes the (signed) distance from the given points to the nearest
   * body in the RigidBodyTree.
   */
  void collisionDetectFromPoints(
      const KinematicsCache<double>& cache,
      const Eigen::Matrix3Xd& points,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi, Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& x, Eigen::Matrix3Xd& body_x,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& body_idx, bool use_margins);

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::vector<DrakeCollision::ElementId>& ids_to_check,
      bool use_margins);

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::vector<int>& bodies_idx,
      const std::set<std::string>& active_element_groups,
      bool use_margins = true);

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::vector<int>& bodies_idx,
      bool use_margins = true);

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      const std::set<std::string>& active_element_groups,
      bool use_margins = true);

  bool collisionDetect(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      bool use_margins = true);

  bool allCollisions(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyB_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& ptsA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& ptsB,
      bool use_margins = true);

  void potentialCollisions(
      const KinematicsCache<double>& cache,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::VectorXd& phi,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& normal,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xA,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix3Xd& xB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<int>& bodyA_idx,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
   * Returns a vector of pointers to all rigid bodies in this tree that belong
   * to a particular model instance.
   *
   * @param[in] model_instance_id The ID of the model instance whose rigid
   * bodies are being searched for.
   *
   * @return A vector of pointers to every rigid body belonging to the sepcified
   * model instance.
   */
  std::vector<const RigidBody*>
  FindModelInstanceBodies(int model_instance_id) const;

/**
 * This is a deprecated version of `FindBody(...)`. Please use `FindBody(...)`
 * instead.
 */
#ifndef SWIG
  DRAKE_DEPRECATED("Please use RigidBodyTree::FindBody().")
#endif
  RigidBody* findLink(const std::string& link_name,
                      const std::string& model_name = "",
                      int model_id = -1) const;

  /**
   * Obtains a vector of indexes of the bodies that are directly attached to the
   * world via any type of joint.  This method has a time complexity of `O(N)`
   * where `N` is the number of bodies in the tree, which can be determined by
   * calling RigidBodyTree::get_num_bodies().
   */
  std::vector<int> FindBaseBodies(int model_instance_id = -1) const;

  /**
   * Obtains the index of a rigid body within this rigid body tree. The rigid
   * body tree maintains a vector of pointers to all rigid bodies that are part
   * of the rigid body tree. The index of a rigid body is the index within this
   * vector at which a pointer to the rigid body is stored.
   *
   * @param[in] body_name The body whose index we want to find. It should
   * be unique within the searched models, otherwise an exception will be
   * thrown.
   *
   * @param[in] model_instance_id The ID of the model instance. This parameter
   * is optional. If supplied, only the model instance with the specified
   * instance ID is searched; otherwise, all model instances are searched.
   *
   * @return The index of the specified rigid body.
   *
   * @throws std::logic_error if no rigid body with the specified \p body_name
   * and \p model_id was found or if multiple matching rigid bodies were found.
   */
  int FindBodyIndex(const std::string& body_name, int model_instance_id = -1)
      const;

  /**
   * Returns a vector of indexes of bodies that are the children of the body at
   * index @p parent_body_index. The resulting list can be further filtered to
   * be bodies that belong to model instance ID @p model_instance_id. This
   * method has a time complexity of `O(N)` where `N` is the number of bodies
   * in the tree, which can be determined by calling
   * RigidBodyTree::get_num_bodies().
   */
  std::vector<int> FindChildrenOfBody(int parent_body_index,
      int model_instance_id = -1) const;

  /**
   * This is a deprecated version of `FindBodyIndex(...)`. Please use
   * `FindBodyIndex(...)` instead.
   */
#ifndef SWIG
  DRAKE_DEPRECATED("Please use RigidBodyTree::FindBodyIndex().")
#endif
  int findLinkId(const std::string& link_name, int model_id = -1) const;

  /**
   * Obtains a pointer to the rigid body whose parent joint is named
   * @p joint_name and is part of a model instance with ID @p model_instance_id.
   *
   * @param[in] joint_name The name of the parent joint of the rigid body to
   * find.
   *
   * @param[in] model_instance_id The ID of the model instance that owns the
   * rigid body to find. This parameter is optional. If supplied, the set of
   * rigid bodies to search through is restricted to those that belong to the
   * speified model instance. Otherwise, all rigid bodies in this tree are
   * searched.
   *
   * @return A pointer to the rigid body whose parent joint is named
   * @p joint_name joint and, if @p model_instance_id is specified, is part of
   * the specified model instance.
   *
   * @throws std::runtime_error If either no rigid body is found or multiple
   * matching rigid bodies are found.
   */
  RigidBody* FindChildBodyOfJoint(const std::string& joint_name,
      int model_instance_id = -1) const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use FindChildBodyOfJoint().")
#endif
  RigidBody* findJoint(const std::string& joint_name, int model_id = -1) const;

  /**
   * Returns the index within the vector of rigid bodies of the rigid body whose
   * parent joint is named @p joint_name and is part of a model instance with ID
   * @p model_instance_id.
   *
   * @param[in] joint_name The name of the parent joint of the rigid body whose
   * index is being searched for.
   *
   * @param[in] model_instance_id The ID of the model instance that owns the
   * rigid body to find. This parameter is optional. If supplied, the set of
   * rigid bodies to search through is restricted to those that belong to the
   * speified model instance. Otherwise, all rigid bodies in this tree are
   * searched.
   *
   * @return The index of the rigid body whose parent joint is named
   * @p joint_name and, if @p model_instance_id is specified, is part of
   * the specified model instance.
   *
   * @throws std::runtime_error If either no rigid body is found or multiple
   * matching rigid bodies are found.
   */
  int FindIndexOfChildBodyOfJoint(const std::string& joint_name,
      int model_instance_id = -1) const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use FindIndexOfChildBodyOfJoint().")
#endif
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

  /**
   * Returns the body at index @p body_index. Parameter @p body_index must be
   * between zero and the number of bodies in this tree, which can be determined
   * by calling RigidBodyTree::get_num_bodies(). Note that the body at
   * index 0 represents the world.
   */
  const RigidBody& get_body(int body_index) const;

  /**
   * Returns the number of bodies in this tree. This includes the one body that
   * represents the world.
   */
  int get_num_bodies() const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_bodies().")
#endif
  int get_number_of_bodies() const;

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
      int ncols_joint = in_terms_of_qdot ? body.getJoint().get_num_positions()
                                         : body.getJoint().get_num_velocities();
      int col_start =
          in_terms_of_qdot ? body.get_position_start_index() :
              body.get_velocity_start_index();
      full.middleCols(col_start, ncols_joint) =
          compact.middleCols(compact_col_start, ncols_joint);
      compact_col_start += ncols_joint;
    }
    return full;
  }

  /**
   * A toString method for this class.
   */
  friend DRAKE_EXPORT std::ostream& operator<<(std::ostream&,
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
   * @return A bare, unowned pointer to the @p body.
   */
  RigidBody* add_rigid_body(std::unique_ptr<RigidBody> body);

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
  int get_num_positions() const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_positions().")
#endif
  int number_of_positions() const;

  /**
   * An accessor to the number of velocity states outputted by this rigid body
   * system.
   */
  int get_num_velocities() const;


#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_velocities().")
#endif
  int number_of_velocities() const;

 public:
  static const std::set<int> default_model_instance_id_set;

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
  // The number of generalized position states in this rigid body tree.
  int num_positions_{};

  // The number of generalized velocity states in this rigid body tree.
  int num_velocities_{};

  // The number of model instances in this rigid body tree.
  int num_model_instances_{};

  // Helper functions for contactConstraints.
  template <typename Scalar>
  void accumulateContactJacobian(
      const KinematicsCache<Scalar>& cache, const int bodyInd,
      Eigen::Matrix3Xd const& bodyPoints, std::vector<size_t> const& cindA,
      std::vector<size_t> const& cindB,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& J) const;

  template <typename Scalar>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void updateCompositeRigidBodyInertias(KinematicsCache<Scalar>& cache) const;

  // Reorder body list to ensure parents are before children in the list
  // RigidBodyTree::bodies.
  //
  // See RigidBodyTree::compile
  void SortTree();

  // Defines a number of collision cliques to be used by DrakeCollision::Model.
  // Collision cliques are defined so that:
  // - There is one clique per RigidBody: and so CollisionElement's attached to
  // a RigidBody do not collide.
  // - There is one clique per pair of RigidBodies that are not meant to
  // collide. These are determined according to the policy provided by
  // RigidBody::CanCollideWith.
  //
  // Collision cliques provide a simple mechanism to omit pairs of collision
  // elements from collision tests. The collision element pair (A, B) will not
  // be tested for collision if A and B belong to the same clique.
  // This particular method implements a default heuristics to create cliques
  // for a RigidBodyTree which are in accordance to the policy implemented by
  // RigidBody::CanCollideWith().
  //
  // @see RigidBody::CanCollideWith.
  void CreateCollisionCliques();

  // collision_model maintains a collection of the collision geometry in the
  // RBM for use in collision detection of different kinds. Small margins are
  // applied to all collision geometry when that geometry is added, to improve
  // the numerical stability of contact gradients taken using the model.
  std::unique_ptr<DrakeCollision::Model> collision_model_;

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  // The following was required for building w/ DRAKE_EXPORT on windows (due
  // to the unique_ptrs).  See
  // http://stackoverflow.com/questions/8716824/cannot-access-private-member-error-only-when-class-has-export-linkage
 private:
  RigidBodyTree(const RigidBodyTree&);
  RigidBodyTree& operator=(const RigidBodyTree&) { return *this; }

  std::set<std::string> already_printed_warnings;
  bool initialized_{false};

  int next_available_clique_ = 0;
};

typedef RigidBodyTree RigidBodyTreed;
