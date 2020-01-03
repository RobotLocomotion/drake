#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/LU>

#include "drake/common/constants.h"
#include "drake/common/eigen_stl_types.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/collision/collision_filter.h"
#include "drake/multibody/collision/drake_collision.h"
#include "drake/multibody/collision/element.h"
#include "drake/multibody/force_torque_measurement.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/kinematic_path.h"
#include "drake/multibody/kinematics_cache-inl.h"
#include "drake/multibody/pose_map.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_actuator.h"
#include "drake/multibody/rigid_body_distance_constraint.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_loop.h"
#include "drake/multibody/shapes/drake_shapes.h"

#define BASIS_VECTOR_HALF_COUNT \
  2  // number of basis vectors over 2 (i.e. 4 basis vectors in this case)

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;

/**
 * Defines RigidBodyTree constants. A separate struct is necessary to avoid
 * having these constants being templated on `<T>`. For more details about the
 * problem with having these templated on `<T>`, see #4169.
 */
struct RigidBodyTreeConstants {
  /**
   * Defines the name of the RigidBody within a RigidBodyTree that represents
   * the world.
   */
  static const char* const kWorldName;

  /**
   * Defines the index of the RigidBody within a RigidBodyTree that represents
   * the world.
   */
  static const int kWorldBodyIndex;

  /**
   * The ID of the first non-world model instance in the tree.
   */
  static const int kFirstNonWorldModelInstanceId;

  /**
   * Defines the default model instance ID set. This is a set containing the
   * model instance ID of the first model instance that is added to the tree.
   */
  static const std::set<int> default_model_instance_id_set;
};

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
 *
 * @tparam T The scalar type. Must be a valid Eigen scalar.
 */
template <typename T>
class RigidBodyTree {
 public:
  /// A constructor that initializes the gravity vector to be [0, 0, -9.81] and
  /// a single RigidBody named "world". This RigidBody can be accessed by
  /// calling RigidBodyTree::world().
  RigidBodyTree();

  virtual ~RigidBodyTree();

  /**
   * Returns a deep clone of this RigidBodyTree<double>. Currently, everything
   * *except* for collision and visual elements are cloned.  Only supported for
   * T = double.
   */
  std::unique_ptr<RigidBodyTree<double>> Clone() const {
    // N.B. We specialize this method below for T = double.  This method body
    // is the default implementation for all _other_ values of T.
    throw std::logic_error("RigidBodyTree::Clone only supports T = double");
  }

  /**
   * When @p val is true, diagnostics in compile() will be printed with
   * drake::log()->info(). When false, drake::log()->debug() will be used
   * instead.
   */
  void print_joint_welding_diagnostics(bool wants_to_print) {
    print_weld_diasnostics_ = wants_to_print;
  }

  /**
   * Adds a new model instance to this `RigidBodyTree`. The model instance is
   * identified by a unique model instance ID, which is the return value of
   * this method.
   */
  // This method is not thread safe!
  int add_model_instance();

  // This method is not thread safe.
  int get_next_clique_id() { return next_available_clique_++; }

  // TODO(liang.fok) Update this method implementation once the world
  // is assigned its own model instance ID (#3088). It should return
  // num_model_instances_ - 1.
  /**
   * Returns the number of model instances in the tree, not including the world.
   */
  int get_num_model_instances() const { return num_model_instances_; }

  /**
   * Adds a frame.
   *
   * @param frame Frame to be added.
   * @pre Neither a body nor frame with the same identifying information (name
   * and model id / name) should already exist in the tree.
   * @throws std::runtime_error if preconditions are not met.
   */
  // TODO(eric.cousineau): Rename to `AddFrame`.
  void addFrame(std::shared_ptr<RigidBodyFrame<T>> frame);

  /**
   * Returns a map from DOF position name to DOF index within the output vector
   * of this RigidBodyTree.
   *
   * <b>WARNING:</b> There is a known bug in this method, see: #4697.
   */
  std::map<std::string, int> computePositionNameToIndexMap() const;

  void surfaceTangents(
      Eigen::Map<Eigen::Matrix3Xd> const& normals,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      std::vector<Eigen::Map<Eigen::Matrix3Xd>>& tangents) const;

  /**
   * Applies the given transform to the given @p body's collision elements,
   * displacing them from their current configurations.  These new poses
   * will be considered the elements' pose with respect to the body.
   *
   * This is important to the parsing code to maintain a Drake RigidBodyTree
   * invariant.  RigidBody instances do not maintain their own pose relative
   * to their in-board joint.  The joint's space is considered to be the body's
   * space.  So, if a URDF or SDF file defines the body with a non-identity pose
   * relative to the parent, the parser uses this to move the collision elements
   * relative to the effective body frame -- that of the parent joint.
   *
   * @param body The body whose collision elements will be moved.
   * @param displace_transform The transform to apply to each collision element.
   * @param true if the collision element was successfully updated.
   * @returns true if the @body's elements were successfully transformed.
   */
  bool transformCollisionFrame(
      RigidBody<T>* body,
      const Eigen::Isometry3d& displace_transform);

  void compile();  // call me after the model is loaded

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

  std::string getStateName(int state_num) const;

  void drawKinematicTree(std::string graphviz_dotfile_filename) const;

  /// Checks whether @p cache is valid for use with this RigidBodyTree. Throws
  /// a std::runtime_error exception if it is not valid.
  template <typename Scalar>
  void CheckCacheValidity(const KinematicsCache<Scalar>& cache) const;

  /// Creates a KinematicsCache to perform computations with this RigidBodyTree.
  /// The returned KinematicsCache is consistently templated on the scalar type
  /// for this RigidBodyTree instance.
  /// Aborts if this RigidBodyTree was not previously initialized with a call
  /// to RigidBodyTree::compile().
  /// @returns The created KinematicsCache.
  KinematicsCache<T> CreateKinematicsCache() const;

  /// A helper template method used to create a KinematicsCache templated on
  /// `CacheT` from a RigidBodyTree templated on `T`, with `CacheT` and `T`
  /// not necessarily the same scalar type.
  /// This method is particularly useful in mex files where only a reference
  /// to a `RigidBodyTree<double>` is available to create kinematics caches
  /// on different scalar types.
  /// Aborts if this RigidBodyTree was not previously initialized with a call
  /// to RigidBodyTree::compile().
  ///
  /// Users should not call this method but instead create KinematicsCache
  /// objects with RigidBodyTree:CreateKinematicsCache().
  ///
  /// @tparam CacheT The scalar type for the returned KinematicsCache.
  /// @returns A KinematicsCache templated on `CacheT` that can be used for
  /// computations on this RigidBodyTree with methods instantiated on `CacheT`.
  // TODO(amcastro-tri): Remove this method once older pieces of code such as
  // createKinematicsCacheAutoDiffmex.cpp are updated to use a RigidBodyTree to
  // manage cache creation.
  template <typename CacheT>
  KinematicsCache<CacheT> CreateKinematicsCacheWithType() const;

  /// Creates a KinematicsCache given a reference to a vector of rigid bodies
  /// contained within a RigidBodyTree.
  /// This method is static since all the information to create the
  /// corresponding KinematicsCache resides in the input parameter vector
  /// `bodies`.
  ///
  /// @param bodies A vector of unique pointers to the rigid bodies of a given
  /// RigidBodyTree for which a KinematicsCache needs to be created.
  /// @returns The created KinematicsCache.
  //
  // TODO(amcastro-tri): Remove this method once older pieces of code such as
  // KinematicsCacheHelper are updated to use a RigidBodyTree to manage cache
  // creation.
  static KinematicsCache<T>
  CreateKinematicsCacheFromBodiesVector(
      const std::vector<std::unique_ptr<RigidBody<T>>>& bodies);

  /// Initializes a `KinematicsCache` with the given configuration @p q,
  /// computes the kinematics, and returns the cache.
  ///
  /// This method is explicitly instantiated in rigid_body_tree.cc for a
  /// small set of supported `DerivedQ`.
  template <typename DerivedQ>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(
      const Eigen::MatrixBase<DerivedQ>& q) const;

  /// Initializes a `KinematicsCache` with the given configuration @p q
  /// and velocity @p v, computes the kinematics, and returns the cache.
  ///
  /// This method is explicitly instantiated in rigid_body_tree.cc for a
  /// small set of supported `DerivedQ` and `DerivedV`.
  template <typename DerivedQ, typename DerivedV>
  KinematicsCache<typename DerivedQ::Scalar> doKinematics(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v, bool compute_JdotV = true) const;

  /// Computes the kinematics on the given @p cache.
  ///
  /// This method is explicitly instantiated in rigid_body_tree.cc for a
  /// small set of supported Scalar types.
  template <typename Scalar>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void doKinematics(KinematicsCache<Scalar>& cache,
                    bool compute_JdotV = false) const;

  /**
   * Returns true if @p body is part of a model instance whose ID is in
   * @p model_instance_id_set.
   */
  bool is_part_of_model_instances(
      const RigidBody<T>& body,
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
      RigidBodyTreeConstants::default_model_instance_id_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1> centerOfMass(
      const KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set) const;

  /**
   * Computes the summed spatial inertia in the world frame of all bodies that
   * belong to model instances in @p model_instance_id_set.
   * @param cache Reference to the KinematicsCache.
   * @param model_instance_id_set A set of model instance ID values
   * corresponding to the model instances whose spatial inertia should be
   * included in the returned value.
   * @return The summed spatial inertia.
   */
  drake::Matrix6<T> LumpedSpatialInertiaInWorldFrame(
      const KinematicsCache<T>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set) const;

  /**
   * Computes the summed spatial inertia in the world frame of all the bodies
   * in @p bodies_of_interest.
   * @param cache Reference to the KinematicsCache.
   * @param bodies_of_interest Vector of bodies, whose spatial inertia will be
   * summed and returned.
   * @return The summed spatial inertia.
   */
  drake::Matrix6<T> LumpedSpatialInertiaInWorldFrame(
      const KinematicsCache<T>& cache,
      const std::vector<const RigidBody<T>*>& bodies_of_interest) const;

  /// Computes the pose `X_WB` of @p body's frame B in the world frame W.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @retval X_WB
  drake::Isometry3<T> CalcBodyPoseInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body) const {
    return CalcFramePoseInWorldFrame(
        cache, body, drake::Isometry3<T>::Identity());
  }

  /// Computes the pose `X_WF` of @p frame_F in the world frame W. @p frame_F
  /// does not necessarily need to be owned by this RigidBodyTree. However,
  /// the RigidBody to which @p frame_F attaches to has to be owned by this
  /// RigidBodyTree.
  /// @param cache Reference to the KinematicsCache.
  /// @param frame_F Reference to the RigidBodyFrame.
  /// @retval X_WF
  drake::Isometry3<T> CalcFramePoseInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBodyFrame<T>& frame_F) const {
    return CalcFramePoseInWorldFrame(cache, frame_F.get_rigid_body(),
        frame_F.get_transform_to_body().template cast<T>());
  }

  /// Computes the pose `X_WF` of the rigid body frame F in the world frame W.
  /// Frame F is rigidly attached to @p body.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @param X_BF The pose of frame F in body frame B.
  /// @retval X_WF
  drake::Isometry3<T> CalcFramePoseInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF) const;

  /// Computes the spatial velocity `V_WB` of @p body's frame B measured and
  /// expressed in the world frame W.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @retval V_WB
  drake::Vector6<T> CalcBodySpatialVelocityInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body) const;

  /// Computes the spatial velocity `V_WF` of RigidBodyFrame @p frame_F measured
  /// and expressed in the world frame W. @p frame_F does not necessarily need
  /// to be owned by this RigidBodyTree. However, the RigidBody to which
  /// @p frame_F attaches to has to be owned by this RigidBodyTree.
  /// @param cache Reference to the KinematicsCache.
  /// @param frame_F Reference to the RigidBodyFrame.
  /// @retval V_WF
  drake::Vector6<T> CalcFrameSpatialVelocityInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBodyFrame<T>& frame_F) const {
    return CalcFrameSpatialVelocityInWorldFrame(
        cache, frame_F.get_rigid_body(),
        frame_F.get_transform_to_body().template cast<T>());
  }

  /// Computes the spatial velocity `V_WF` of the frame F measured and expressed
  /// in the world frame W. Frame F is rigidly attached to @p body.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @param X_BF The pose of frame F in body frame B.
  /// @retval V_WF
  drake::Vector6<T> CalcFrameSpatialVelocityInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF) const;

  /// Computes the Jacobian `J_WF` of the spatial velocity `V_WF` of frame F
  /// measured and expressed in the world frame W such that `V_WF = J_WF * v`,
  /// where `v` is the generalized velocity. Frame F is rigidly attached to
  /// @p body.
  /// @param cache Reference to the KinematicsCache.
  /// @param B Reference to the RigidBody.
  /// @param X_BF The pose of frame F in body frame B.
  /// @param in_terms_of_qdot `true` for `J_WF` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WF = J_WF * qdot`. `false` for `J_WF` computed with respect to `v`.
  /// @retval J_WF
  drake::Matrix6X<T> CalcFrameSpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF,
      bool in_terms_of_qdot = false) const;

  /// Computes the Jacobian `J_WF` of the spatial velocity `V_WF` of frame F
  /// measured and expressed in the world frame W such that `V_WF = J_WF * v`,
  /// where `v` is the generalized velocity. Frame F is rigidly attached to
  /// @p body. This version does not allocate memory and will assert if `J_WF`
  /// is incorrectly sized.
  /// @param[in] cache Reference to the KinematicsCache.
  /// @param[in] B Reference to the RigidBody.
  /// @param[in] X_BF The pose of frame F in body frame B.
  /// @param[in] in_terms_of_qdot `true` for `J_WF` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WF = J_WF * qdot`. `false` for `J_WF` computed with respect to `v`.
  /// @param[out] J_WF Pointer to the output Jacobian.
  void CalcFrameSpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF, bool in_terms_of_qdot,
      drake::Matrix6X<T>* J_WF) const;

  /// Computes the Jacobian `J_WF` of the spatial velocity `V_WF` of frame F
  /// measured and expressed in the world frame W such that `V_WF = J_WF * v`,
  /// where `v` is the generalized velocity. @p frame_F does not necessarily
  /// need to be owned by this RigidBodyTree. However, the RigidBody to which
  /// @p frame_F attaches to has to be owned by this RigidBodyTree.
  /// @param cache Reference to the KinematicsCache.
  /// @param frame_F Reference to the RigidBodyFrame.
  /// @param in_terms_of_qdot `true` for `J_WF` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WF = J_WF * qdot`. `false` for `J_WF` computed with respect to `v`.
  /// @retval J_WF
  drake::Matrix6X<T> CalcFrameSpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBodyFrame<T>& frame_F,
      bool in_terms_of_qdot = false) const {
    return CalcFrameSpatialVelocityJacobianInWorldFrame(
        cache, frame_F.get_rigid_body(),
        frame_F.get_transform_to_body().template cast<T>(), in_terms_of_qdot);
  }

  /// Computes the Jacobian `J_WF` of the spatial velocity `V_WF` of frame F
  /// measured and expressed in the world frame W such that `V_WF = J_WF * v`,
  /// where `v` is the generalized velocity. @p frame_F does not necessarily
  /// need to be owned by this RigidBodyTree. However, the RigidBody to which
  /// @p frame_F attaches to has to be owned by this RigidBodyTree. This
  /// version does not allocate memory and will assert if `J_WF` is incorrectly
  /// sized.
  /// @param[in] cache Reference to the KinematicsCache.
  /// @param[in] frame_F Reference to the RigidBodyFrame.
  /// @param[in] in_terms_of_qdot `true` for `J_WF` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WF = J_WF * qdot`. `false` for `J_WF` computed with respect to `v`.
  /// @param[out] J_WF Pointer to the output Jacobian.
  void CalcFrameSpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBodyFrame<T>& frame_F,
      bool in_terms_of_qdot, drake::Matrix6X<T>* J_WF) const {
    return CalcFrameSpatialVelocityJacobianInWorldFrame(
        cache, frame_F.get_rigid_body(),
        frame_F.get_transform_to_body().template cast<T>(),
        in_terms_of_qdot, J_WF);
  }

  /// Computes the Jacobian `J_WB` of the spatial velocity `V_WB` of body
  /// frame B measured and expressed in the world frame `W` such that
  /// `V_WB = J_WB * v`, where `v` is the generalized velocity.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @param in_terms_of_qdot `true` for `J_WB` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WB = J_WB * qdot`. `false` for `J_WB` computed with respect to `v`.
  /// @retval J_WB
  drake::Matrix6X<T> CalcBodySpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      bool in_terms_of_qdot = false) const {
    return CalcFrameSpatialVelocityJacobianInWorldFrame(
        cache, body, drake::Isometry3<T>::Identity(), in_terms_of_qdot);
  }

  /// Computes the Jacobian `J_WB` of the spatial velocity `V_WB` of body
  /// frame B measured and expressed in the world frame `W` such that
  /// `V_WB = J_WB * v`, where `v` is the generalized velocity. This version
  /// does not allocate memory and will assert if `J_WB` is incorrectly sized.
  /// @param[in] cache Reference to the KinematicsCache.
  /// @param[in] body Reference to the RigidBody.
  /// @param[in] in_terms_of_qdot `true` for `J_WB` computed with respect to the
  /// time derivative of the generalized position such that
  /// `V_WB = J_WB * qdot`. `false` for `J_WB` computed with respect to `v`.
  /// @param[out] J_WB Pointer to the output Jacobian.
  void CalcBodySpatialVelocityJacobianInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      bool in_terms_of_qdot, drake::Matrix6X<T>* J_WB) const {
    CalcFrameSpatialVelocityJacobianInWorldFrame(
        cache, body, drake::Isometry3<T>::Identity(), in_terms_of_qdot, J_WB);
  }

  /// Computes `Jdot_WF * v`, where `J_WF` is the Jacobian of spatial velocity,
  /// `V_WF`, of frame F measured and expressed in the world frame W, and
  /// `v` is the generalized velocity. Frame F is rigidly attached to @p body.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @param X_BF The pose of frame F in body frame B.
  /// @retval `Jdot_WF * v`
  drake::Vector6<T> CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body,
      const drake::Isometry3<T>& X_BF) const;

  /// Computes `Jdot_WF * v`, where `J_WF` is the Jacobian of spatial velocity
  /// `V_WF` of frame F measured and expressed in the world frame W, and
  /// `v` is the generalized velocity. @p frame_F does not necessarily need to
  /// be owned by this RigidBodyTree. However, the RigidBody to which @p frame_F
  /// attaches to has to be owned by this RigidBodyTree.
  /// @param cache Reference to the KinematicsCache.
  /// @param frame_F Reference to the RigidBodyFrame.
  /// @retval `Jdot_WF * v`
  drake::Vector6<T> CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBodyFrame<T>& frame_F) const {
    return CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
        cache, frame_F.get_rigid_body(),
        frame_F.get_transform_to_body().template cast<T>());
  }

  /// Computes `Jdot_WB * v`, where `J_WB` is the Jacobian of the spatial
  /// velocity `V_WB` of body frame B measured and expressed in the world
  /// frame W, and `v` is the generalized velocity.
  /// @param cache Reference to the KinematicsCache.
  /// @param body Reference to the RigidBody.
  /// @retval `Jdot_WB * v`
  drake::Vector6<T> CalcBodySpatialVelocityJacobianDotTimesVInWorldFrame(
      const KinematicsCache<T>& cache, const RigidBody<T>& body) const {
    return CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
        cache, body, drake::Isometry3<T>::Identity());
  }

  /// Converts a vector of the time derivative of generalized coordinates (qdot)
  /// to generalized velocity (v).
  /// @param cache the kinematics cache, which is assumed to be up-to-date with
  ///        respect to the state
  /// @param qdot a `nq` dimensional vector, where `nq` is the dimension of the
  ///      generalized coordinates.
  /// @returns a `nv` dimensional vector, where `nv` is the dimension of the
  ///      generalized velocities.
  /// @sa transformVelocityToQDot()
  template <typename Derived>
  static drake::VectorX<typename Derived::Scalar>
  transformQDotToVelocity(
      const KinematicsCache<typename Derived::Scalar>& cache,
      const Eigen::MatrixBase<Derived>& qdot);

  /// Converts a vector of generalized velocities (v) to the time
  /// derivative of generalized coordinates (qdot).
  /// @param cache the kinematics cache, which is assumed to be up-to-date with
  ///        respect to the state
  /// @param v a `nv` dimensional vector, where `nv` is the dimension of the
  ///      generalized velocities.
  /// @retval qdot a `nq` dimensional vector, where `nq` is the dimension of the
  ///      generalized coordinates.
  /// @sa transformQDotToVelocity()
  template <typename Derived>
  static drake::VectorX<typename Derived::Scalar>
  transformVelocityToQDot(
      const KinematicsCache<typename Derived::Scalar>& cache,
      const Eigen::MatrixBase<Derived>& v);

  /**
   * Converts a matrix B, which transforms generalized velocities (v) to an
   * output space X, to a matrix A, which transforms the time
   * derivative of generalized coordinates (qdot) to the same output X. For
   * example, B could be a Jacobian matrix that transforms generalized
   * velocities to spatial velocities at the end-effector. Formally, this would
   * be the matrix of partial derivatives of end-effector configuration computed
   * with respect to quasi-coordinates (ꝗ). This function would allow
   * transforming that Jacobian so that all partial derivatives would be
   * computed with respect to qdot.
   * @param Av, a `m x nv` sized matrix, where `nv` is the dimension of the
   *      generalized velocities.
   * @retval A a `m x nq` sized matrix, where `nq` is the dimension of the
   *      generalized coordinates.
   * @sa transformQDotMappingToVelocityMapping()
   */
  template <typename Derived>
  static drake::MatrixX<typename Derived::Scalar>
  transformVelocityMappingToQDotMapping(
      const KinematicsCache<typename Derived::Scalar>& cache,
      const Eigen::MatrixBase<Derived>& Av);

  /**
   * Converts a matrix A, which transforms the time derivative of generalized
   * coordinates (qdot) to an output space X, to a matrix B, which transforms
   * generalized velocities (v) to the same space X. For example, A could be a
   * Jacobian matrix that transforms qdot to spatial velocities at the end
   * effector. Formally, this would be the matrix of partial derivatives of
   * end-effector configuration computed with respect to the generalized
   * coordinates (q). This function would allow the user to
   * transform this Jacobian matrix to the more commonly used one: the matrix of
   * partial derivatives of end-effector configuration computed with respect to
   * quasi-coordinates (ꝗ).
   * @param Ap a `m x nq` sized matrix, where `nq` is the dimension of the
   *      generalized coordinates.
   * @retval B, a `m x nv` sized matrix, where `nv` is the dimension of the
   *      generalized velocities.
   * @sa transformVelocityMappingToQDotMapping()
   */
  template <typename Derived>
  static drake::MatrixX<typename Derived::Scalar>
  transformQDotMappingToVelocityMapping(
      const KinematicsCache<typename Derived::Scalar>& cache,
      const Eigen::MatrixBase<Derived>& Ap);

  template <typename Scalar>
  static drake::MatrixX<Scalar> GetVelocityToQDotMapping(
          const KinematicsCache<Scalar>& cache);

  template <typename Scalar>
  static drake::MatrixX<Scalar> GetQDotToVelocityMapping(
          const KinematicsCache<Scalar>& cache);

  template <typename Scalar>
  drake::TwistMatrix<Scalar> worldMomentumMatrix(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> worldMomentumMatrixDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set  =
          RigidBodyTreeConstants::default_model_instance_id_set) const;

  template <typename Scalar>
  drake::TwistMatrix<Scalar> centroidalMomentumMatrix(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  drake::TwistVector<Scalar> centroidalMomentumMatrixDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, Eigen::Dynamic>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  centerOfMassJacobian(KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set,
      bool in_terms_of_qdot = false) const;

  template <typename Scalar>
  Eigen::Matrix<Scalar, drake::kSpaceDimension, 1>
  centerOfMassJacobianDotTimesV(
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      KinematicsCache<Scalar>& cache,
      const std::set<int>& model_instance_id_set =
          RigidBodyTreeConstants::default_model_instance_id_set) const;

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
   * @param[out] ancestor_bodies A vector of body indexes of the ancestor bodies
   * of the body with index @p body_index.
   */
  void FindAncestorBodies(int body_index,
                          std::vector<int>* ancestor_bodies) const;

  /// Identical to the above overload, expect that this function return the
  /// ancestor bodies instead of using an output argument.
  std::vector<int> FindAncestorBodies(int body_index) const;

  /// Find the kinematic path between two bodies or frames. This function will
  /// not allocate memory if `path`, `start_body_ancestors` and
  /// `end_body_ancestors` are preallocated.
  void FindKinematicPath(int start_body_or_frame_idx,
                         int end_body_or_frame_idx,
                         std::vector<int>* start_body_ancestors,
                         std::vector<int>* end_body_ancestors,
                         KinematicPath* path) const;

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

  /// Convenience alias for rigid body to external wrench map, for use with
  /// inverseDynamics and dynamicsBiasTerm.
  using BodyToWrenchMap = drake::eigen_aligned_std_unordered_map<
    RigidBody<double> const*, drake::WrenchVector<T>>;

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
          RigidBody<T> const*, drake::WrenchVector<Scalar>>& external_wrenches,
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
          RigidBody<T> const*, drake::WrenchVector<Scalar>>& external_wrenches,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& vd,
      bool include_velocity_terms = true) const;

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorques(
      Eigen::MatrixBase<DerivedV> const& v) const;


  /// Computes the generalized forces that correspond to joint springs.
  ///
  /// Spring forces are computed joint-by-joint and are a function of position
  /// only (they do not couple between joints)
  template <typename Scalar>
  drake::VectorX<Scalar> CalcGeneralizedSpringForces(
      const drake::VectorX<Scalar>& q) const;

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
    // Relative transformation from frame "from_body_or_frame_ind" to frame
    // "to_body_or_frame_ind".
    auto relative_transform =
        relativeTransform(cache, to_body_or_frame_ind, from_body_or_frame_ind);
    return relative_transform * points.template cast<Scalar>();
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, 4, 1> relativeQuaternion(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind) const {
    const drake::Matrix3<Scalar> R = relativeTransform(cache,
                         to_body_or_frame_ind, from_body_or_frame_ind).linear();
    return drake::math::RotationMatrix<Scalar>::ToQuaternionAsVector4(R);
  }

  template <typename Scalar>
  Eigen::Matrix<Scalar, 3, 1> relativeRollPitchYaw(
      const KinematicsCache<Scalar>& cache, int from_body_or_frame_ind,
      int to_body_or_frame_ind) const {
    const drake::Isometry3<Scalar> pose = relativeTransform(cache,
                                  to_body_or_frame_ind, from_body_or_frame_ind);
    const drake::math::RotationMatrix<Scalar> R(pose.linear());
    const drake::math::RollPitchYaw<Scalar> rpy(R);
    return rpy.vector();
  }

  template <typename Scalar, typename DerivedPoints>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> transformPointsJacobian(
      const KinematicsCache<Scalar>& cache,
      const Eigen::MatrixBase<DerivedPoints>& points,
      int from_body_or_frame_ind, int to_body_or_frame_ind,
      bool in_terms_of_qdot) const {
    using PointScalar = typename std::conditional<
      std::is_same<typename DerivedPoints::Scalar, double>::value,
      double, drake::AutoDiffXd>::type;
    return DoTransformPointsJacobian<Scalar, PointScalar>(
        cache,
        points.template cast<PointScalar>().eval(),
        from_body_or_frame_ind, to_body_or_frame_ind, in_terms_of_qdot);
  }

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
      int from_body_or_frame_ind, int to_body_or_frame_ind) const {
    return DoTransformPointsJacobianDotTimesV<Scalar>(
        cache, points, from_body_or_frame_ind, to_body_or_frame_ind);
  }

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

 private:
  template <typename Scalar>
  void GeometricJacobian(
      const KinematicsCache<Scalar>& cache, int base_body_or_frame_ind,
      int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind,
      bool in_terms_of_qdot, std::vector<int>* v_indices,
      drake::Matrix6X<Scalar>* J) const;

 public:
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

  /**
   * Computes the Jacobian for many points in the format currently used by
   * MATLAB.  (possibly should be scheduled for deletion, taking
   * accumulateContactJacobians() with it)
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

  /**
   * Adds a new collision element to the tree.  The input @p element will be
   * copied and that copy will be stored in the tree, associated with the
   * given @p body.  This association is pending.  It is necessary to call
   * compile() in order for the element to be fully integrated into the
   * RigidBodyTree.
   * @param element the element to add.
   * @param body the body to associate the element with.
   * @param group_name a group name to tag the associated element with.
   */
  void addCollisionElement(
      const drake::multibody::collision::Element& element,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      RigidBody<T>& body, const std::string& group_name);

  /// Retrieve a `const` pointer to an element of the collision model.
  ///
  /// @note The use of Find (instead of get) and the use of CamelCase both
  /// imply a potential runtime cost are carried over from the collision model
  /// accessor method.
  const drake::multibody::collision::Element* FindCollisionElement(
      const drake::multibody::collision::ElementId& id) const {
    return collision_model_->FindElement(id);
  }

  template <class UnaryPredicate>
  void removeCollisionGroupsIf(UnaryPredicate test) {
    for (const auto& body_ptr : bodies_) {
      std::vector<std::string> names_of_groups_to_delete;
      for (const auto& group : body_ptr->get_group_to_collision_ids_map()) {
        const std::string& group_name = group.first;
        if (test(group_name)) {
          names_of_groups_to_delete.push_back(group_name);
          for (auto id : group.second) {
            collision_model_->RemoveElement(id);
          }
        }
      }
      for (const auto& group_name : names_of_groups_to_delete) {
        drake::internal::RigidBodyAttorney::
            RemoveCollisionGroupAndElements(body_ptr.get(), group_name);
      }
    }
  }

  /**
   * Updates the collision elements registered with the collision detection
   * engine.  Note: If U is not a double then the transforms from kinematics
   * cache will be forcefully cast to doubles (discarding any gradient
   * information).  Callers that set @p throw_if_missing_gradient to
   * `false` are responsible for ensuring that future code is secure despite all
   * gradients with respect to the collision engine being arbitrarily set to
   * zero.
   * @see ComputeMaximumDepthCollisionPoints for an example.
   *
   * @throws std::runtime_error based on the criteria of DiscardZeroGradient()
   * only if @p throws_if_missing_gradient is true.
   */
  template <typename U>
  void updateCollisionElements(
      const RigidBody<T>& body,
      const Eigen::Transform<U, 3, Eigen::Isometry>& transform_to_world,
      bool throw_if_missing_gradient = true);

  /**
   * @see updateCollisionElements
   * @throws std::runtime_error based on the criteria of DiscardZeroGradient()
   * only if @p throws_if_missing_gradient is true.
   */
  template <typename U>
  void updateDynamicCollisionElements(const KinematicsCache<U>& kin_cache,
                                      bool throw_if_missing_gradient = true);

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
  void getTerrainContactPoints(const RigidBody<T>& body,
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

  /**
   * Computes the *signed* distance from the given points to the nearest body
   * in the RigidBodyTree.
   *
   * @param[in] cache a KinematicsCache constructed by
   * RigidBodyTree::doKinematics given `q` and `v`.
   * @param[in] points A 3xN matrix of points, in world frame, to which signed
   * distance will be computed.
   * @param[out] phi Resized to N elements and filled with the computed signed
   * distances, or inf if no closest point was found.
   * @param[out] normal Resized to 3xN elements and filled with collision
   * element normals in world frame, at the closest point on the collision
   * geometry to each point in `points`. Undefined where no closest point was
   * found.
   * @param[out] x Resized to 3xN elements and filled with the closest points
   * on the collision geometry to each point in `points`, in world frame.
   * Undefined where no closest point was found.
   * @param[out] body_x Resized to 3xN elements and filled with the closest
   * points on the collision geometry to each point in `points`, in the body
   * frame of the closest body. Undefined where no closest point was found.
   * @param[out] body_idx Resized to N elements and filled with the body idx
   * of the closest body to each point in `points`, or -1 where no closest
   * body was found.
   * @param[in] use_margins Whether to pad each collision body with a narrow
   * (see bullet_model) margin to improve stability of normal estimation at
   * the cost of the accuracy of closest points calculations.
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
      const std::vector<drake::multibody::collision::ElementId>& ids_to_check,
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

  /** Computes the point of closest approach between bodies in the
   RigidBodyTree that are in contact.

   @param cache[in] a KinematicsCache constructed by RigidBodyTree::doKinematics
   given `q` and `v`.

   Collision points are returned as a vector of PointPair's.
   See the documentation for PointPair for details. The collision point on the
   surface of each body is stored in the PointPair structure in the frame of the
   corresponding body.

   @param[in] use_margins If `true` the model uses the representation with
   margins. If `false`, the representation without margins is used instead.

   @throws std::runtime_error based on the criteria of DiscardZeroGradient()
   only if @p throws_if_missing_gradient is true.
   */
  template <typename U>
  std::vector<drake::multibody::collision::PointPair<U>>
  ComputeMaximumDepthCollisionPoints(const KinematicsCache<U>& cache,
                                     bool use_margins = true, bool
                                     throw_if_missing_gradient = true);

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
  RigidBody<T>* FindBody(const std::string& body_name,
                      const std::string& model_name = "",
                      int model_id = -1) const;

  /**
   * Reports the RigidBody that owns the collision element indicated by the id.
   * @param element_id       The id to query.
   * @return A pointer to the owning RigidBody.
   * @throws std::logic_error if no body can be mapped to the element id.
   */
  const RigidBody<double>* FindBody(
      drake::multibody::collision::ElementId element_id) const;

  /**
   * Returns a vector of pointers to all rigid bodies in this tree that belong
   * to a particular model instance.
   *
   * @param[in] model_instance_id The ID of the model instance whose rigid
   * bodies are being searched for.
   *
   * @return A vector of pointers to every rigid body belonging to the specified
   * model instance.
   */
  std::vector<const RigidBody<T>*>
  FindModelInstanceBodies(int model_instance_id) const;

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
  int FindBodyIndex(const std::string& body_name,
                    int model_instance_id = -1) const;

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
   * Obtains a pointer to the rigid body whose parent joint is named
   * @p joint_name and is part of a model instance with ID @p model_instance_id.
   *
   * @param[in] joint_name The name of the parent joint of the rigid body to
   * find.
   *
   * @param[in] model_instance_id The ID of the model instance that owns the
   * rigid body to find. This parameter is optional. If supplied, the set of
   * rigid bodies to search through is restricted to those that belong to the
   * specified model instance. Otherwise, all rigid bodies in this tree are
   * searched.
   *
   * @return A pointer to the rigid body whose parent joint is named
   * @p joint_name joint and, if @p model_instance_id is specified, is part of
   * the specified model instance.
   *
   * @throws std::runtime_error If either no rigid body is found or multiple
   * matching rigid bodies are found.
   */
  RigidBody<T>* FindChildBodyOfJoint(const std::string& joint_name,
                                     int model_instance_id = -1) const;

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
   * specified model instance. Otherwise, all rigid bodies in this tree are
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

  /**
   * Finds a frame of the specified \p frame_name belonging to a model with the
   * specified \p model_id.
   *
   * @param[in] frame_name The name of the frame to find.
   *
   * @param[in] model_id The ID of the model to which the frame belongs. If this
   * value is -1, search all models.
   *
   * @return The frame with the specified name and model instance ID.
   *
   * @throws std::logic_error if either multiple matching frames are found or no
   * matching frame is found.
   */
  std::shared_ptr<RigidBodyFrame<T>> findFrame(const std::string& frame_name,
                                            int model_id = -1) const;

  /**
   * Returns the body at index @p body_index. Parameter @p body_index must be
   * between zero and the number of bodies in this tree, which can be determined
   * by calling RigidBodyTree::get_num_bodies().
   */
  const RigidBody<T>& get_body(int body_index) const {
    DRAKE_DEMAND(body_index >= 0 && body_index < get_num_bodies());
    return *bodies_[body_index].get();
  }

  /**
   * Returns the body at index @p body_index. Parameter @p body_index must be
   * between zero and the number of bodies in this tree, which can be determined
   * by calling RigidBodyTree::get_num_bodies().
   */
  RigidBody<T>* get_mutable_body(int body_index);

  /**
   * Returns the number of bodies in this tree. This includes the one body that
   * represents the world.
   */
  int get_num_bodies() const {
    return static_cast<int>(bodies_.size());
  }

  /**
   * Returns the number of frames in this tree.
   */
  int get_num_frames() const;

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

  /// For details on parameters see RigidBodyDistanceContraint.
  void addDistanceConstraint(int bodyA_index_in, const Eigen::Vector3d& r_AP_in,
                             int bodyB_index_in, const Eigen::Vector3d& r_BQ_in,
                             double distance_in);

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
                  Eigen::Dynamic>
        full(compact.rows(), ncols);
    full.setZero();
    int compact_col_start = 0;
    for (std::vector<int>::const_iterator it = joint_path.begin();
         it != joint_path.end(); ++it) {
      RigidBody<T>& body = *bodies_[*it];
      int ncols_joint = in_terms_of_qdot ? body.getJoint().get_num_positions()
                                         : body.getJoint().get_num_velocities();
      int col_start = in_terms_of_qdot ? body.get_position_start_index()
                                       : body.get_velocity_start_index();
      full.middleCols(col_start, ncols_joint) =
          compact.middleCols(compact_col_start, ncols_joint);
      compact_col_start += ncols_joint;
    }
    return full;
  }

  /**
   * A toString method for this class.
   */
  friend std::ostream& operator<<(std::ostream&, const RigidBodyTree<double>&);

  /**
   * @brief Adds and takes ownership of a rigid body. This also adds a frame
   * whose pose is the same as the body's.
   *
   * A RigidBodyTree is the sole owner and manager of the RigidBody's in it.
   * A body is assigned a unique id (RigidBody::id()) when added to a
   * RigidBodyTree. This unique id can be use to access a body using
   * RigidBodyTree::get_bodies()[id].
   *
   * @param[in] body The rigid body to add to this rigid body tree.
   * @return A bare, unowned pointer to the @p body.
   * @pre Neither a body nor frame with the same identifying information (name
   * and model id / name) should already exist in the tree.
   * @throws std::runtime_error if preconditions are not met.
   */
  // TODO(eric.cousineau): Rename to `AddRigidBody`.
  RigidBody<T>* add_rigid_body(std::unique_ptr<RigidBody<T>> body);

  /**
   * Attempts to define a new collision filter group.  The given name *must*
   * be unique since the last invocation of compile() (or construction,
   * whichever is more recent). Duplicate names or attempting to add more
   * collision filter groups than the system can handle will lead to failure. In
   * the event of failure, an exception is thrown. kMaxNumCollisionFilterGroups
   * defines the limit of total collision filter groups that are supported.
   * @param name        The unique name of the new group.
   * @trhows std::logic_error in response to failure conditions.
   */
  void DefineCollisionFilterGroup(const std::string& name);

  /**
   * Adds a RigidBody to a collision filter group. The RigidBody is referenced
   * by name and model instance id. The process will fail if the body cannot be
   * found or if the group cannot be found.
   * @param group_name      The collision filter group name to add the body to.
   * @param body_name       The name of the body to add.
   * @param model_id        The id of the model instance to which this body
   *                        belongs.
   * @throws std::logic_error in response to failure conditions.
   */
  void AddCollisionFilterGroupMember(const std::string& group_name,
                                     const std::string& body_name,
                                     int model_id);

  /**
   * Adds a collision group to the set of groups ignored by the specified
   * collision filter group. Will fail if the specified group name does not
   * refer to an existing collision filter group. (The target group name need
   * not exist at this time.)
   * @param group_name
   * @param target_group_name
   * @throws std::logic_error in response to failure conditions.
   */
  void AddCollisionFilterIgnoreTarget(const std::string& group_name,
                                      const std::string& target_group_name);

  /**
   * @brief Returns a mutable reference to the RigidBody associated with the
   * world in the model. This is the root of the RigidBodyTree.
   */
  RigidBody<T>& world() { return *bodies_[0]; }

  /**
   * @brief Returns a const reference to the RigidBody associated with the
   * world in the model. This is the root of the RigidBodyTree.
   */
  const RigidBody<T>& world() const { return *bodies_[0]; }

  /**
   * Returns the number of position states outputted by this %RigidBodyTree.
   */
  int get_num_positions() const { return num_positions_; }

  /**
   * Returns the number of velocity states outputted by this %RigidBodyTree.
   */
  int get_num_velocities() const { return num_velocities_; }

  /**
   * Returns the number of actuators in this %RigidBodyTree.
   */
  int get_num_actuators() const { return static_cast<int>(actuators.size()); }

  /**
   * Returns whether this %RigidBodyTree is initialized. It is initialized after
   * compile() is called.
   */
  bool initialized() const { return initialized_; }

 public:
  Eigen::VectorXd joint_limit_min;
  Eigen::VectorXd joint_limit_max;

 private:
  // Rigid body objects
  std::vector<std::unique_ptr<RigidBody<T>>> bodies_;

  // Rigid body frames
  std::vector<std::shared_ptr<RigidBodyFrame<T>>> frames_;

 public:
  /// List of bodies.
  // TODO(amcastro-tri): start using accessors body(int).
  auto& get_bodies() const { return bodies_; }

  /// List of frames.
  auto& get_frames() const { return frames_; }

  // Rigid body actuators
  std::vector<RigidBodyActuator, Eigen::aligned_allocator<RigidBodyActuator>>
      actuators;

  // Rigid body loops
  std::vector<RigidBodyLoop<T>,
              Eigen::aligned_allocator<RigidBodyLoop<T>>> loops;

  // Rigid body distance constraints
  std::vector<RigidBodyDistanceConstraint,
      Eigen::aligned_allocator<RigidBodyDistanceConstraint>>
      distance_constraints;

  drake::TwistVector<double> a_grav;
  Eigen::MatrixXd B;  // the B matrix maps inputs into joint-space forces

 private:
  // drake::log()->info() is used for prints if true, and
  // drake::log()->debug() is used otherwise.
  bool print_weld_diasnostics_{false};

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

  // Examines the state of the tree, and confirms that all nodes (i.e,
  // RigidBody instances) have a kinematics path to the root.  In other words,
  // there should only be a single body that has no parent: the world.
  // Throws an exception if it is *not* a complete tree.
  void ConfirmCompleteTree() const;

  // Given the body, tests to see if it has a kinematic path to the world node.
  // Uses a cache of known "connected" bodies to accelerate the computation.
  // The connected set consist of the body indices (see
  // RigidBody::get_body_index) which are known to be connected to the world.
  // This function has a side-effect of updating the set of known connected.
  // This assumes that the connected set has been initialized with the value
  // 0 (the world body).
  // If not connected, throws an exception.
  void TestConnectedToWorld(const RigidBody<T>& body,
                            std::set<int>* connected) const;

  // Reorder body list to ensure parents are before children in the list
  // of bodies.
  //
  // See RigidBodyTree::compile
  void SortTree();

  // Performs the work that is required for the collision state of the
  // RigidBodyTree to become finalized.
  void CompileCollisionState();

  // Defines a number of collision cliques to be used by
  // drake::multibody::collision::Model.
  // Collision cliques are defined so that:
  // - Collision elements on a single body are *not* considered for collision.
  // - Collision elements on *pairs* of RigidBody instances are configured not
  //   to be considered for collision if RigidBody::CanCollideWith() returns
  //   false.
  //
  // @see RigidBody::CanCollideWith.
  void CreateCollisionCliques(std::unordered_set<RigidBody<T>*>* recompile_set);

  // collision_model maintains a collection of the collision geometry in the
  // RBM for use in collision detection of different kinds. Small margins are
  // applied to all collision geometry when that geometry is added, to improve
  // the numerical stability of contact gradients taken using the model.
  std::unique_ptr<drake::multibody::collision::Model> collision_model_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  RigidBodyTree(const RigidBodyTree&);
  RigidBodyTree& operator=(const RigidBodyTree&) { return *this; }

  // The positional arguments identically match the public non-Do variant above.
  template <typename Scalar, typename PointScalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
  DoTransformPointsJacobian(
      const KinematicsCache<Scalar>&,
      const Eigen::Ref<const drake::Matrix3X<PointScalar>>&,
      int, int, bool) const;

  // The positional arguments identically match the public non-Do variant above.
  template <typename Scalar>
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> DoTransformPointsJacobianDotTimesV(
      const KinematicsCache<Scalar>&, const Eigen::Ref<const Eigen::Matrix3Xd>&,
      int, int) const;

  // TODO(SeanCurtis-TRI): This isn't properly used.
  // No query operations should work if it hasn't been initialized.  Calling
  // compile() is the only thing that should set this. Furthermore, any
  // operation that changes the tree (e.g., adding a body, collision element,
  // etc.) should clear the bit again, requiring another call to compile().
  bool initialized_{false};

  int next_available_clique_ = 0;

 private:
  // A utility class for storing body collision data during RBT instantiation.
  struct BodyCollisionItem {
    BodyCollisionItem(const std::string& grp_name, size_t element_index)
        : group_name(grp_name), element(element_index) {
    }
    std::string group_name;
    size_t element;
  };

  typedef std::vector<BodyCollisionItem> BodyCollisions;
  // This data structure supports an orderly instantiation of the collision
  // elements.  It is populated during tree construction, exercised during
  // RigidBodyTree::compile() at the conclusion of which it is emptied.
  // It has no run-time value.  This is a hacky alternative to having a
  // proper Intermediate Representation (IR).
  std::unordered_map<RigidBody<T>*, BodyCollisions> body_collision_map_;

  // Bullet's collision results are affected by the order in which the collision
  // elements are added. This queues the collision elements in the added order
  // so that when they are registered with the collision engine, they'll be
  // submitted in the invocation order.
  //
  // For more information, see:
  //     https://github.com/bulletphysics/bullet3/issues/888
  std::vector<std::unique_ptr<drake::multibody::collision::Element>>
      element_order_;

  // A manager for instantiating and managing collision filter groups.
  drake::multibody::collision::CollisionFilterGroupManager<T>
      collision_group_manager_{};
};

// This template method specialization is defined in the cc file.
template <>
std::unique_ptr<RigidBodyTree<double>> RigidBodyTree<double>::Clone() const;


// To mitigate compilation time, only one `*.o` file will instantiate the
// methods of RigidBodyTree that are templated only on `T` (and not on, e.g.,
// `Scalar`).  Refer to the the rigid_body_tree.cc comments for details.
extern template class RigidBodyTree<double>;

typedef RigidBodyTree<double> RigidBodyTreed;
