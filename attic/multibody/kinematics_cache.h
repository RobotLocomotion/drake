#pragma once

#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/joints/drake_joint.h"
#include "drake/multibody/kinematic_path.h"

template <typename T>
class KinematicsCacheElement {
 public:
  /*
   * Configuration dependent
   */

  Eigen::Transform<T, drake::kSpaceDimension, Eigen::Isometry>
      transform_to_world;
  Eigen::Matrix<T, drake::kTwistSize, Eigen::Dynamic, 0, drake::kTwistSize,
                DrakeJoint::MAX_NUM_VELOCITIES>
      motion_subspace_in_body;  // gradient w.r.t. q_i only
  Eigen::Matrix<T, drake::kTwistSize, Eigen::Dynamic, 0, drake::kTwistSize,
                DrakeJoint::MAX_NUM_VELOCITIES>
      motion_subspace_in_world;  // gradient w.r.t. q

  // Jacobian matrix of quasi-coordinate variables computed with respect
  // to generalized coordinate variables.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0,
                DrakeJoint::MAX_NUM_VELOCITIES,
                DrakeJoint::MAX_NUM_POSITIONS> qdot_to_v;

  // Jacobian matrix of generalized coordinate variables computed with respect
  // to quasi-coordinate variables.
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, 0,
                DrakeJoint::MAX_NUM_POSITIONS,
                DrakeJoint::MAX_NUM_VELOCITIES> v_to_qdot;
  drake::SquareTwistMatrix<T> inertia_in_world;
  drake::SquareTwistMatrix<T> crb_in_world;

  /*
   * Configuration and velocity dependent
   */

  // Gradient with respect to q only.  The gradient with respect to v is
  // motion_subspace_in_world.
  drake::TwistVector<T> twist_in_world;
  // Gradient with respect to q_i and v_i only.
  drake::TwistVector<T> motion_subspace_in_body_dot_times_v;
  // Gradient with respect to q and v.
  drake::TwistVector<T> motion_subspace_in_world_dot_times_v;

 public:
  KinematicsCacheElement(int num_positions_joint, int num_velocities_joint);

  int get_num_positions() const { return static_cast<int>(v_to_qdot.rows()); }
  int get_num_velocities() const { return static_cast<int>(v_to_qdot.cols()); }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename T>
class KinematicsCache {
 private:
  std::vector<KinematicsCacheElement<T>,
              Eigen::aligned_allocator<KinematicsCacheElement<T>>> elements_;
  int num_positions_;
  int num_velocities_;
  Eigen::Matrix<T, Eigen::Dynamic, 1> q;
  Eigen::Matrix<T, Eigen::Dynamic, 1> v;
  bool velocity_vector_valid;
  bool position_kinematics_cached;
  bool jdotV_cached;
  bool inertias_cached;

 public:
  /// @name Preallocated scratch pad variables.
  /// These variables are used to prevent dynamic memory allocation during
  /// runtime.
  /// @{

  /// Preallocated variables used in GeometricJacobian. Preallocated as the size
  /// of the path is dependent on the base body/frame and end effector
  /// body/frame.
  struct DataInGeometricJacobian {
    KinematicPath kinematic_path;
    std::vector<int> start_body_ancestors;
    std::vector<int> end_body_ancestors;
  };
  mutable DataInGeometricJacobian geometric_jacobian_temp;

  /// Preallocated variables used in
  /// CalcFrameSpatialVelocityJacobianInWorldFrame.
  struct DataInCalcFrameSpatialVelocityJacobianInWorldFrame {
    /// Jacobians used as an intermediate representation since the Jacobian can
    /// not be transformed in place.
    drake::Matrix6X<T> J_positions;
    drake::Matrix6X<T> J_velocities;
    /// Vector of indices used to transform to the world frame.
    std::vector<int> v_or_q_indices;
  };
  mutable DataInCalcFrameSpatialVelocityJacobianInWorldFrame
    spatial_velocity_jacobian_temp;

  /// @}

  /// Constructor for a KinematicsCache given the number of positions and
  /// velocities per body in the vectors @p num_joint_positions and
  /// @p num_joint_velocities, respectively.
  ///
  /// For a RigidBodyTree with `nbodies` rigid bodies, `num_joint_positions`
  /// and `num_joint_velocities` are vectors of size `nbodies` containing in
  /// the i-th entry the number of positions and the number of velocities for
  /// the i-th RigidBody in the RigidBodyTree.
  ///
  /// Note that you will typically not create a KinematicsCache object using
  /// this constructor. Instead, you usually obtain a KinematicsCache object
  /// by calling RigidBodyTree::CreateKinematicsCache() or
  /// RigidBodyTree::CreateKinematicsCacheWithType(). The second option is
  /// useful if you need a particular type for your cache like
  /// Eigen::AutoDiffScalar.
  ///
  /// For examples on how to create and use the KinematicsCache, see
  /// rigid_body_tree_dynamics_test.cc and rigid_body_tree_kinematics_test.cc.
  ///
  /// @param num_positions Total number of positions in the RigidBodyTree.
  /// @param num_velocities Total number of velocities in the RigidBodyTree.
  /// @param num_joint_positions A `std::vector<int>` containing in the i-th
  /// entry the number of positions for the i-th body in the RigidBodyTree.
  /// @param num_joint_velocities A `std::vector<int>` containing in the i-th
  /// entry the number of velocities for the i-th body in the RigidBodyTree.
  KinematicsCache(int num_positions, int num_velocities,
                  const std::vector<int>& num_joint_positions,
                  const std::vector<int>& num_joint_velocities);

  /// Requests a cache entry for a body mobilized by a joint with
  /// @p num_positions and @p num_velocities.
  void CreateCacheElement(int num_positions, int num_velocities);

  /// Returns constant reference to a cache entry for body @p body_id.
  const KinematicsCacheElement<T>& get_element(int body_id) const;

  /// Returns mutable pointer to a cache entry for body @p body_id.
  KinematicsCacheElement<T>* get_mutable_element(int body_id);

  template <typename Derived>
  void initialize(const Eigen::MatrixBase<Derived>& q_in);

  template <typename DerivedQ, typename DerivedV>
  void initialize(const Eigen::MatrixBase<DerivedQ>& q_in,
                  const Eigen::MatrixBase<DerivedV>& v_in);

  void checkCachedKinematicsSettings(bool velocity_kinematics_required,
                                     bool jdot_times_v_required,
                                     const char* method_name) const;

  /// Returns `q`, the generalized position vector of the RigidBodyTree that was
  /// used to compute this KinematicsCache.
  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getQ() const;

  /// Returns `v`, the generalized velocity vector of the RigidBodyTree that was
  /// used to compute this KinematicsCache.
  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getV() const;

  /// Returns `x`, the state vector of the RigidBodyTree that was used to
  /// compute this KinematicsCache. This is the concatenation of `q`, the
  /// RigidBodyTree's generalized position vector, and `v` the RigidBodyTree's
  /// generalized velocity vector into a single vector. Within `x`, `q` precedes
  /// `v`.
  Eigen::Matrix<T, Eigen::Dynamic, 1> getX() const;

  /// Returns `true` if this KinematicsCache object has a valid `v` vector. `v`
  /// is the generalized velocity vector of the RigidBodyTree that was used to
  /// compute this KinematicsCache.
  bool hasV() const;

  void setInertiasCached();

  bool areInertiasCached();

  void setPositionKinematicsCached();

  void setJdotVCached(bool jdotV_cached_in);

  int get_num_cache_elements() const;

  int get_num_positions() const;

  int get_num_velocities() const;

 private:
  void invalidate();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
