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
#include "drake/multibody/rigid_body.h"

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

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <typename T>
class KinematicsCache {
 private:
  std::vector<KinematicsCacheElement<T>,
              Eigen::aligned_allocator<KinematicsCacheElement<T>>> elements_;
  std::vector<RigidBody<double> const*> bodies;
  int num_positions;
  int num_velocities;
  Eigen::Matrix<T, Eigen::Dynamic, 1> q;
  Eigen::Matrix<T, Eigen::Dynamic, 1> v;
  bool velocity_vector_valid;
  bool position_kinematics_cached;
  bool jdotV_cached;
  bool inertias_cached;

 public:
  explicit KinematicsCache(
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies_in);

  /// Returns constant reference to a cach entry for body @p body_id.
  const KinematicsCacheElement<T>& get_element(int body_id) const;

  /// Returns mutable pointer to a cach entry for body @p body_id.
  KinematicsCacheElement<T>* get_mutable_element(int body_id);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_mutable_element().")
#endif
  KinematicsCacheElement<T>& getElement(const RigidBody<double>& body);

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_element().")
#endif
  const KinematicsCacheElement<T>& getElement(
      const RigidBody<double>& body) const;

  // TODO(amcastro-tri): This accessor was added to allow
  // RigidBodyTree::transformVelocityMappingToQDotMapping() and
  // RigidBodyTree::transformQDotMappingToVelocityMapping() to be static
  // methods. That is a requirement from velocityToPositionDotMappingmex()
  // and positionDotToVelocityMappingmex() in rigidBodyTreeMexFunctions which
  // define a Matlab API not passing the "this" pointer to the RigidBodyTree
  // model.
  // This problem will be solved when KinematicsCache works with RigidBody
  // id's instead of actual RigidBody pointers as handlers to KinematicsCache
  // entries.
  // Another solution would be to change the Matlab API to pass "this"
  // pointer to the RigidBodyTree as it does with any other RigidBodyTree
  // method.
  const std::vector<RigidBody<double> const*>& get_bodies() const;

  template <typename Derived>
  void initialize(const Eigen::MatrixBase<Derived>& q_in);

  template <typename DerivedQ, typename DerivedV>
  void initialize(const Eigen::MatrixBase<DerivedQ>& q_in,
                  const Eigen::MatrixBase<DerivedV>& v_in);

  void checkCachedKinematicsSettings(bool velocity_kinematics_required,
                                     bool jdot_times_v_required,
                                     const std::string& method_name) const;

  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getQ() const;

  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getV() const;

  Eigen::Matrix<T, Eigen::Dynamic, 1> getX() const;

  bool hasV() const;

  void setInertiasCached();

  bool areInertiasCached();

  void setPositionKinematicsCached();

  void setJdotVCached(bool jdotV_cached_in);

  int get_num_positions() const;

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_positions().")
#endif
  int getNumPositions() const;

  int get_num_velocities() const;

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_velocities().")
#endif
  int getNumVelocities() const;

 private:
  void invalidate();

  // TODO(amcastro-tri): this method should belong to RigidBodyTree and only be
  // used on initialization. The RigidBodyTree should have this value stored so
  // that KinematicsCache can request it when needed. See the KinematicsCache
  // constructor where this request is made.
  // See TODO for get_num_velocities.
  static int get_num_positions(
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies);

  // TODO(amcastro-tri): See TODO for get_num_positions.
  static int get_num_velocities(
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies);

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
