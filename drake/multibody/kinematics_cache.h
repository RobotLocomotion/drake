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
  KinematicsCacheElement(int num_positions_joint, int num_velocities_joint)
      : motion_subspace_in_body(drake::kTwistSize, num_velocities_joint),
        motion_subspace_in_world(drake::kTwistSize, num_velocities_joint),
        qdot_to_v(num_velocities_joint, num_positions_joint),
        v_to_qdot(num_positions_joint, num_velocities_joint) {
    // empty
  }

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};

template <typename T>
class KinematicsCache {
 private:
  typedef KinematicsCacheElement<T> KinematicsCacheElementT;
  typedef std::pair<RigidBody<double> const* const, KinematicsCacheElementT>
      RigidBodyKCacheElementPair;
  typedef Eigen::aligned_allocator<RigidBodyKCacheElementPair>
      RigidBodyKCacheElementPairAllocator;
  typedef std::unordered_map<RigidBody<double> const*, KinematicsCacheElementT,
                             std::hash<RigidBody<double> const*>,
                             std::equal_to<RigidBody<double> const*>,
                             RigidBodyKCacheElementPairAllocator>
      RigidBodyToKCacheElementMap;

  RigidBodyToKCacheElementMap elements;
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
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies_in)
      : num_positions(get_num_positions(bodies_in)),
        num_velocities(get_num_velocities(bodies_in)),
        q(Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_positions)),
        v(Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_velocities)),
        velocity_vector_valid(false) {
    for (const auto& body_unique_ptr : bodies_in) {
      const RigidBody<double>& body = *body_unique_ptr;
      int num_positions_joint =
          body.has_parent_body() ? body.getJoint().get_num_positions() : 0;
      int num_velocities_joint =
          body.has_parent_body() ? body.getJoint().get_num_velocities() : 0;
      elements.insert({&body, KinematicsCacheElement<T>(
                                  num_positions_joint, num_velocities_joint)});
      bodies.push_back(&body);
    }
    invalidate();
  }

  KinematicsCacheElement<T>& getElement(const RigidBody<double>& body) {
    return elements.at(&body);
  }

  const KinematicsCacheElement<T>& getElement(
      const RigidBody<double>& body) const {
    return elements.at(&body);
  }

  template <typename Derived>
  void initialize(const Eigen::MatrixBase<Derived>& q_in) {
    static_assert(Derived::ColsAtCompileTime == 1, "q must be a vector");
    static_assert(std::is_same<typename Derived::Scalar, T>::value,
                  "T type of q must match T type of KinematicsCache");
    DRAKE_ASSERT(q.rows() == q_in.rows());
    q = q_in;
    invalidate();
    velocity_vector_valid = false;
  }

  template <typename DerivedQ, typename DerivedV>
  void initialize(const Eigen::MatrixBase<DerivedQ>& q_in,
                  const Eigen::MatrixBase<DerivedV>& v_in) {
    initialize(q_in);  // also invalidates
    static_assert(DerivedV::ColsAtCompileTime == 1, "v must be a vector");
    static_assert(std::is_same<typename DerivedV::Scalar, T>::value,
                  "T type of v must match T type of KinematicsCache");
    DRAKE_ASSERT(v.rows() == v_in.rows());
    v = v_in;
    velocity_vector_valid = true;
  }

  void checkCachedKinematicsSettings(bool velocity_kinematics_required,
                                     bool jdot_times_v_required,
                                     const std::string& method_name) const {
    if (!position_kinematics_cached) {
      throw std::runtime_error(method_name +
                               " requires position kinematics, which have not "
                               "been cached. Please call doKinematics.");
    }
    if (velocity_kinematics_required && !hasV()) {
      throw std::runtime_error(method_name +
                               " requires velocity kinematics, which have not "
                               "been cached. Please call doKinematics with a "
                               "velocity vector.");
    }
    if (jdot_times_v_required && !jdotV_cached) {
      throw std::runtime_error(
          method_name +
          " requires Jdot times v, which has not been cached. Please call "
          "doKinematics with a velocity vector and compute_JdotV set to true.");
    }
  }

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
   * @param B, a `m x nv` sized matrix, where `nv` is the dimension of the
   *      generalized velocities.
   * @retval A a `m x nq` sized matrix, where `nq` is the dimension of the
   *      generalized coordinates.
   * @sa transformQDotMappingToVelocityMapping()
   */
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
  transformVelocityMappingToQDotMapping(
      const Eigen::MatrixBase<Derived>& B) const {
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                  Eigen::Dynamic>
        A(B.rows(), get_num_positions());
    int A_col_start = 0;
    int B_col_start = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      const RigidBody<double>& body = **it;
      if (body.has_parent_body()) {
        const DrakeJoint& joint = body.getJoint();
        const auto& element = getElement(body);
        A.middleCols(A_col_start, joint.get_num_positions()).noalias() =
            B.middleCols(B_col_start, joint.get_num_velocities()) *
            element.qdot_to_v;
        A_col_start += joint.get_num_positions();
        B_col_start += joint.get_num_velocities();
      }
    }
    return A;
  }

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
   * @param A a `m x nq` sized matrix, where `nq` is the dimension of the
   *      generalized coordinates.
   * @retval B, a `m x nv` sized matrix, where `nv` is the dimension of the
   *      generalized velocities.
   * @sa transformVelocityMappingToQDotMapping()
   */
  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
  transformQDotMappingToVelocityMapping(
      const Eigen::MatrixBase<Derived>& A) const {
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                  Eigen::Dynamic>
        B(A.rows(), get_num_velocities());
    int B_col_start = 0;
    int A_col_start = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      const RigidBody<double>& body = **it;
      if (body.has_parent_body()) {
        const DrakeJoint& joint = body.getJoint();
        const auto& element = getElement(body);
        B.middleCols(B_col_start, joint.get_num_velocities()).noalias() =
            A.middleCols(A_col_start, joint.get_num_positions()) *
            element.v_to_qdot;
        B_col_start += joint.get_num_velocities();
        A_col_start += joint.get_num_positions();
      }
    }
    return B;
  }

  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getQ() const { return q; }

  const Eigen::Matrix<T, Eigen::Dynamic, 1>& getV() const {
    if (hasV())
      return v;
    else
      throw std::runtime_error(
          "Kinematics cache has no valid velocity vector.");
  }

  Eigen::Matrix<T, Eigen::Dynamic, 1> getX() const {
    if (hasV()) {
      Eigen::Matrix<T, Eigen::Dynamic, 1> x(get_num_positions() +
                                                 get_num_velocities());
      x << q, v;
      return x;
    } else {
      return getQ();
    }
  }

  bool hasV() const { return velocity_vector_valid; }

  void setInertiasCached() { inertias_cached = true; }

  bool areInertiasCached() { return inertias_cached; }

  void setPositionKinematicsCached() { position_kinematics_cached = true; }

  void setJdotVCached(bool jdotV_cached_in) { jdotV_cached = jdotV_cached_in; }

  int get_num_positions() const { return num_positions; }

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_positions().")
#endif
  int getNumPositions() const { return get_num_positions(); }

  int get_num_velocities() const { return num_velocities; }

// TODO(liang.fok): Remove this deprecated method prior to Release 1.0.
#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_velocities().")
#endif
  int getNumVelocities() const { return get_num_velocities(); }

 private:
  void invalidate() {
    position_kinematics_cached = false;
    jdotV_cached = false;
    inertias_cached = false;
  }

  // TODO(amcastro-tri): this method should belong to RigidBodyTree and only be
  // used on initialization. The RigidBodyTree should have this value stored so
  // that KinematicsCache can request it when needed. See the KinematicsCache
  // constructor where this request is made.
  // See TODO for get_num_velocities.
  static int get_num_positions(
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies) {
    auto add_num_positions = [](
        int result, const std::unique_ptr<RigidBody<double>>& body_ptr) -> int {
      return body_ptr->has_parent_body()
                 ? result + body_ptr->getJoint().get_num_positions()
                 : result;
    };
    return std::accumulate(bodies.begin(), bodies.end(), 0, add_num_positions);
  }

  // TODO(amcastro-tri): See TODO for get_num_positions.
  static int get_num_velocities(
      const std::vector<std::unique_ptr<RigidBody<double>> >& bodies) {
    auto add_num_velocities = [](
        int result, const std::unique_ptr<RigidBody<double>>& body_ptr) -> int {
      return body_ptr->has_parent_body()
                 ? result + body_ptr->getJoint().get_num_velocities()
                 : result;
    };
    return std::accumulate(bodies.begin(), bodies.end(), 0, add_num_velocities);
  }

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
