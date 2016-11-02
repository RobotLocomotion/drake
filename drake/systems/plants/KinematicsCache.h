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
#include "drake/systems/plants/RigidBody.h"
#include "drake/systems/plants/joints/DrakeJoint.h"

template <typename Scalar>
class KinematicsCacheElement {
 public:
  /*
   * Configuration dependent
   */

  Eigen::Transform<Scalar, drake::kSpaceDimension, Eigen::Isometry>
      transform_to_world;
  Eigen::Matrix<Scalar, drake::kTwistSize, Eigen::Dynamic, 0, drake::kTwistSize,
                DrakeJoint::MAX_NUM_VELOCITIES>
      motion_subspace_in_body;  // gradient w.r.t. q_i only
  Eigen::Matrix<Scalar, drake::kTwistSize, Eigen::Dynamic, 0, drake::kTwistSize,
                DrakeJoint::MAX_NUM_VELOCITIES>
      motion_subspace_in_world;  // gradient w.r.t. q
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                DrakeJoint::MAX_NUM_VELOCITIES,
                DrakeJoint::MAX_NUM_POSITIONS> qdot_to_v;  // gradient w.r.t. q
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0,
                DrakeJoint::MAX_NUM_POSITIONS,
                DrakeJoint::MAX_NUM_VELOCITIES> v_to_qdot;  // gradient w.r.t. q
  drake::SquareTwistMatrix<Scalar> inertia_in_world;
  drake::SquareTwistMatrix<Scalar> crb_in_world;

  /*
   * Configuration and velocity dependent
   */

  // Gradient with respect to q only.  The gradient with respect to v is
  // motion_subspace_in_world.
  drake::TwistVector<Scalar> twist_in_world;
  // Gradient with respect to q_i and v_i only.
  drake::TwistVector<Scalar> motion_subspace_in_body_dot_times_v;
  // Gradient with respect to q and v.
  drake::TwistVector<Scalar> motion_subspace_in_world_dot_times_v;

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

template <typename Scalar>
class KinematicsCache {
 private:
  typedef KinematicsCacheElement<Scalar> KinematicsCacheElementScalar;
  typedef std::pair<RigidBody const* const, KinematicsCacheElementScalar>
      RigidBodyKCacheElementPair;
  typedef Eigen::aligned_allocator<RigidBodyKCacheElementPair>
      RigidBodyKCacheElementPairAllocator;
  typedef std::unordered_map<
      RigidBody const*, KinematicsCacheElementScalar,
      std::hash<RigidBody const*>, std::equal_to<RigidBody const*>,
      RigidBodyKCacheElementPairAllocator> RigidBodyToKCacheElementMap;

  RigidBodyToKCacheElementMap elements;
  std::vector<RigidBody const*> bodies;
  int num_positions;
  int num_velocities;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> v;
  bool velocity_vector_valid;
  bool position_kinematics_cached;
  bool jdotV_cached;
  bool inertias_cached;

 public:
  explicit KinematicsCache(
      const std::vector<std::unique_ptr<RigidBody> >& bodies_in)
      : num_positions(get_num_positions(bodies_in)),
        num_velocities(get_num_velocities(bodies_in)),
        q(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_positions)),
        v(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_velocities)),
        velocity_vector_valid(false) {
    for (const auto& body_unique_ptr : bodies_in) {
      const RigidBody& body = *body_unique_ptr;
      int num_positions_joint =
          body.has_parent_body() ? body.getJoint().get_num_positions() : 0;
      int num_velocities_joint =
          body.has_parent_body() ? body.getJoint().get_num_velocities() : 0;
      elements.insert({&body, KinematicsCacheElement<Scalar>(
                                  num_positions_joint, num_velocities_joint)});
      bodies.push_back(&body);
    }
    invalidate();
  }

  KinematicsCacheElement<Scalar>& getElement(const RigidBody& body) {
    return elements.at(&body);
  }

  const KinematicsCacheElement<Scalar>& getElement(
      const RigidBody& body) const {
    return elements.at(&body);
  }

  template <typename Derived>
  void initialize(const Eigen::MatrixBase<Derived>& q_in) {
    static_assert(Derived::ColsAtCompileTime == 1, "q must be a vector");
    static_assert(std::is_same<typename Derived::Scalar, Scalar>::value,
                  "scalar type of q must match scalar type of KinematicsCache");
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
    static_assert(std::is_same<typename DerivedV::Scalar, Scalar>::value,
                  "scalar type of v must match scalar type of KinematicsCache");
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

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
  transformVelocityMappingToPositionDotMapping(
      const Eigen::MatrixBase<Derived>& mat) const {
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                  Eigen::Dynamic> ret(mat.rows(), get_num_positions());
    int ret_col_start = 0;
    int mat_col_start = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      const RigidBody& body = **it;
      if (body.has_parent_body()) {
        const DrakeJoint& joint = body.getJoint();
        const auto& element = getElement(body);
        ret.middleCols(ret_col_start, joint.get_num_positions()).noalias() =
            mat.middleCols(mat_col_start, joint.get_num_velocities()) *
            element.qdot_to_v;
        ret_col_start += joint.get_num_positions();
        mat_col_start += joint.get_num_velocities();
      }
    }
    return ret;
  }

  template <typename Derived>
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
  transformPositionDotMappingToVelocityMapping(
      const Eigen::MatrixBase<Derived>& mat) const {
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                  Eigen::Dynamic> ret(mat.rows(), get_num_velocities());
    int ret_col_start = 0;
    int mat_col_start = 0;
    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
      const RigidBody& body = **it;
      if (body.has_parent_body()) {
        const DrakeJoint& joint = body.getJoint();
        const auto& element = getElement(body);
        ret.middleCols(ret_col_start, joint.get_num_velocities()).noalias() =
            mat.middleCols(mat_col_start, joint.get_num_positions()) *
            element.v_to_qdot;
        ret_col_start += joint.get_num_velocities();
        mat_col_start += joint.get_num_positions();
      }
    }
    return ret;
  }

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& getQ() const { return q; }

  const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& getV() const {
    if (hasV())
      return v;
    else
      throw std::runtime_error(
          "Kinematics cache has no valid velocity vector.");
  }

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> getX() const {
    if (hasV()) {
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x(get_num_positions() +
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
      const std::vector<std::unique_ptr<RigidBody> >& bodies) {
    auto add_num_positions = [](
        int result, const std::unique_ptr<RigidBody>& body_ptr) -> int {
      return body_ptr->has_parent_body()
                 ? result + body_ptr->getJoint().get_num_positions()
                 : result;
    };
    return std::accumulate(bodies.begin(), bodies.end(), 0, add_num_positions);
  }

  // TODO(amcastro-tri): See TODO for get_num_positions.
  static int get_num_velocities(
      const std::vector<std::unique_ptr<RigidBody> >& bodies) {
    auto add_num_velocities = [](
        int result, const std::unique_ptr<RigidBody>& body_ptr) -> int {
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
