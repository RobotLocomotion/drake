#pragma once

/// @file
/// Template method implementations for kinematics_cache.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/multibody/kinematics_cache.h"

#include <string>
#include <vector>

template <typename T>
KinematicsCacheElement<T>::KinematicsCacheElement(
    int num_positions_joint, int num_velocities_joint)
    : motion_subspace_in_body(drake::kTwistSize, num_velocities_joint),
      motion_subspace_in_world(drake::kTwistSize, num_velocities_joint),
      qdot_to_v(num_velocities_joint, num_positions_joint),
      v_to_qdot(num_positions_joint, num_velocities_joint) {
  // empty
}

template <typename T>
KinematicsCache<T>::KinematicsCache(
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

template <typename T>
KinematicsCacheElement<T>& KinematicsCache<T>::getElement(
    const RigidBody<double>& body) {
  return elements.at(&body);
}

template <typename T>
const KinematicsCacheElement<T>& KinematicsCache<T>::getElement(
    const RigidBody<double>& body) const {
  return elements.at(&body);
}

template <typename T>
template <typename Derived>
void KinematicsCache<T>::initialize(const Eigen::MatrixBase<Derived>& q_in) {
  static_assert(Derived::ColsAtCompileTime == 1, "q must be a vector");
  static_assert(std::is_same<typename Derived::Scalar, T>::value,
                "T type of q must match T type of KinematicsCache");
  DRAKE_ASSERT(q.rows() == q_in.rows());
  q = q_in;
  invalidate();
  velocity_vector_valid = false;
}

template <typename T>
template <typename DerivedQ, typename DerivedV>
void KinematicsCache<T>::initialize(const Eigen::MatrixBase<DerivedQ>& q_in,
                                    const Eigen::MatrixBase<DerivedV>& v_in) {
  initialize(q_in);  // also invalidates
  static_assert(DerivedV::ColsAtCompileTime == 1, "v must be a vector");
  static_assert(std::is_same<typename DerivedV::Scalar, T>::value,
                "T type of v must match T type of KinematicsCache");
  DRAKE_ASSERT(v.rows() == v_in.rows());
  v = v_in;
  velocity_vector_valid = true;
}

template <typename T>
void KinematicsCache<T>::checkCachedKinematicsSettings(
    bool velocity_kinematics_required, bool jdot_times_v_required,
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
    throw std::runtime_error(method_name +
        " requires Jdot times v, which has not been cached. Please call "
        "doKinematics with a velocity vector and compute_JdotV set to true.");
  }
}

template <typename T>
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
              Eigen::Dynamic>
KinematicsCache<T>::transformVelocityMappingToQDotMapping(
    const Eigen::MatrixBase<Derived>& B) const {
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
      A(B.rows(), get_num_positions());
  DRAKE_DEMAND(B.cols() == get_num_velocities());
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

template <typename T>
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
              Eigen::Dynamic>
KinematicsCache<T>::transformQDotMappingToVelocityMapping(
    const Eigen::MatrixBase<Derived>& A) const {
  Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime,
                Eigen::Dynamic>
      B(A.rows(), get_num_velocities());
  DRAKE_DEMAND(A.cols() == get_num_positions());
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

template <typename T>
const Eigen::Matrix<T, Eigen::Dynamic, 1>& KinematicsCache<T>::getQ() const {
  return q;
}

template <typename T>
const Eigen::Matrix<T, Eigen::Dynamic, 1>& KinematicsCache<T>::getV() const {
  if (hasV())
    return v;
  else
    throw std::runtime_error(
        "Kinematics cache has no valid velocity vector.");
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> KinematicsCache<T>::getX() const {
  if (hasV()) {
    Eigen::Matrix<T, Eigen::Dynamic, 1> x(get_num_positions() +
        get_num_velocities());
    x << q, v;
    return x;
  } else {
    return getQ();
  }
}

template <typename T>
bool KinematicsCache<T>::hasV() const { return velocity_vector_valid; }

template <typename T>
void KinematicsCache<T>::setInertiasCached() { inertias_cached = true; }

template <typename T>
bool KinematicsCache<T>::areInertiasCached() { return inertias_cached; }

template <typename T>
void KinematicsCache<T>::setPositionKinematicsCached() {
  position_kinematics_cached = true;
}

template <typename T>
void KinematicsCache<T>::setJdotVCached(bool jdotV_cached_in) {
  jdotV_cached = jdotV_cached_in;
}

template <typename T>
int KinematicsCache<T>::get_num_positions() const { return num_positions; }

template <typename T>
int KinematicsCache<T>::getNumPositions() const { return get_num_positions(); }

template <typename T>
int KinematicsCache<T>::get_num_velocities() const { return num_velocities; }

template <typename T>
int KinematicsCache<T>::getNumVelocities() const {
  return get_num_velocities();
}

template <typename T>
void KinematicsCache<T>::invalidate() {
  position_kinematics_cached = false;
  jdotV_cached = false;
  inertias_cached = false;
}

template <typename T>
int KinematicsCache<T>::get_num_positions(
    const std::vector<std::unique_ptr<RigidBody<double>> >& bodies) {
  auto add_num_positions = [](
      int result, const std::unique_ptr<RigidBody<double>>& body_ptr) -> int {
    return body_ptr->has_parent_body()
           ? result + body_ptr->getJoint().get_num_positions()
           : result;
  };
  return std::accumulate(bodies.begin(), bodies.end(), 0, add_num_positions);
}

template <typename T>
int KinematicsCache<T>::get_num_velocities(
    const std::vector<std::unique_ptr<RigidBody<double>> >& bodies) {
  auto add_num_velocities = [](
      int result, const std::unique_ptr<RigidBody<double>>& body_ptr) -> int {
    return body_ptr->has_parent_body()
           ? result + body_ptr->getJoint().get_num_velocities()
           : result;
  };
  return std::accumulate(bodies.begin(), bodies.end(), 0, add_num_velocities);
}
