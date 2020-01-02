#pragma once

/// @file
/// Template method implementations for kinematics_cache.h.
/// Most users should only include that file, not this one.
/// For background, see https://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/multibody/kinematics_cache.h"
/* clang-format on */

#include <algorithm>
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
void KinematicsCache<T>::CreateCacheElement(
    int num_positions, int num_velocities) {
  elements_.emplace_back(num_positions, num_velocities);
}

template <typename T>
KinematicsCache<T>::KinematicsCache(
    int num_positions, int num_velocities,
    const std::vector<int>& num_joint_positions,
    const std::vector<int>& num_joint_velocities)
    : num_positions_(num_positions),
      num_velocities_(num_velocities),
      q(Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_positions_)),
      v(Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(num_velocities_)),
      velocity_vector_valid(false) {
  DRAKE_DEMAND(num_joint_positions.size() == num_joint_velocities.size());
  for (int body_id = 0;
       body_id < static_cast<int>(num_joint_positions.size()); ++body_id) {
    elements_.emplace_back(num_joint_positions[body_id],
                           num_joint_velocities[body_id]);
  }


  geometric_jacobian_temp.kinematic_path.joint_path.reserve(num_positions_);
  geometric_jacobian_temp.
    kinematic_path.joint_direction_signs.reserve(num_positions_);
  geometric_jacobian_temp.kinematic_path.body_path.reserve(num_positions_);
  geometric_jacobian_temp.start_body_ancestors.reserve(num_positions_);
  geometric_jacobian_temp.end_body_ancestors.reserve(num_positions_);
  spatial_velocity_jacobian_temp.J_positions.setZero(6, num_positions_);
  spatial_velocity_jacobian_temp.J_velocities.setZero(6, num_velocities_);
  spatial_velocity_jacobian_temp.
    v_or_q_indices.reserve(std::max(num_positions_, num_velocities_));

  invalidate();
}

template <typename T>
const KinematicsCacheElement<T>& KinematicsCache<T>::get_element(
    int body_id) const {
  return elements_[body_id];
}

template <typename T>
KinematicsCacheElement<T>* KinematicsCache<T>::get_mutable_element(
    int body_id) {
  return &elements_[body_id];
}

template <typename T>
template <typename Derived>
void KinematicsCache<T>::initialize(const Eigen::MatrixBase<Derived>& q_in) {
  static_assert(Derived::ColsAtCompileTime == 1, "q must be a vector");
  static_assert(std::is_same<typename Derived::Scalar, T>::value,
                "T type of q must match T type of KinematicsCache");
  DRAKE_DEMAND(q.rows() == q_in.rows());
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
  DRAKE_DEMAND(v.rows() == v_in.rows());
  v = v_in;
  velocity_vector_valid = true;
}

template <typename T>
void KinematicsCache<T>::checkCachedKinematicsSettings(
    bool velocity_kinematics_required, bool jdot_times_v_required,
    const char* method_name) const {
  if (!position_kinematics_cached) {
    throw std::runtime_error(std::string(method_name) +
        " requires position kinematics, which have not "
            "been cached. Please call doKinematics.");
  }
  if (velocity_kinematics_required && !hasV()) {
    throw std::runtime_error(std::string(method_name) +
        " requires velocity kinematics, which have not "
            "been cached. Please call doKinematics with a "
            "velocity vector.");
  }
  if (jdot_times_v_required && !jdotV_cached) {
    throw std::runtime_error(std::string(method_name) +
        " requires Jdot times v, which has not been cached. Please call "
        "doKinematics with a velocity vector and compute_JdotV set to true.");
  }
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
int KinematicsCache<T>::get_num_cache_elements() const {
  return static_cast<int>(elements_.size());
}

template <typename T>
int KinematicsCache<T>::get_num_positions() const { return num_positions_; }

template <typename T>
int KinematicsCache<T>::get_num_velocities() const { return num_velocities_; }

template <typename T>
void KinematicsCache<T>::invalidate() {
  position_kinematics_cached = false;
  jdotV_cached = false;
  inertias_cached = false;
}
