#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/controllers/qp_inverse_dynamics/deprecated.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

/**
 * A wrapper class around KinematicsCache and several useful matrices such as
 * the inertia matrix, etc. This class serves mainly as a explicit cache to
 * avoid repeated computation. This class can be replaced when System's cache
 * is ready.
 */
template <typename T>
class DRAKE_DEPRECATED_QPID RobotKinematicState {
 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RobotKinematicState)

 public:
  /**
   * Constructor. The internal KinematicsCache is initialized to @p robot's
   * zero configuration and zero velocity. Internal time is set to zero.
   * @param robot Is aliased internally. @p robot's life span must be longer
   * than this.
   */
  explicit RobotKinematicState(const RigidBodyTree<T>* robot)
      : robot_(robot), cache_(robot_->CreateKinematicsCache()) {
    UpdateKinematics(0, robot_->getZeroConfiguration(),
                     VectorX<T>::Zero(robot_->get_num_velocities()));
  }

  virtual ~RobotKinematicState() {}

  std::unique_ptr<RobotKinematicState<T>> Clone() const {
    return std::unique_ptr<RobotKinematicState<T>>(DoClone());
  }

  /**
   * The generalized position @p q and velocity @p v are used to update the
   * internal KinematicsCache. @p time is used to update the internal time.
   * @param time In seconds
   * @param q Generalized positions.
   * @param v Generalized velocities.
   */
  void UpdateKinematics(T t, const Eigen::Ref<const VectorX<T>>& q,
                        const Eigen::Ref<const VectorX<T>>& v) {
    time_ = t;
    UpdateKinematics(q, v);
  }

  /**
   * The generalized position @p q and velocity @p v are used to update the
   * internal KinematicsCache.
   * @param q Generalized positions.
   * @param v Generalized velocities.
   */
  void UpdateKinematics(const Eigen::Ref<const VectorX<T>>& q,
                        const Eigen::Ref<const VectorX<T>>& v) {
    cache_.initialize(q, v);
    robot_->doKinematics(cache_, true);

    M_ = robot_->massMatrix(cache_);
    com_ = robot_->centerOfMass(cache_);
    drake::eigen_aligned_std_unordered_map<RigidBody<double> const*,
                                           drake::TwistVector<double>>
        f_ext;
    bias_term_ = robot_->dynamicsBiasTerm(cache_, f_ext);
    centroidal_momentum_matrix_ = robot_->centroidalMomentumMatrix(cache_);
    centroidal_momentum_matrix_dot_times_v_ =
        robot_->centroidalMomentumMatrixDotTimesV(cache_);
    centroidal_momentum_ = centroidal_momentum_matrix_ * v;
  }

  T get_time() const { return time_; }
  const RigidBodyTree<T>& get_robot() const { return *robot_; }
  const KinematicsCache<T>& get_cache() const { return cache_; }

  const Vector3<T>& get_com() const { return com_; }
  Vector3<T> get_com_velocity() const {
    return centroidal_momentum_.template tail<3>() / robot_->getMass();
  }
  const Vector6<T>& get_centroidal_momentum() const {
    return centroidal_momentum_;
  }

  const MatrixX<T>& get_M() const { return M_; }
  const VectorX<T>& get_bias_term() const { return bias_term_; }
  const Matrix6X<T>& get_centroidal_momentum_matrix() const {
    return centroidal_momentum_matrix_;
  }
  const Vector6<T>& get_centroidal_momentum_matrix_dot_times_v() const {
    return centroidal_momentum_matrix_dot_times_v_;
  }

 protected:
  virtual RobotKinematicState<T>* DoClone() const {
    return new RobotKinematicState<T>(*this);
  }

 private:
  const RigidBodyTree<T>* robot_;
  KinematicsCache<T> cache_;

  T time_;

  // Equation of motion: M * vd + h = tau + J^T * lambda, where J is the
  // stacked contact Jacobian, lambda is the contact forces, and tau is the
  // joint torques.
  // Inertial matrix (M)
  MatrixX<T> M_;
  // Bias term (h)
  VectorX<T> bias_term_;

  // Center of mass
  Vector3<T> com_;
  // Centroidal momentum = [angular; linear] momentum.
  Vector6<T> centroidal_momentum_;

  // [angular; linear] = centroidal_momentum_matrix_ * v
  Matrix6X<T> centroidal_momentum_matrix_;
  Vector6<T> centroidal_momentum_matrix_dot_times_v_;
};

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
