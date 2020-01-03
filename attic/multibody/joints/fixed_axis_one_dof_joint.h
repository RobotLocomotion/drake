#pragma once

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"
#include "drake/multibody/joints/drake_joint_impl.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverloaded-virtual"
template <typename Derived>
class FixedAxisOneDoFJoint : public DrakeJointImpl<Derived> {
 protected:
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  FixedAxisOneDoFJoint(Derived& derived, const std::string& name,
                       const Eigen::Isometry3d& transform_to_parent_body,
                       const drake::TwistVector<double>& _joint_axis)
      : DrakeJointImpl<Derived>(derived, name, transform_to_parent_body, 1, 1),
        joint_axis_(_joint_axis) {}

 public:
  virtual ~FixedAxisOneDoFJoint() {}

  using DrakeJoint::get_num_positions;
  using DrakeJoint::get_num_velocities;

  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(
      const Eigen::MatrixBase<DerivedQ>& q,
      // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
      Eigen::MatrixBase<DerivedMS>& motion_subspace,
      typename drake::math::Gradient<DerivedMS, Eigen::Dynamic>::type*
          dmotion_subspace = nullptr) const {
    drake::unused(q);
    motion_subspace = joint_axis_.cast<typename DerivedQ::Scalar>();
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), get_num_positions());
    }
  }

  template <typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(
      const Eigen::MatrixBase<DerivedQ>& q,
      const Eigen::MatrixBase<DerivedV>& v,
      Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>&
          motion_subspace_dot_times_v,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdq = nullptr,
      typename drake::math::Gradient<
          Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type*
          dmotion_subspace_dot_times_vdv = nullptr) const {
    drake::unused(q, v);

    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(drake::kTwistSize, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(drake::kTwistSize, 1);
    }
  }

  template <typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_VELOCITIES,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_POSITIONS>& qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dqdot_to_v) const {
    qdot_to_v.setIdentity(get_num_velocities(), get_num_positions());
    drake::math::resizeDerivativesToMatchScalar(qdot_to_v, q(0));
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), get_num_positions());
    }
  }

  template <typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ>& q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                            DrakeJoint::MAX_NUM_VELOCITIES>& v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic,
                            Eigen::Dynamic>* dv_to_qdot) const {
    v_to_qdot.setIdentity(get_num_positions(), get_num_velocities());
    drake::math::resizeDerivativesToMatchScalar(v_to_qdot, q(0));
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), get_num_positions());
    }
  }

  template <typename DerivedV>
  Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(
      const Eigen::MatrixBase<DerivedV>& v) const {
    typedef typename DerivedV::Scalar Scalar;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ret(get_num_velocities(), 1);
    using std::abs;
    ret[0] = damping_ * v[0];
    Scalar coulomb_window_fraction = v[0] / coulomb_window_;
    Scalar coulomb =
        std::min(Scalar(1), std::max(Scalar(-1), coulomb_window_fraction)) *
        coulomb_friction_;
    ret[0] += coulomb;
    return ret;
  }

  /// Compute the spring torque for a simple singleaxis joint.
  /// Since this is a one DOF joint, the input and output vectors will be
  /// length 1.
  ///
  /// Torque is computed to be included in the dynamics bias terms, and thus
  /// appears here with a "positive" gain.
  template <typename DerivedQ>
  Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, 1> SpringTorque(
      const Eigen::MatrixBase<DerivedQ>& q) const {
    typedef typename DerivedQ::Scalar Scalar;
    DRAKE_DEMAND(q.size() == 1);
    drake::VectorX<Scalar> torque(get_num_velocities(), 1);
    torque[0] = stiffness_ * (nominal_position_ - q[0]);
    return torque;
  }

  void setJointLimits(double joint_limit_min, double joint_limit_max) {
    if (joint_limit_min > joint_limit_max) {
      throw std::logic_error(
          "ERROR: joint_limit_min cannot be larger than joint_limit_max");
    }

    DrakeJoint::joint_limit_min_[0] = joint_limit_min;
    DrakeJoint::joint_limit_max_[0] = joint_limit_max;
  }

  void SetJointLimitDynamics(double joint_limit_stiffness,
                             double joint_limit_dissipation) {
    DrakeJoint::joint_limit_stiffness_[0] = joint_limit_stiffness;
    DrakeJoint::joint_limit_dissipation_[0] = joint_limit_dissipation;
  }

  Eigen::VectorXd zeroConfiguration() const override {
    return Eigen::VectorXd::Zero(1);
  }

  Eigen::VectorXd randomConfiguration(
      std::default_random_engine& generator) const override {
    Eigen::VectorXd q(1);
    if (std::isfinite(DrakeJoint::joint_limit_min_.value()) &&
        std::isfinite(DrakeJoint::joint_limit_max_.value())) {
      std::uniform_real_distribution<double> distribution(
          DrakeJoint::joint_limit_min_.value(),
          DrakeJoint::joint_limit_max_.value());
      q[0] = distribution(generator);
    } else {
      std::normal_distribution<double> distribution;
      double stddev = 1.0;
      double joint_limit_offset = 1.0;
      if (std::isfinite(DrakeJoint::joint_limit_min_.value())) {
        distribution = std::normal_distribution<double>(
            DrakeJoint::joint_limit_min_.value() + joint_limit_offset, stddev);
      } else if (std::isfinite(DrakeJoint::joint_limit_max_.value())) {
        distribution = std::normal_distribution<double>(
            DrakeJoint::joint_limit_max_.value() - joint_limit_offset, stddev);
      } else {
        distribution = std::normal_distribution<double>();
      }

      q[0] = distribution(generator);
      if (q[0] < DrakeJoint::joint_limit_min_.value()) {
        q[0] = DrakeJoint::joint_limit_min_.value();
      }
      if (q[0] > DrakeJoint::joint_limit_max_.value()) {
        q[0] = DrakeJoint::joint_limit_max_.value();
      }
    }
    return q;
  }

  void setDynamics(double damping, double coulomb_friction,
      double coulomb_window) {
    damping_ = damping;
    coulomb_friction_ = coulomb_friction;
    coulomb_window_ = coulomb_window;
    DRAKE_ASSERT(coulomb_window_ > 0);
  }

  /// Set the spring stiffness and nominal position.
  /// The resuting force will be
  ///   torque = stiffness * (nominal_position - position).
  void SetSpringDynamics(double stiffness, double nominal_position) {
    stiffness_ = stiffness;
    nominal_position_ = nominal_position;
  }

  std::string get_position_name(int index) const override {
    if (index != 0) throw std::runtime_error("bad index");
    return DrakeJoint::name_;
  }

  const drake::TwistVector<double>& joint_axis() const { return joint_axis_; }

  double damping() const { return damping_; }
  double coulomb_friction() const { return coulomb_friction_; }
  double coulomb_window() const { return coulomb_window_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  /// Initializes the private member variables within the provided `clone`.
  void DoInitializeClone(DrakeJoint* clone) const override {
    FixedAxisOneDoFJoint* fixed_axis_one_dof_joint =
        dynamic_cast<FixedAxisOneDoFJoint*>(clone);
    DRAKE_DEMAND(fixed_axis_one_dof_joint != nullptr);
    fixed_axis_one_dof_joint->joint_axis_ = this->joint_axis_;
    fixed_axis_one_dof_joint->damping_ = this->damping_;
    fixed_axis_one_dof_joint->coulomb_friction_ = this->coulomb_friction_;
    fixed_axis_one_dof_joint->coulomb_window_ = this->coulomb_window_;
    fixed_axis_one_dof_joint->stiffness_ = this->stiffness_;
    fixed_axis_one_dof_joint->nominal_position_ = this->nominal_position_;
  }

 private:
  drake::TwistVector<double> joint_axis_;
  double damping_{};
  double coulomb_friction_{};
  double stiffness_{0};
  double nominal_position_{0};
  // We're trying to emulate MATLAB's code:
  // NOLINTNEXTLINE(whitespace/line_length)
  // https://github.com/RobotLocomotion/drake/blob/d7f3c011d37d471d7b9293ecf2066c98d88b2a05/drake/matlab/systems/plants/RigidBody.m#L29
  double coulomb_window_{std::numeric_limits<double>::epsilon()};
};
#pragma GCC diagnostic pop  // pop -Wno-overloaded-virtual
