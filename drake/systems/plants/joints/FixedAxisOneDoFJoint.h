#ifndef ONEDOFJOINT_H_
#define ONEDOFJOINT_H_

#include "DrakeJointImpl.h"
#include <cmath>
#include <Eigen/Core>
#include <limits>
#include <exception>
#include <stdexcept>
#include "drakeGradientUtil.h"


template <typename Derived>
class FixedAxisOneDoFJoint : public DrakeJointImpl<Derived>
{
  // disable copy construction and assignment
  //FixedAxisOneDoFJoint(const DrakeJoint&) = delete;
  //FixedAxisOneDoFJoint& operator=(const FixedAxisOneDoFJoint&) = delete;

private:
  Eigen::Matrix<double, TWIST_SIZE, 1> joint_axis;
  double damping;
  double coulomb_friction;
  double coulomb_window;

protected:
  FixedAxisOneDoFJoint(Derived& derived, const std::string& name, const Eigen::Isometry3d& transform_to_parent_body, const Eigen::Matrix<double, TWIST_SIZE, 1>& _joint_axis) :
      DrakeJointImpl<Derived>(derived, name, transform_to_parent_body, 1, 1),
      joint_axis(_joint_axis),
      damping(0.0),
      coulomb_friction(0.0),
      coulomb_window(0.0) { };

public:
  virtual ~FixedAxisOneDoFJoint() {};

  using DrakeJoint::getNumPositions;
  using DrakeJoint::getNumVelocities;
  
  template <typename DerivedQ, typename DerivedMS>
  void motionSubspace(const Eigen::MatrixBase<DerivedQ> & q,
                      Eigen::MatrixBase<DerivedMS>& motion_subspace,
                      typename Gradient<DerivedMS, Eigen::Dynamic>::type* dmotion_subspace = nullptr) const {
    motion_subspace = joint_axis.cast<typename DerivedQ::Scalar>();
    if (dmotion_subspace) {
      dmotion_subspace->setZero(motion_subspace.size(), getNumPositions());
    }
  };

  template<typename DerivedQ, typename DerivedV>
  void motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v,
                               Eigen::Matrix<typename DerivedQ::Scalar, 6, 1> &motion_subspace_dot_times_v,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdq = nullptr,
                               typename Gradient<Eigen::Matrix<typename DerivedQ::Scalar, 6, 1>, Eigen::Dynamic>::type *dmotion_subspace_dot_times_vdv = nullptr) const {
    motion_subspace_dot_times_v.setZero();

    if (dmotion_subspace_dot_times_vdq) {
      dmotion_subspace_dot_times_vdq->setZero(TWIST_SIZE, 1);
    }

    if (dmotion_subspace_dot_times_vdv) {
      dmotion_subspace_dot_times_vdv->setZero(TWIST_SIZE, 1);
    }
  };

  template<typename DerivedQ>
  void qdot2v(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_VELOCITIES, DrakeJoint::MAX_NUM_POSITIONS> &qdot_to_v,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dqdot_to_v) const {
    qdot_to_v.setIdentity(getNumVelocities(), getNumPositions());
    if (dqdot_to_v) {
      dqdot_to_v->setZero(qdot_to_v.size(), getNumPositions());
    }
  };

  template<typename DerivedQ>
  void v2qdot(const Eigen::MatrixBase<DerivedQ> & q,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, DrakeJoint::MAX_NUM_POSITIONS, DrakeJoint::MAX_NUM_VELOCITIES> &v_to_qdot,
              Eigen::Matrix<typename DerivedQ::Scalar, Eigen::Dynamic, Eigen::Dynamic> *dv_to_qdot) const {
    v_to_qdot.setIdentity(getNumPositions(), getNumVelocities());
    if (dv_to_qdot) {
      dv_to_qdot->setZero(v_to_qdot.size(), getNumPositions());
    }
  };

  template <typename DerivedV>
  GradientVar<typename DerivedV::Scalar, Eigen::Dynamic, 1> frictionTorque(const Eigen::MatrixBase<DerivedV> & v, int gradient_order) const {
    GradientVar<typename DerivedV::Scalar, Eigen::Dynamic, 1> ret(getNumVelocities(), 1, getNumVelocities(), gradient_order);
    using std::abs;
    typedef typename DerivedV::Scalar Scalar;
    ret.value()[0] = damping * v[0];
    Scalar coulomb_window_fraction = v[0] / coulomb_window;
    ret.value()[0] += std::min(static_cast<Scalar>(1.0), std::max(static_cast<Scalar>(-1.0), coulomb_window_fraction)) * coulomb_friction;
    if (gradient_order > 0) {
      ret.gradient().value()(0, 0) = damping;
      if (abs(v[0]) < coulomb_window)
        ret.gradient().value()(0, 0) += sign(v[0]) * (coulomb_friction / coulomb_window);
    }
    return ret;
  }

  void setJointLimits(double joint_limit_min, double joint_limit_max)
  {
    if (joint_limit_min > joint_limit_max) {
      throw std::logic_error("joint_limit_min cannot be larger than joint_limit_max");
    }

    this->DrakeJoint::joint_limit_min[0] = joint_limit_min;
    this->DrakeJoint::joint_limit_max[0] = joint_limit_max;
  }

  Eigen::VectorXd randomConfiguration(std::default_random_engine& generator) const
  {
    Eigen::VectorXd q(1);
    if (std::isfinite(DrakeJoint::joint_limit_min.value()) && std::isfinite(DrakeJoint::joint_limit_max.value())) {
      std::uniform_real_distribution<double> distribution(DrakeJoint::joint_limit_min.value(), DrakeJoint::joint_limit_max.value());
      q[0] = distribution(generator);
    }
    else {
      std::normal_distribution<double> distribution;
      double stddev = 1.0;
      double joint_limit_offset = 1.0;
      if (std::isfinite(DrakeJoint::joint_limit_min.value())) {
        distribution = std::normal_distribution<double>(DrakeJoint::joint_limit_min.value() + joint_limit_offset, stddev);
      }
      else if (std::isfinite(DrakeJoint::joint_limit_max.value())) {
        distribution = std::normal_distribution<double>(DrakeJoint::joint_limit_max.value() - joint_limit_offset, stddev);
      }
      else {
        distribution = std::normal_distribution<double>();
      }

      q[0] = distribution(generator);
      if (q[0] < DrakeJoint::joint_limit_min.value()) {
        q[0] = DrakeJoint::joint_limit_min.value();
      }
      if (q[0] > DrakeJoint::joint_limit_max.value()) {
        q[0] = DrakeJoint::joint_limit_max.value();
      }
    }
    return q;
  }

  void setDynamics(double damping, double coulomb_friction, double coulomb_window)
  {
    this->damping = damping;
    this->coulomb_friction = coulomb_friction;
    this->coulomb_window = coulomb_window;
  }


  virtual std::string getPositionName(int index) const { if (index!=0) throw std::runtime_error("bad index"); return DrakeJoint::name; }

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif /* ONEDOFJOINT_H_ */
