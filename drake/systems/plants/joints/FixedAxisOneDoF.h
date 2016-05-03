#ifndef DRAKE_FIXEDAXISONEDOF_H
#define DRAKE_FIXEDAXISONEDOF_H

#include "drake/systems/plants/joints/JointType.h"

namespace Drake {

template <typename J>
class FixedAxisOneDoF : public JointType<J> {
 protected:
  SpatialVector<J> joint_axis;
  J damping;
  J coulomb_friction;
  J coulomb_window;

 public:
  using JointType<J>::getNumPositions;
  using JointType<J>::getNumVelocities;
  using JointType<J>::getJointLimitMin;
  using JointType<J>::getJointLimitMax;

  FixedAxisOneDoF(const SpatialVector<J>& joint_axis) : JointType<J>(1, 1), joint_axis(joint_axis), damping(0), coulomb_friction(0), coulomb_window(0) { };

  inline virtual std::string getPositionNamePostfix(int index) const override {
    if (index != 0) throw std::runtime_error("bad index");
    return "";
  }

  inline virtual Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const override {
    Eigen::VectorXd q(getNumPositions());
    const auto& joint_limit_min = getJointLimitMin();
    const auto& joint_limit_max = getJointLimitMax();
    if (std::isfinite(joint_limit_min.value()) &&
        std::isfinite(joint_limit_max.value())) {
      std::uniform_real_distribution<double> distribution(
          joint_limit_min.value(),
          joint_limit_max.value());
      q[0] = distribution(generator);
    } else {
      std::normal_distribution<double> distribution;
      double stddev = 1.0;
      double joint_limit_offset = 1.0;
      if (std::isfinite(joint_limit_min.value())) {
        distribution = std::normal_distribution<double>(
            joint_limit_min.value() + joint_limit_offset, stddev);
      } else if (std::isfinite(joint_limit_max.value())) {
        distribution = std::normal_distribution<double>(
            joint_limit_max.value() - joint_limit_offset, stddev);
      } else {
        distribution = std::normal_distribution<double>();
      }

      q[0] = distribution(generator);
      if (q[0] < joint_limit_min.value()) {
        q[0] = joint_limit_min.value();
      }
      if (q[0] > joint_limit_max.value()) {
        q[0] = joint_limit_max.value();
      }
    }
    return q;
  }

  template <typename Q>
  MotionSubspace<Promote<J, Q>> motionSubspace() const {
    using T = Promote<J, Q>;
    return ConvertMatrix<T>()(joint_axis);
  }

  template <typename Q>
  SpatialVector<Promote<J, Q>> motionSubspaceDotTimesV() const {
    return SpatialVector<Promote<J, Q>>::Zero();
  }

  template <typename Q>
  ConfigurationDerivativeToVelocity<Promote<J, Q>> configurationDerivativeToVelocity() const {
    return ConfigurationDerivativeToVelocity<Promote<J, Q>>::Identity(getNumPositions(), getNumVelocities());
  }

  template <typename Q>
  VelocityToConfigurationDerivative <Promote<J, Q>> velocityToConfigurationDerivative() const {
    return VelocityToConfigurationDerivative<Promote<J, Q>>::Identity(getNumPositions(), getNumVelocities());
  }

  template <typename DerivedV>
  Eigen::Matrix<Promote<J, typename DerivedV::Scalar>, Eigen::Dynamic, 1> frictionTorque(const Eigen::MatrixBase<DerivedV> &v) const {
    using T = Promote<J, typename DerivedV::Scalar>;

    Eigen::Matrix<T, Eigen::Dynamic, 1> ret(getNumVelocities(), 1);
    using std::abs;
    ret[0] = damping * v[0];
    T coulomb_window_fraction = v[0] / coulomb_window;
    T coulomb = std::min(T(1), std::max(T(-1), coulomb_window_fraction)) * coulomb_friction;
    ret[0] += coulomb;
    return ret;
  }

  void setDynamics(const J& damping, const J& coulomb_friction, const J& coulomb_window) {
    this->damping = damping;
    this->coulomb_friction = coulomb_friction;
    this->coulomb_window = coulomb_window;
  }

  void setJointLimits(double joint_limit_min, double joint_limit_max) {
    if (joint_limit_min > joint_limit_max) {
      throw std::logic_error(
          "joint_limit_min cannot be larger than joint_limit_max");
    }

    this->JointType<J>::joint_limit_min[0] = joint_limit_min;
    this->JointType<J>::joint_limit_max[0] = joint_limit_max;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(SpatialVector<J>)%16)==0)
};

}

#endif //DRAKE_FIXEDAXISONEDOF_H
