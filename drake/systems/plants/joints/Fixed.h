#ifndef DRAKE_FIXED_H
#define DRAKE_FIXED_H

#include "drake/systems/plants/joints/JointType.h"

namespace Drake {

template <typename J>
class Fixed : public JointType<J> {
 public:
  using JointType<J>::getNumPositions;
  using JointType<J>::getNumVelocities;

  Fixed() : JointType<J>(1, 1) { }


  inline virtual Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const override {
    return Eigen::VectorXd::Zero(getNumPositions());
  }

  inline virtual std::string getPositionNamePostfix(int index) const override {
    throw std::runtime_error("bad index");
  }

  template <typename Q>
  Transform3D<Promote<J, Q>> jointTransform() const {
    using T = Promote<J, Q>;
    return Transform3D<T>::Identity();
  }

  template <typename Q>
  MotionSubspace<Promote<J, Q>> motionSubspace() const {
    return MotionSubspace<Promote<J, Q>>::Zero(TWIST_SIZE, getNumVelocities());
  }

  template <typename Q>
  SpatialVector<Promote<J, Q>> motionSubspaceDotTimesV() const {
    return SpatialVector<Promote<J, Q>>::Zero();
  };

  template <typename Q>
  ConfigurationDerivativeToVelocity<Promote<J, Q>> configurationDerivativeToVelocity() const {
    return ConfigurationDerivativeToVelocity<Promote<J, Q>>::Zero(getNumVelocities(), getNumPositions());
  }

  template <typename Q>
  VelocityToConfigurationDerivative<Promote<J, Q>> velocityToConfigurationDerivative() const {
    return VelocityToConfigurationDerivative<Promote<J, Q>>::Zero(getNumPositions(), getNumVelocities());
  }

  template <typename V>
  Eigen::Matrix<Promote<J, V>, Eigen::Dynamic, 1> frictionTorque() const {
    return Eigen::Matrix<V, Eigen::Dynamic, 1>::Zero(getNumVelocities(), 1);
  }
};

}

#endif //DRAKE_FIXED_H
