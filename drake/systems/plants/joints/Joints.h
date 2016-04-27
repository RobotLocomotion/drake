//
// Created by Twan Koolen on 4/27/16.
//

#ifndef DRAKE_JOINTS_H_H
#define DRAKE_JOINTS_H_H

#include <Eigen/Geometry>
#include "drake/systems/plants/joints/Joint.h"
#include "drake/systems/plants/joints/JointTypes.h"

namespace Drake {
template <typename J, typename DerivedQ>
Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ> &q) {
  const auto& type = *joint.type;
  if (dynamic_cast<const QuaternionFloating<J>*>(&type)) {
    return dynamic_cast<const QuaternionFloating<J>*>(&type)->jointTransform(q);
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ>
MotionSubspace<Promote<J, typename DerivedQ::Scalar>> motionSubspace(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ> &q) {
  const auto& type = *joint.type;
  using Q = typename DerivedQ::Scalar;
  if (dynamic_cast<const QuaternionFloating<J>*>(&type)) {
    return dynamic_cast<const QuaternionFloating<J>*>(&type)->template motionSubspace<Q>();
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ>
ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ>& q) {
  const auto& type = *joint.type;
  if (dynamic_cast<const QuaternionFloating<J>*>(&type)) {
    return dynamic_cast<const QuaternionFloating<J>*>(&type)->template configurationDerivativeToVelocity(q);
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ>
VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ>& q) {
  const auto& type = *joint.type;
  if (dynamic_cast<const QuaternionFloating<J>*>(&type)) {
    return dynamic_cast<const QuaternionFloating<J>*>(&type)->template velocityToConfigurationDerivative(q);
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedV>
Eigen::Matrix<Promote<J, typename DerivedV::Scalar>, Eigen::Dynamic, 1> frictionTorque(const Joint<J>& joint, const Eigen::MatrixBase<DerivedV>& v) {
  const auto& type = *joint.type;
  using V = typename DerivedV::Scalar;
  if (dynamic_cast<const QuaternionFloating<J>*>(&type)) {
    return dynamic_cast<const QuaternionFloating<J>*>(&type)->template frictionTorque<V>();
  }
  throw std::runtime_error("joint type not handled");
}
}

#endif //DRAKE_JOINTS_H_H
