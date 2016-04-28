//
// Created by Twan Koolen on 4/27/16.
//

#ifndef DRAKE_JOINTS_H_H
#define DRAKE_JOINTS_H_H

#include "drake/systems/plants/joints/Joint.h"
#include "drake/systems/plants/joints/QuaternionFloating.h"
#include "drake/systems/plants/joints/RollPitchYawFloating.h"

namespace Drake {
template <typename J, typename DerivedQ>
Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ> &q) {
  const auto& type = *joint.type;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->jointTransform(q);
  }
  auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(&type);
  if (rollPitchYawFloatingJoint) {
    return rollPitchYawFloatingJoint->jointTransform(q);
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ>
MotionSubspace<Promote<J, typename DerivedQ::Scalar>> motionSubspace(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ> &q) {
  const auto& type = *joint.type;
  using Q = typename DerivedQ::Scalar;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->template motionSubspace<Q>();
  }
  auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(&type);
  if (rollPitchYawFloatingJoint) {
    return rollPitchYawFloatingJoint->motionSubspace(q);
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ, typename DerivedV>
SpatialVector<Promote<J, typename DerivedQ::Scalar>> motionSubspaceDotTimesV(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v) {
  const auto& type = *joint.type;
  using Q = typename DerivedQ::Scalar;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->template motionSubspaceDotTimesV();
  }
};

template <typename J, typename DerivedQ>
ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ>& q) {
  const auto& type = *joint.type;
  using Q = typename DerivedQ::Scalar;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->configurationDerivativeToVelocity(q);
  }
  auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(&type);
  if (rollPitchYawFloatingJoint) {
    return rollPitchYawFloatingJoint->template configurationDerivativeToVelocity<Q>();
  }
  throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedQ>
VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Joint<J>& joint, const Eigen::MatrixBase<DerivedQ>& q) {
  const auto& type = *joint.type;
  using Q = typename DerivedQ::Scalar;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->velocityToConfigurationDerivative(q);
  }
  auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(&type);
  if (rollPitchYawFloatingJoint) {
    return rollPitchYawFloatingJoint->template velocityToConfigurationDerivative<Q>();
  }
    throw std::runtime_error("joint type not handled");
}

template <typename J, typename DerivedV>
Eigen::Matrix<Promote<J, typename DerivedV::Scalar>, Eigen::Dynamic, 1> frictionTorque(const Joint<J>& joint, const Eigen::MatrixBase<DerivedV>& v) {
  const auto& type = *joint.type;
  using V = typename DerivedV::Scalar;

  auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(&type);
  if (quaternionFloatingJoint) {
    return quaternionFloatingJoint->template frictionTorque<V>();
  }
  auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(&type);
  if (rollPitchYawFloatingJoint) {
    return rollPitchYawFloatingJoint->template frictionTorque<V>();
  }
  throw std::runtime_error("joint type not handled");
}
}

#endif //DRAKE_JOINTS_H_H
