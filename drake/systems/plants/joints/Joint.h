#ifndef DRAKE_JOINT_H
#define DRAKE_JOINT_H

#include <random>
#include "drake/core/Conversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/systems/plants/joints/QuaternionFloating.h"
#include "drake/systems/plants/joints/RollPitchYawFloating.h"
#include "drake/systems/plants/joints/FixedAxisOneDoF.h"

namespace Drake {

template <typename J>
class Joint {
 private:
  const std::string name;
  const Transform3D<J> transform_to_parent_body;
  const std::unique_ptr<JointType<J>> type;

 public:
  Joint(const std::string &name, const Transform3D<J> &transform_to_parent_body, std::unique_ptr<JointType<J>> type) :
      name(name), transform_to_parent_body(transform_to_parent_body), type(std::move(type)) { };

  inline int getNumPositions() const { return type->getNumPositions(); }
  inline int getNumVelocities() const { return type->getNumVelocities(); }
  inline const std::string &getName() const { return name; };
  inline const Transform3D<J> &getTransformToParentBody() const { return transform_to_parent_body; };
  inline std::string getPositionName(int index) const {
    return name + "_" + type->getPositionName(index);
  }
  inline std::string getVelocityName(int index) const {
    return name + "_" + type->getVelocityName(index);
  }
  inline bool isFloating() const { return type->isFloating(); };
  inline Eigen::VectorXd zeroConfiguration() const { return type->zeroConfiguration(); };
  inline Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const { return type->randomConfiguration(generator); };
  inline const Eigen::VectorXd &getJointLimitMin() const { return type->getJointLimitMin(); };
  inline const Eigen::VectorXd &getJointLimitMax() const { return type->getJointLimitMax(); };

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) {
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->jointTransform(q);
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->jointTransform(q);
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ>
  MotionSubspace<Promote<J, typename DerivedQ::Scalar>> motionSubspace(const Eigen::MatrixBase<DerivedQ> &q) {
    using Q = typename DerivedQ::Scalar;

    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template motionSubspace<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->motionSubspace(q);
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ, typename DerivedV>
  SpatialVector<Promote<J, typename DerivedQ::Scalar>> motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v) {
    using Q = typename DerivedQ::Scalar;

    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template motionSubspaceDotTimesV<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->motionSubspaceDotTimesV(q);
    }
    // TODO
  };

  template <typename DerivedQ>
  ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Eigen::MatrixBase<DerivedQ>& q) {
    using Q = typename DerivedQ::Scalar;

    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->configurationDerivativeToVelocity(q);
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->template configurationDerivativeToVelocity<Q>();
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ>
  VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Eigen::MatrixBase<DerivedQ>& q) {
    using Q = typename DerivedQ::Scalar;

    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->velocityToConfigurationDerivative(q);
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->template velocityToConfigurationDerivative<Q>();
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedV>
  Eigen::Matrix<Promote<J, typename DerivedV::Scalar>, Eigen::Dynamic, 1> frictionTorque(const Eigen::MatrixBase<DerivedV>& v) {
    using V = typename DerivedV::Scalar;

    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template frictionTorque<V>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->template frictionTorque<V>();
    }
    throw std::runtime_error("joint type not handled");
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(Transform3D<J>)%16)==0)
};

}

#endif //DRAKE_JOINT_H
