#ifndef DRAKE_JOINT_H
#define DRAKE_JOINT_H

#include <random>
#include <memory>
#include "drake/util/TypeConversions.h"
#include "drake/util/drakeGeometryUtil.h"
#include "drake/systems/plants/joints/QuaternionFloating.h"
#include "drake/systems/plants/joints/RollPitchYawFloating.h"
#include "drake/systems/plants/joints/Revolute.h"
#include "drake/systems/plants/joints/Prismatic.h"
#include "drake/systems/plants/joints/Helical.h"
#include "drake/systems/plants/joints/Fixed.h"

namespace Drake {

template <typename J>
class Joint {
 public:
  enum FloatingBaseType { FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2 };

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
    return name + type->getPositionNamePostfix(index);
  }
  inline std::string getVelocityName(int index) const {
    return name + type->getVelocityNamePostfix(index);
  }
  inline bool isFloating() const { return type->isFloating(); };
  inline Eigen::VectorXd zeroConfiguration() const { return type->zeroConfiguration(); };
  inline Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const { return type->randomConfiguration(generator); };
  inline const Eigen::VectorXd &getJointLimitMin() const { return type->getJointLimitMin(); };
  inline const Eigen::VectorXd &getJointLimitMax() const { return type->getJointLimitMax(); };

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    using Q = typename DerivedQ::Scalar;

    auto revoluteJoint = dynamic_cast<const Revolute<J>*>(type.get());
    if (revoluteJoint) {
      return revoluteJoint->jointTransform(q);
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->jointTransform(q);
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template jointTransform<Q>();
    }
    auto prismaticJoint = dynamic_cast<const Prismatic<J>*>(type.get());
    if (prismaticJoint) {
      return prismaticJoint->jointTransform(q);
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->jointTransform(q);
    }
    auto helicalJoint = dynamic_cast<const Helical<J>*>(type.get());
    if (helicalJoint) {
      return helicalJoint->jointTransform(q);
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ>
  MotionSubspace<Promote<J, typename DerivedQ::Scalar>> motionSubspace(const Eigen::MatrixBase<DerivedQ> &q) const {
    using Q = typename DerivedQ::Scalar;

    auto fixedAxisOneDoFJoint = dynamic_cast<const FixedAxisOneDoF<J>*>(type.get());
    if (fixedAxisOneDoFJoint) {
      return fixedAxisOneDoFJoint->template motionSubspace<Q>();
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template motionSubspace<Q>();
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template motionSubspace<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->motionSubspace(q);
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ, typename DerivedV>
  SpatialVector<Promote<J, typename DerivedQ::Scalar>> motionSubspaceDotTimesV(const Eigen::MatrixBase<DerivedQ> &q, const Eigen::MatrixBase<DerivedV> &v) const {
    using Q = typename DerivedQ::Scalar;

    auto fixedAxisOneDoFJoint = dynamic_cast<const FixedAxisOneDoF<J>*>(type.get());
    if (fixedAxisOneDoFJoint) {
      return fixedAxisOneDoFJoint->template motionSubspaceDotTimesV<Q>();
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template motionSubspaceDotTimesV<Q>();
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template motionSubspaceDotTimesV<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->motionSubspaceDotTimesV(q, v);
    }
    throw std::runtime_error("joint type not handled");
  };

  template <typename DerivedQ>
  ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;

    auto fixedAxisOneDoFJoint = dynamic_cast<const FixedAxisOneDoF<J>*>(type.get());
    if (fixedAxisOneDoFJoint) {
      return fixedAxisOneDoFJoint->template configurationDerivativeToVelocity<Q>();
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->configurationDerivativeToVelocity(q);
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template configurationDerivativeToVelocity<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->template configurationDerivativeToVelocity<Q>();
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedQ>
  VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;

    auto fixedAxisOneDoFJoint = dynamic_cast<const FixedAxisOneDoF<J>*>(type.get());
    if (fixedAxisOneDoFJoint) {
      return fixedAxisOneDoFJoint->template velocityToConfigurationDerivative<Q>();
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->velocityToConfigurationDerivative(q);
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template velocityToConfigurationDerivative<Q>();
    }
    auto rollPitchYawFloatingJoint = dynamic_cast<const RollPitchYawFloating<J>*>(type.get());
    if (rollPitchYawFloatingJoint) {
      return rollPitchYawFloatingJoint->template velocityToConfigurationDerivative<Q>();
    }
    throw std::runtime_error("joint type not handled");
  }

  template <typename DerivedV>
  Eigen::Matrix<Promote<J, typename DerivedV::Scalar>, Eigen::Dynamic, 1> frictionTorque(const Eigen::MatrixBase<DerivedV>& v) const {
    using V = typename DerivedV::Scalar;

    auto fixedAxisOneDoFJoint = dynamic_cast<const FixedAxisOneDoF<J>*>(type.get());
    if (fixedAxisOneDoFJoint) {
      return fixedAxisOneDoFJoint->frictionTorque(v);
    }
    auto quaternionFloatingJoint = dynamic_cast<const QuaternionFloating<J>*>(type.get());
    if (quaternionFloatingJoint) {
      return quaternionFloatingJoint->template frictionTorque<V>();
    }
    auto fixedJoint = dynamic_cast<const Fixed<J>*>(type.get());
    if (fixedJoint) {
      return fixedJoint->template frictionTorque<V>();
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

using DrakeJoint = Drake::Joint<double>; // for now

#endif //DRAKE_JOINT_H
