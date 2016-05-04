#ifndef DRAKE_MULTIBODY_DYNAMICS_JOINTS_JOINT_H
#define DRAKE_MULTIBODY_DYNAMICS_JOINTS_JOINT_H

#include <random>
#include "drake/util/drakeGeometryUtil.h"


namespace drake {

// enum class FloatingBaseType { kFixed = 0, kRollPitchYaw = 1, kQuaternion = 2 };


const int kMaxNumJointVelocities = 6;
const int kMaxNumJointPositions = 7;

// TODO: move to a better place
template <typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

template <typename Scalar>
using Transform3D = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

template <typename Scalar>
using SpatialVector = Eigen::Matrix<Scalar, TWIST_SIZE, 1>;

template <typename Scalar>
using MotionSubspaceType = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, kMaxNumJointVelocities>;

template <typename Scalar>
using ConfigurationDerivativeToVelocityType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxNumJointVelocities, kMaxNumJointPositions>;

template <typename Scalar>
using VelocityToConfigurationDerivativeType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxNumJointPositions, kMaxNumJointVelocities>;
// end move to better place

// TODO: find place to implement joint limits

template <typename Scalar>
class Joint {
 public:
  Joint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body, int num_positions, int num_velocities, bool is_floating) :
      name_(name), transform_to_parent_body_(transform_to_parent_body), num_positions_(num_positions), num_velocities_(num_velocities), is_floating_(is_floating) {
    assert(num_positions <= kMaxNumJointPositions);
    assert(num_velocities <= kMaxNumJointVelocities);
  }

  int GetNumPositions() const { return num_positions_; }

  int GetNumVelocities() const { return num_velocities_; }

  const std::string &GetName() const { return name_; };

  const Transform3D<Scalar> &GetTransformToParentBody() const { return transform_to_parent_body_; };

  std::string GetPositionName(int index) const {
    return name_ + GetPositionNamePostfix(index);
  }

  std::string GetVelocityName(int index) const {
    return name_ + GetVelocityNamePostfix(index);
  }

  bool IsFloating() const { return is_floating_; };

  virtual std::string GetPositionNamePostfix(int index) const = 0;

  virtual std::string GetVelocityNamePostfix(int index) const {
    return GetPositionNamePostfix(index) + "dot";
  }

  virtual VectorX<double> ZeroConfiguration() const {
    return Eigen::VectorXd::Zero(GetNumPositions());
  }

  virtual VectorX<double> RandomConfiguration(std::default_random_engine &generator) const = 0;

  virtual Transform3D<Scalar> JointTransform(const Eigen::Ref<const VectorX<Scalar>> &q) const = 0;

  virtual MotionSubspaceType<Scalar> MotionSubspace(const Eigen::Ref<const VectorX<Scalar>> &q) const = 0;

  virtual SpatialVector<Scalar> MotionSubspaceDotTimesV(const Eigen::Ref<const VectorX<Scalar>> &q, const Eigen::Ref<const VectorX<Scalar>> &v) const = 0;

  virtual ConfigurationDerivativeToVelocityType<Scalar> ConfigurationDerivativeToVelocity(const Eigen::Ref<const VectorX<Scalar>> &q) const = 0;

  virtual VelocityToConfigurationDerivativeType<Scalar> VelocityToConfigurationDerivative(const Eigen::Ref<const VectorX<Scalar>> &q) const = 0;

  virtual VectorX<Scalar> FrictionTorque(const Eigen::Ref<const VectorX<Scalar>> &v) const = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(Transform3D<Scalar>)%16)==0)

 private:
  const std::string name_;
  const Transform3D<Scalar> transform_to_parent_body_;
  const int num_positions_;
  const int num_velocities_;
  const bool is_floating_;
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINTS_JOINT_H
