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
using MotionSubspace = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, kMaxNumJointVelocities>;

template <typename Scalar>
using ConfigurationDerivativeToVelocity = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxNumJointVelocities, kMaxNumJointPositions>;

template <typename Scalar>
using VelocityToConfigurationDerivative = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxNumJointPositions, kMaxNumJointVelocities>;
// end move to better place

// TODO: find place to implement joint limits

template <typename Scalar>
class Joint {
 public:
  Joint(const std::string &name, const Transform3D<Scalar> &transform_to_parent_body, int num_positions, int num_velocities, bool is_floating) :
      name(name), transform_to_parent_body(transform_to_parent_body), num_positions(num_positions), num_velocities(num_velocities), is_floating(is_floating) {
    assert(num_positions <= kMaxNumJointPositions);
    assert(num_velocities <= kMaxNumJointVelocities);
  }

  int getNumPositions() const { return num_positions; }

  int getNumVelocities() const { return num_velocities; }

  const std::string &getName() const { return name; };

  const Transform3D<Scalar> &getTransformToParentBody() const { return transform_to_parent_body; };

  std::string getPositionName(int index) const {
    return name + getPositionNamePostfix(index);
  }

  std::string getVelocityName(int index) const {
    return name + getVelocityNamePostfix(index);
  }

  bool isFloating() const { return is_floating; };

  virtual std::string getPositionNamePostfix(int index) const = 0;

  virtual std::string getVelocityNamePostfix(int index) const {
    return getPositionName(index) + "dot";
  }

  virtual VectorX<double> zeroConfiguration() const {
    return Eigen::VectorXd::Zero(getNumPositions());
  }

  virtual VectorX<double> randomConfiguration(std::default_random_engine &generator) const = 0;

  virtual Transform3D<Scalar> jointTransform(const Eigen::Ref<VectorX<Scalar>> &q) const = 0;

  virtual MotionSubspace<Scalar> motionSubspace(const Eigen::Ref<VectorX<Scalar>> &q) const = 0;

  virtual SpatialVector<Scalar> motionSubspaceDotTimesV(const Eigen::Ref<VectorX<Scalar>> &q, const Eigen::Ref<VectorX<Scalar>> &v) const = 0;

  virtual ConfigurationDerivativeToVelocity<Scalar> configurationDerivativeToVelocity(const Eigen::Ref<VectorX<Scalar>> &q) const = 0;

  virtual VelocityToConfigurationDerivative<Scalar> velocityToConfigurationDerivative(const Eigen::Ref<VectorX<Scalar>> &q) const = 0;

  virtual VectorX<Scalar> frictionTorque(const Eigen::Ref<VectorX<Scalar>> &v) const = 0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((sizeof(Transform3D<Scalar>)%16)==0)

 private:
  const std::string name;
  const Transform3D<Scalar> transform_to_parent_body;
  const int num_positions;
  const int num_velocities;
  const bool is_floating;
};

}

#endif //DRAKE_MULTIBODY_DYNAMICS_JOINTS_JOINT_H
