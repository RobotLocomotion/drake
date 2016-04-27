#ifndef DRAKE_JOINT_H
#define DRAKE_JOINT_H

#include <random>
#include "drake/core/Conversions.h"
#include "drake/util/drakeGeometryUtil.h"

namespace Drake {

// TODO: move to better place
template <typename Scalar>
using Transform3D = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

template <typename Scalar>
using SpatialVector = Eigen::Matrix<Scalar, TWIST_SIZE, 1>;
// end move to better place

enum class FloatingBaseType { FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2 };

template <typename J>
class JointType;

template <typename J>
class Joint {
 public:
  static const int MAX_NUM_POSITIONS = 7;
  static const int MAX_NUM_VELOCITIES = 6;

 private:
  const std::string name;
  const Transform3D<J> transform_to_parent_body;
  const std::unique_ptr<JointType<J>> joint_type;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename Scalar>
using MotionSubspace = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, Joint<Scalar>::MAX_NUM_VELOCITIES>;

template <typename Scalar>
using ConfigurationDerivativeToVelocity = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, Joint<Scalar>::MAX_NUM_VELOCITIES,  Joint<Scalar>::MAX_NUM_POSITIONS>;

template <typename Scalar>
using VelocityToConfigurationDerivative = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, Joint<Scalar>::MAX_NUM_POSITIONS, Joint<Scalar>::MAX_NUM_VELOCITIES>;

}

#endif //DRAKE_JOINT_H
