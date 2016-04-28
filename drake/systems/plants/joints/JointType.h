#ifndef DRAKE_JOINTTYPE_H_H
#define DRAKE_JOINTTYPE_H_H

namespace Drake {

// TODO: move to better place
const int MAX_NUM_JOINT_VELOCITIES = 6;
const int MAX_NUM_JOINT_POSITIONS = 7;

template <typename Scalar>
using Transform3D = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

template <typename Scalar>
using SpatialVector = Eigen::Matrix<Scalar, TWIST_SIZE, 1>;
// end move to better place

enum class FloatingBaseType { FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2 };

template <typename Scalar>
using MotionSubspace = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, MAX_NUM_JOINT_VELOCITIES>;

template <typename Scalar>
using ConfigurationDerivativeToVelocity = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_JOINT_VELOCITIES, MAX_NUM_JOINT_POSITIONS>;

template <typename Scalar>
using VelocityToConfigurationDerivative = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_JOINT_POSITIONS, MAX_NUM_JOINT_VELOCITIES>;


template <typename J>
class JointType {
 public:
  virtual ~JointType() {};
};

}

#endif //DRAKE_JOINTTYPE_H_H
