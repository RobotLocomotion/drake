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

enum class FloatingBaseType { FIXED = 0, ROLLPITCHYAW = 1, QUATERNION = 2 };

template <typename Scalar>
using MotionSubspace = Eigen::Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic, 0, TWIST_SIZE, MAX_NUM_JOINT_VELOCITIES>;

template <typename Scalar>
using ConfigurationDerivativeToVelocity = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_JOINT_VELOCITIES, MAX_NUM_JOINT_POSITIONS>;

template <typename Scalar>
using VelocityToConfigurationDerivative = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_NUM_JOINT_POSITIONS, MAX_NUM_JOINT_VELOCITIES>;
// end move to better place


template <typename J>
class JointType {
 private:
  int num_positions;
  int num_velocities;
  Eigen::Matrix<J, Eigen::Dynamic, 1> joint_limit_min;
  Eigen::Matrix<J, Eigen::Dynamic, 1> joint_limit_max;

 public:
  JointType(int num_positions, int num_velocities) :
      num_positions(num_positions),
      num_velocities(num_velocities),
      joint_limit_min(Eigen::Matrix<J, Eigen::Dynamic, 1>::Constant(num_positions, -std::numeric_limits<double>::infinity())),
      joint_limit_max(Eigen::Matrix<J, Eigen::Dynamic, 1>::Constant(num_positions, std::numeric_limits<double>::infinity())) {
    assert(num_positions <= MAX_NUM_JOINT_POSITIONS);
    assert(num_velocities <= MAX_NUM_JOINT_VELOCITIES);
  }
  virtual ~JointType() {};

  inline int getNumPositions() const { return num_positions; };

  inline int getNumVelocities() const { return num_velocities; };

  inline const Eigen::Matrix<J, Eigen::Dynamic, 1> &getJointLimitMin() const { return joint_limit_min;}

  inline const Eigen::Matrix<J, Eigen::Dynamic, 1> &getJointLimitMax() const { return joint_limit_max;}

  virtual Eigen::VectorXd zeroConfiguration() const { return Eigen::VectorXd::Zero(num_positions); };

  virtual Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const = 0;

  virtual std::string getPositionName(int index) const = 0;

  virtual std::string getVelocityName(int index) const {
    return getPositionName(index) + "dot";
  }

  virtual bool isFloating() const { return false; }

};

}

#endif //DRAKE_JOINTTYPE_H_H
