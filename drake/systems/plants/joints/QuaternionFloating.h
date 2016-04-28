#ifndef DRAKE_QUATERNIONFLOATING_H
#define DRAKE_QUATERNIONFLOATING_H

#include "drake/systems/plants/joints/JointType.h"

namespace Drake {


template <typename J>
class QuaternionFloating : public JointType<J> {
 public:
  using JointType<J>::getNumPositions;
  using JointType<J>::getNumVelocities;

  QuaternionFloating() : JointType<J>(7, 6) { };

  virtual inline Eigen::VectorXd zeroConfiguration() const override {
    Eigen::VectorXd ret(getNumPositions());
    ret << 0, 0, 0, 1, 0, 0, 0;
    return ret;
  }

  virtual inline Eigen::VectorXd randomConfiguration(std::default_random_engine &generator) const override {
    Eigen::VectorXd q(getNumPositions());
    std::normal_distribution<double> normal;

    // position
    q[0] = normal(generator);
    q[1] = normal(generator);
    q[2] = normal(generator);

    // orientation
    Eigen::Vector4d quat = uniformlyRandomQuat(generator);
    q[3] = quat(0);
    q[4] = quat(1);
    q[5] = quat(2);
    q[6] = quat(3);
    return q;
  }

  virtual inline std::string getPositionName(int index) const override {
    switch (index) {
      case 0:
        return "x";
      case 1:
        return "y";
      case 2:
        return "z";
      case 3:
        return "qw";
      case 4:
        return "qx";
      case 5:
        return "qy";
      case 6:
        return "qz";
      default:
        throw std::runtime_error("bad index");
    }
  }

  virtual inline std::string getVelocityName(int index) const override {
    switch (index) {
      case 0:
        return "wx";
      case 1:
        return "wy";
      case 2:
        return "wz";
      case 3:
        return "vx";
      case 4:
        return "vy";
      case 5:
        return "vz";
      default:
        throw std::runtime_error("bad index");
    }
  }

  virtual inline bool isFloating() const override { return true; }

  template <typename DerivedQ>
  Transform3D<Promote<J, typename DerivedQ::Scalar>> jointTransform(const Eigen::MatrixBase<DerivedQ> &q) const {
    using T = Promote<J, typename DerivedQ::Scalar>;
    Transform3D<T> ret;
    ret.linear() = ConvertMatrix<T>()(quat2rotmat(q.template bottomRows<4>()));
    ret.translation() << Convert<T>()(q[0]), Convert<T>()(q[1]), Convert<T>()(q[2]);
    ret.makeAffine();
    return ret;
  }

  template <typename Q>
  MotionSubspace<Promote<J, Q>> motionSubspace() const {
    return MotionSubspace<Promote<J, Q>>::Identity(TWIST_SIZE, getNumVelocities());
  }

  template <typename Q>
  SpatialVector<Promote<J, Q>> motionSubspaceDotTimesV() const {
    return SpatialVector<Promote<J, Q>>::Zero();
  };

  template <typename DerivedQ>
  ConfigurationDerivativeToVelocity<Promote<J, typename DerivedQ::Scalar>> configurationDerivativeToVelocity(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    ConfigurationDerivativeToVelocity<T> ret(getNumVelocities(), getNumPositions());
    auto quat = ConvertMatrix<T>()(q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION));
    auto R = quat2rotmat(quat);

    Eigen::Matrix<T, 4, 1> quattilde;
    typename Gradient<Eigen::Matrix<T, 4, 1>, QUAT_SIZE, 1>::type dquattildedquat;
    normalizeVec(quat, quattilde, &dquattildedquat);
    auto RTransposeM = (R.transpose() * quatdot2angularvelMatrix(quat)).eval();
    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 4>(0, 3).noalias() = RTransposeM * dquattildedquat;
    ret.template block<3, 3>(3, 0) = R.transpose();
    ret.template block<3, 4>(3, 3).setZero();
    return ret;
  }

  template <typename DerivedQ>
  VelocityToConfigurationDerivative<Promote<J, typename DerivedQ::Scalar>> velocityToConfigurationDerivative(const Eigen::MatrixBase<DerivedQ>& q) const {
    using Q = typename DerivedQ::Scalar;
    using T = Promote<J, Q>;

    VelocityToConfigurationDerivative<T> ret(getNumPositions(), getNumVelocities());
    auto quat = ConvertMatrix<T>()(q.template middleRows<QUAT_SIZE>(SPACE_DIMENSION));
    auto R = quat2rotmat(quat);

    Eigen::Matrix<T, QUAT_SIZE, SPACE_DIMENSION> M;
    typename Gradient<decltype(M), QUAT_SIZE, 1>::type* dM = nullptr;
    angularvel2quatdotMatrix(quat, M, dM);

    ret.template block<3, 3>(0, 0).setZero();
    ret.template block<3, 3>(0, 3) = R;
    ret.template block<4, 3>(3, 0).noalias() = M * R;
    ret.template block<4, 3>(3, 3).setZero();
    return ret;
  }

  template <typename V>
  Eigen::Matrix<Promote<J, V>, Eigen::Dynamic, 1> frictionTorque() const {
    return Eigen::Matrix<V, Eigen::Dynamic, 1>::Zero(getNumVelocities(), 1);
  }
};

}

#endif //DRAKE_QUATERNIONFLOATING_H
