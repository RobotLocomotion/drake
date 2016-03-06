#ifndef DRAKE_QUADROTORCONTROL_H
#define DRAKE_QUADROTORCONTROL_H

#include <Eigen/Dense>
#include "lcmtypes/drake/lcmt_quadrotor_control_t.hpp"

template <typename ScalarType = double>
class QuadrotorControl {
public:
    typedef drake::lcmt_quadrotor_control_t LCMMessageType;
    static std::string channel() { return "QUAD_CONTROL"; };

    QuadrotorControl(void) : motors(Eigen::Vector4d::Zero()) {}

    template <typename Derived>
    QuadrotorControl(const Eigen::MatrixBase<Derived>& x) : motors(x) {};

    template <typename Derived>
    QuadrotorControl& operator=(const Eigen::MatrixBase<Derived>& x) {
      motors = x;
      return *this;
    }

    friend Eigen::Vector4d toEigen(const QuadrotorControl<ScalarType>& vec) {
      return vec.motors;
    }

    friend std::string getCoordinateName(const QuadrotorControl<ScalarType>& vec, unsigned int index) {
      return index >= 0 && index < RowsAtCompileTime ? std::string("motor") + std::to_string(index + 1) : std::string("error");
    }

    const static int RowsAtCompileTime = 4;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<ScalarType, 4, 1> motors;
};

bool decode(const drake::lcmt_quadrotor_control_t& msg, double& t, QuadrotorControl<double>& x) {
  t = double(msg.timestamp)/1000.0;
  x.motors = Eigen::Vector4d(msg.motors);
  return true;
}

#endif //DRAKE_SUPERBUILD_QUADROTORCONTROL_H
