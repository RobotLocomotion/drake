#ifndef DRAKE_EXAMPLES_QUADROTOR_QUADROTORINPUT_H_
#define DRAKE_EXAMPLES_QUADROTOR_QUADROTORINPUT_H_

#include <Eigen/Dense>
#include "lcmtypes/drake/lcmt_quadrotor_input_t.hpp"

template <typename ScalarType = double>
class QuadrotorInput {
 public:
    typedef drake::lcmt_quadrotor_input_t LCMMessageType;
    static std::string channel() { return "QUAD_CONTROL"; }

    QuadrotorInput(void) : motors(Eigen::Vector4d::Zero()) {}

    template <typename Derived>
    QuadrotorInput(const Eigen::MatrixBase<Derived>& x) : motors(x) {}

    template <typename Derived>
    QuadrotorInput& operator=(const Eigen::MatrixBase<Derived>& x) {
      motors = x;
      return *this;
    }

    friend Eigen::Vector4d toEigen(const QuadrotorInput<ScalarType>& vec) {
      return vec.motors;
    }

    friend std::string getCoordinateName(const QuadrotorInput<ScalarType>& vec, unsigned int index) {
      return index >= 0 && index < RowsAtCompileTime ? std::string("motor") + std::to_string(index + 1) : std::string("error");
    }

    static const int RowsAtCompileTime = 4;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<ScalarType, 4, 1> motors;
};

bool decode(const drake::lcmt_quadrotor_input_t& msg, double& t, QuadrotorInput<double>& x) {
  t = double(msg.timestamp)/1000.0;
  x.motors = Eigen::Vector4d(msg.motors);
  return true;
}

#endif  // DRAKE_EXAMPLES_QUADROTOR_QUADROTORINPUT_H_
