#ifndef DRAKE_QUADROTORSTATE_H
#define DRAKE_QUADROTORSTATE_H

#include <Eigen/Dense>
#include "lcmtypes/drake/lcmt_quadrotor_state_t.hpp"

template <typename ScalarType = double>
class QuadrotorState {
public:

    typedef drake::lcmt_quadrotor_state_t LCMMessageType;
    static std::string channel() { return "QUAD_STATE"; };

    QuadrotorState(void) {
      state.setZero();
    }

    template <typename Derived>
    QuadrotorState(const Eigen::MatrixBase<Derived>& x) : state(x) {};

    template <typename Derived>
    QuadrotorState& operator=(const Eigen::MatrixBase<Derived>& x) {
      state = x;
      return *this;
    }

    friend Eigen::Matrix<ScalarType, 13, 1> toEigen(const QuadrotorState<ScalarType>& vec) {
      return vec.state;
    }

    friend std::string getCoordinateName(const QuadrotorState<ScalarType>& vec, unsigned int index) {
      static const std::vector<std::string> coordinate_names =
          {"x","y","z",
           "qw","qx","qy","qz",
           "xdot", "ydot", "zdot",
           "angvel_x", "angvel_y", "angvel_z"
          };
      return index >= 0 && index < RowsAtCompileTime ? coordinate_names[index] : std::string("error");
    }

    const static int RowsAtCompileTime = 13;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<ScalarType, 13, 1> state;
};

bool encode(const double& t, const QuadrotorState<double> & x, drake::lcmt_quadrotor_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  for(std::size_t i = 0; i < QuadrotorState<double>::RowsAtCompileTime; i++) {
    msg.x[i] = x.state[i];
  }
  return true;
}

#endif
