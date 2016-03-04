#ifndef DRAKE_QUADROTORSTATE_H
#define DRAKE_QUADROTORSTATE_H

#include <Eigen/Dense>
#include "lcmtypes/drake/lcmt_quadrotor_state_t.hpp"

template <typename ScalarType = double>
class QuadrotorState {
public:

    const static int RowsAtCompileTime = 19;
    typedef drake::lcmt_quadrotor_state_t LCMMessageType;
    static std::string channel() { return "QUAD_STATE"; };

    QuadrotorState(void) {
        position.setZero();
        orientation.setZero();
        twist.setZero();
        accelerometer.setZero();
        gyroscope.setZero();
    }

    template <typename Derived>
    QuadrotorState(const Eigen::MatrixBase<Derived>& x) {
      fromEigen<Derived>(x);
    };

    template <typename Derived>
    QuadrotorState& operator=(const Eigen::MatrixBase<Derived>& x) {
      fromEigen<Derived>(x);
      return *this;
    }

    friend Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> toEigen(const QuadrotorState<ScalarType>& vec) {
        Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> x = Eigen::Matrix<ScalarType, RowsAtCompileTime, 1>::Zero();

        x.segment<3>(0) = vec.position;
        x.segment<4>(3) = vec.orientation;
        x.segment<6>(7) = vec.twist;
        x.segment<3>(13) = vec.accelerometer;
        x.segment<3>(16) = vec.gyroscope;

        return x;
    }

    template <typename Derived>
    void fromEigen(const Eigen::MatrixBase<Derived>& x) {
        position = x.template segment<3>(0);
        orientation = x.template segment<4>(3);
        twist = x.template segment<6>(7);
        accelerometer = x.template segment<3>(13);
        gyroscope = x.template segment<3>(16);
    }

    friend std::string getCoordinateName(const QuadrotorState<ScalarType>& vec, unsigned int index) {
      static const std::vector<std::string> coordinate_names =
          {"x","y","z",
           "qx","qy","qz","qw",
           "xdot", "ydot", "zdot",
           "angvel_x", "angvel_y", "angvel_z",
           "accel_x", "accel_y", "accel_z",
           "gyro_x", "gyro_y", "gyro_z"
          };
      return index >= 0 && index < RowsAtCompileTime ? coordinate_names[index] : std::string("error");
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<ScalarType, 3, 1> position;
    Eigen::Matrix<ScalarType, 4, 1> orientation;
    Eigen::Matrix<ScalarType, 6, 1> twist;
    Eigen::Matrix<ScalarType, 3, 1> accelerometer;
    Eigen::Matrix<ScalarType, 3, 1> gyroscope;
};

bool encode(const double& t, const QuadrotorState<double> & x, drake::lcmt_quadrotor_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  Eigen::Map<Eigen::Vector3d> lcm_position(msg.position);
  Eigen::Map<Eigen::Vector4d> lcm_orientation(msg.orientation);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> lcm_twist(msg.twist);
  Eigen::Map<Eigen::Vector3d> lcm_accelerometer(msg.accelerometer);
  Eigen::Map<Eigen::Vector3d> lcm_gyroscope(msg.gyroscope);
  lcm_position = x.position;
  lcm_orientation = x.orientation;
  lcm_twist = x.twist;
  lcm_accelerometer = x.accelerometer;
  lcm_gyroscope = x.gyroscope;
  return true;
}

#endif
