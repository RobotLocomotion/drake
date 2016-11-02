#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/lcmt_quadrotor_output_t.hpp"

template <typename ScalarType = double>
class QuadrotorOutput {
 public:
    static const int num_lidar_points = 100;
    static const int RowsAtCompileTime = 22 + num_lidar_points + 1;

    typedef drake::lcmt_quadrotor_output_t LCMMessageType;
    static std::string channel() { return "QUAD_OUTPUT"; }

    QuadrotorOutput(void) : rangefinder(0) {
        position.setZero();
        orientation.setZero();
        twist.setZero();
        accelerometer.setZero();
        gyroscope.setZero();
        magnetometer.setZero();
        lidar_returns.setZero();
    }

    template <typename Derived>
    QuadrotorOutput(  // NOLINT(runtime/explicit) per drake::Vector.
        const Eigen::MatrixBase<Derived>& x) {
      fromEigen<Derived>(x);
    }

    template <typename Derived>
    QuadrotorOutput& operator=(const Eigen::MatrixBase<Derived>& x) {
      fromEigen<Derived>(x);
      return *this;
    }

    friend Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> toEigen(
            const QuadrotorOutput<ScalarType>& vec) {
        Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> x =
            Eigen::Matrix<ScalarType, RowsAtCompileTime, 1>::Zero();

        x.segment<3>(0) = vec.position;
        x.segment<4>(3) = vec.orientation;
        x.segment<6>(7) = vec.twist;
        x.segment<3>(13) = vec.accelerometer;
        x.segment<3>(16) = vec.gyroscope;
        x.segment<3>(19) = vec.magnetometer;
        x.segment<num_lidar_points>(22) = vec.lidar_returns;
        x[num_lidar_points + 22] = vec.rangefinder;
        return x;
    }

    template <typename Derived>
    void fromEigen(const Eigen::MatrixBase<Derived>& x) {
        position = x.template segment<3>(0);
        orientation = x.template segment<4>(3);
        twist = x.template segment<6>(7);
        accelerometer = x.template segment<3>(13);
        gyroscope = x.template segment<3>(16);
        magnetometer = x.template segment<3>(19);
        lidar_returns = x.template segment<num_lidar_points>(22);
        rangefinder = x[num_lidar_points + 22];
    }

    friend std::string getCoordinateName(const QuadrotorOutput<ScalarType>& vec,
                                         unsigned int index) {
      static const std::vector<std::string> coordinate_names =
          {"x", "y", "z",
           "qx", "qy", "qz", "qw",
           "xdot", "ydot", "zdot",
           "angvel_x", "angvel_y", "angvel_z",
           "accel_x", "accel_y", "accel_z",
           "gyro_x", "gyro_y", "gyro_z",
           "mag_x", "mag_y", "mag_z"
          };
      if (index >= 22) return "lidar_" + std::to_string(index - 22);
      return (index >= 0 && index < RowsAtCompileTime) ? coordinate_names[index]
                                                       : std::string("error");
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<ScalarType, 3, 1> position;
    Eigen::Matrix<ScalarType, 4, 1> orientation;
    Eigen::Matrix<ScalarType, 6, 1> twist;
    Eigen::Matrix<ScalarType, 3, 1> accelerometer;
    Eigen::Matrix<ScalarType, 3, 1> gyroscope;
    Eigen::Matrix<ScalarType, 3, 1> magnetometer;
    Eigen::Matrix<ScalarType, num_lidar_points, 1> lidar_returns;
    ScalarType rangefinder;
};

bool encode(
    const double& t, const QuadrotorOutput<double>& x,
    // NOLINTNEXTLINE(runtime/references) This code will be deleted soon.
    drake::lcmt_quadrotor_output_t& msg) {
  msg.timestamp = static_cast<int64_t>(t*1000);
  Eigen::Map<Eigen::Vector3d> lcm_position(msg.position);
  Eigen::Map<Eigen::Vector4d> lcm_orientation(msg.orientation);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> lcm_twist(msg.twist);
  Eigen::Map<Eigen::Vector3d> lcm_accelerometer(msg.accelerometer);
  Eigen::Map<Eigen::Vector3d> lcm_gyroscope(msg.gyroscope);
  Eigen::Map<Eigen::Vector3d> lcm_magnetometer(msg.magnetometer);

  Eigen::Map<
    Eigen::Matrix<double, QuadrotorOutput<double>::num_lidar_points, 1>
    > lcm_lidar(msg.lidar_returns);
  lcm_position = x.position;
  lcm_orientation = x.orientation;
  lcm_twist = x.twist;
  lcm_accelerometer = x.accelerometer;
  lcm_gyroscope = x.gyroscope;
  lcm_magnetometer = x.magnetometer;
  lcm_lidar = x.lidar_returns;
  msg.rangefinder = x.rangefinder;
  return true;
}
