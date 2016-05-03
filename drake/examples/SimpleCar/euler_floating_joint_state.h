// Copyright 2016 Robot Locomotion Group @ CSAIL. All rights reserved.
#pragma once

// This file is generated by a script.  Do not edit!
// See drake/examples/SimpleCar/lcm_vector_gen.py.

#include <stdexcept>
#include <string>
#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_euler_floating_joint_state_t.hpp"

namespace Drake {

/// Describes the row indices of a EulerFloatingJointState.
struct EulerFloatingJointStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordiates = 6;

  // The index of each individual coordinate.
  static const int kX = 0;
  static const int kY = 1;
  static const int kZ = 2;
  static const int kRoll = 3;
  static const int kPitch = 4;
  static const int kYaw = 5;
};

/// Models the Drake::LCMVector concept.
template <typename ScalarType = double>
class EulerFloatingJointState {
 public:
  // An abbreviation for our row index constants.
  typedef EulerFloatingJointStateIndices K;

  /// @name Getters and Setters
  //@{
  const ScalarType& x() const { return value_(K::kX); }
  void set_x(const ScalarType& x) { value_(K::kX) = x; }
  const ScalarType& y() const { return value_(K::kY); }
  void set_y(const ScalarType& y) { value_(K::kY) = y; }
  const ScalarType& z() const { return value_(K::kZ); }
  void set_z(const ScalarType& z) { value_(K::kZ) = z; }
  const ScalarType& roll() const { return value_(K::kRoll); }
  void set_roll(const ScalarType& roll) { value_(K::kRoll) = roll; }
  const ScalarType& pitch() const { return value_(K::kPitch); }
  void set_pitch(const ScalarType& pitch) { value_(K::kPitch) = pitch; }
  const ScalarType& yaw() const { return value_(K::kYaw); }
  void set_yaw(const ScalarType& yaw) { value_(K::kYaw) = yaw; }
  //@}

  /// @name Implement the Drake::Vector concept.
  //@{

  // Even though in practice we have a fixed size, we declare
  // ourselves dynamically sized for compatibility with the
  // system/framework/vector_interface.h API, and so that we
  // can avoid the alignment issues that come into play with
  // a fixed-size Eigen::Matrix member field.
  static const int RowsAtCompileTime = Eigen::Dynamic;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;
  size_t size() const { return K::kNumCoordiates; }

  /// Default constructor.  Sets all rows to zero.
  EulerFloatingJointState()
      : value_(Eigen::Matrix<ScalarType, K::kNumCoordiates, 1>::Zero()) {}

  /// Implicit Eigen::Matrix conversion.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit) per Drake::Vector.
  EulerFloatingJointState(const Eigen::MatrixBase<Derived>& value)
      : value_(value.segment(0, K::kNumCoordiates)) {}

  /// Eigen::Matrix assignment.
  template <typename Derived>
  EulerFloatingJointState& operator=(const Eigen::MatrixBase<Derived>& value) {
    value_ = value.segment(0, K::kNumCoordiates);
    return *this;
  }

  /// Magic conversion specialization back to Eigen.
  friend EigenType toEigen(const EulerFloatingJointState<ScalarType>& vec) {
    return vec.value_;
  }

  /// Magic pretty names for our coordinates.  (This is an optional
  /// part of the Drake::Vector concept, but seems worthwhile.)
  friend std::string getCoordinateName(
      const EulerFloatingJointState<ScalarType>& vec, unsigned int index) {
    switch (index) {
      case K::kX:
        return "x";
      case K::kY:
        return "y";
      case K::kZ:
        return "z";
      case K::kRoll:
        return "roll";
      case K::kPitch:
        return "pitch";
      case K::kYaw:
        return "yaw";
    }
    throw std::domain_error("unknown coordinate index");
  }

  //@}

  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_euler_floating_joint_state_t LCMMessageType;
  static std::string channel() { return "EULER_FLOATING_JOINT_STATE"; }
  //@}

 private:
  EigenType value_;
};

template <typename ScalarType>
bool encode(const double& t, const EulerFloatingJointState<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_euler_floating_joint_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.x = wrap.x();
  msg.y = wrap.y();
  msg.z = wrap.z();
  msg.roll = wrap.roll();
  msg.pitch = wrap.pitch();
  msg.yaw = wrap.yaw();
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_euler_floating_joint_state_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            EulerFloatingJointState<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_x(msg.x);
  wrap.set_y(msg.y);
  wrap.set_z(msg.z);
  wrap.set_roll(msg.roll);
  wrap.set_pitch(msg.pitch);
  wrap.set_yaw(msg.yaw);
  return true;
}

}  // namespace Drake
