#pragma once

#include <Eigen/Core>

namespace drake {

/// Simple Drake::Vector-concept class which captures the parameters of
/// a DrakeJoint::ROLLPITCHYAW (Euler angle) floating joint.
template <typename ScalarType>
class XyzRpy { // models Drake::Vector
 public:
  static const int RowsAtCompileTime = 6;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  XyzRpy() {}

  XyzRpy(const ScalarType& x, const ScalarType& y, const ScalarType& z,
         const ScalarType& roll, const ScalarType& pitch,
         const ScalarType& yaw) {
    vector_ << x, y, z, roll, pitch, yaw;
  }

  template <typename Derived>
  XyzRpy(const Eigen::MatrixBase<Derived>& initial) : vector_(initial) {}

  template <typename Derived>
  XyzRpy& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    vector_ = rhs.vector_;
    return *this;
  }

  friend EigenType toEigen(const XyzRpy<ScalarType>& xyz_rpy) {
    return xyz_rpy.vector_;
  }

 private:
  EigenType vector_;
};
} // namespace drake
