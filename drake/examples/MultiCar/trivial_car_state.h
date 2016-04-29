#pragma once

#include <stdexcept>
#include <string>
#include <Eigen/Core>

namespace Drake {

template <typename ScalarType> // = double>
class TrivialCarState {
 public:
  // Required by Drake::Vector concept.
  static const int RowsAtCompileTime = 3;
  typedef Eigen::Matrix<ScalarType, RowsAtCompileTime, 1> EigenType;

  TrivialCarState() {
    vector_ << 0., 0., 0.;
  }

  TrivialCarState(const double x, const double y, const double heading) {
    vector_ << x, y, heading;
  }

  // Required by Drake::Vector concept.
  template <typename Derived>
  // NOLINTNEXTLINE(runtime/explicit)
  TrivialCarState(const Eigen::MatrixBase<Derived>& initial)
      : vector_(initial) {}

  // Required by Drake::Vector concept.
  template <typename Derived>
  TrivialCarState& operator=(const Eigen::MatrixBase<Derived>& rhs) {
    vector_ = rhs.vector_;
    return *this;
  }

  // Required by Drake::Vector concept.
  friend EigenType toEigen(const TrivialCarState<ScalarType>& vec) {
    return vec.vector_;
  }

 private:
  EigenType vector_;
};

}  // namespace Drake
