#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace geometry {

/*
Defines RGBA (red-blue-green-alpha) values on the range [0..1].
*/
class Rgba {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rgba);

  /*
  Constructs with given (r, g, b, a) values.
  @pre All values are within the range of [0..1].
  */
  Rgba(double r, double g, double b, double a = 1.) {
    set(r, g, b, a);
  }

  /*
  Constructs with given (r, g, b, a) values from an Eigen vector.
  @pre All values are within the range of [0..1].
  */
  explicit Rgba(const Eigen::Vector4d& v) {
    set(v);
  }

  /* Red. */
  double r() const { return r_; }

  /* Green. */
  double g() const { return g_; }

  /* Blue. */
  double b() const { return b_; }

  /* Alpha. */
  double a() const { return a_; }

  /*
  Sets (r, g, b, a) values.
  @pre All values are within the range of [0..1].
  */
  void set(double r, double g, double b, double a = 1.) {
    DRAKE_THROW_UNLESS(r >= 0 && r <= 1);
    DRAKE_THROW_UNLESS(g >= 0 && g <= 1);
    DRAKE_THROW_UNLESS(b >= 0 && b <= 1);
    DRAKE_THROW_UNLESS(a >= 0 && a <= 1);
    r_ = r;
    g_ = g;
    b_ = b;
    a_ = a;
  }

  /*
  Sets (r, g, b, a) values from an Eigen vector.
  @pre All values are within the range of [0..1].
  */
  void set(const Eigen::Vector4d& v) {
    set(v[0], v[1], v[2], v[3]);
  }

  /*
  Returns (r, g, b, a) as an Eigen vector.
  */
  Eigen::Vector4d AsVector4() const {
    return Eigen::Vector4d(r_, g_, b_, a_);
  }

  bool operator==(const Rgba& other) const {
    return
        r_ == other.r_ && g_ == other.g_ && b_ == other.b_ && a_ == other.a_;
  }

  bool operator!=(const Rgba& other) const {
    return !(*this == other);
  }

 private:
  double r_{};
  double g_{};
  double b_{};
  double a_{};
};

}  // namespace geometry
}  // namespace drake
