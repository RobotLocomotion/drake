#pragma once

#include <cmath>

namespace drake {
namespace maliput {
namespace monolane {


class V3 {
 public:
  V3(double xx, double yy, double zz) : x_(xx), y_(yy), z_(zz) {}

  double norm() const {
    return std::sqrt((x_ * x_) + (y_ * y_) + (z_ * z_));
  }

  V3 operator*(const double rhs) const {
    return V3(x_ * rhs, y_ * rhs, z_ * rhs);
  }

  double x() const { return x_; }

  double y() const { return y_; }

  double z() const { return z_; }

  friend V3 operator+(const V3& a, const V3& b);

 private:
  double x_{};
  double y_{};
  double z_{};
};

inline
V3 operator+(const V3& a, const V3& b) {
  return V3(a.x_ + b.x_,
            a.y_ + b.y_,
            a.z_ + b.z_);
}


class V2 {
 public:
  V2(double xx, double yy) : x_(xx), y_(yy) {}

  double norm() const {
    return std::sqrt((x_ * x_) + (y_ * y_));
  }

  double x() const { return x_; }

  double y() const { return y_; }

 private:
  double x_{};
  double y_{};
};


class Rot3 {
 public:
  Rot3(double yy, double pp, double rr) : yaw_(yy), pitch_(pp), roll_(rr) {}

  V3 apply(const V3& in) const {
    const double sa = std::sin(roll_);
    const double ca = std::cos(roll_);
    const double sb = std::sin(pitch_);
    const double cb = std::cos(pitch_);
    const double sg = std::sin(yaw_);
    const double cg = std::cos(yaw_);

    return V3(
        ((cb * cg) * in.x()) +
        ((-ca*sg + sa*sb*cg) * in.y()) +
        ((sa*sg + ca*sb*cg) * in.z()),

        ((cb*sg) * in.x()) +
        ((ca*cg + sa*sb*sg) * in.y()) +
        ((-sa*cg + ca*sb*sg) * in.z()),

        ((-sb) * in.x()) +
        ((sa*cb) * in.y()) +
        ((ca*cb) * in.z()));
  }

  double yaw() const { return yaw_; }

  double pitch() const { return pitch_; }

  double roll() const { return roll_; }

 private:
  double yaw_{};
  double pitch_{};
  double roll_{};
};


// parameterized on domain p in [0, 1]
class CubicPolynomial {
 public:
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  CubicPolynomial(double a, double b, double c, double d)
      : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    // TODO(maddog)  For now, completely bogus linear distance
    //               from (0,0) to (1,df):
    s_1_ = std::sqrt(1. + (df * df));
  }

  double f_p(double p) const {
    return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p);
  }

  double fdot_p(double p) const {
    return b_ + (2. * c_ * p) + (3. * d_ * p * p);
  }

  double fddot_p(double p) const {
    return (2. * c_) + (6. * d_ * p);
  }


  // TODO(maddog)  Perform a proper arc-length parameterization!
  double s_p(double p) const {
    return s_1_ * p;
  }

  double fake_gprime(double p) const {
    // return df;  which is...
    return f_p(1.) - f_p(0.);
  }

  double p_s(double s) const {
    return s / s_1_;
  }

  double f_s(double s) const {
    return f_p(p_s(s));
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};

}  // namespace monolane
}  // namespace maliput
}  // namespace drake
