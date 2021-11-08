#pragma once

#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** Defines RGBA (red, green, blue, alpha) values on the range [0, 1].  */
class Rgba {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rgba);

  /** Constructs with given (r, g, b, a) values.
   @pre All values are within the range of [0, 1].  */
  Rgba(double r, double g, double b, double a = 1.) {
    set(r, g, b, a);
  }

  /** Red.  */
  double r() const { return r_; }

  /** Green.  */
  double g() const { return g_; }

  /** Blue.  */
  double b() const { return b_; }

  /** Alpha.  */
  double a() const { return a_; }

  /** Sets (r, g, b, a) values.
   @pre All values are within the range of [0, 1]. The values are not updated
   if this precondition is not met.  */
  void set(double r, double g, double b, double a = 1.) {
    if ((r < 0 || r > 1) ||
        (g < 0 || g > 1) ||
        (b < 0 || b > 1) ||
        (a < 0 || a > 1)) {
      throw std::runtime_error(fmt::format(
          "All values must be within the range [0, 1]. Values provided: "
          "(r={}, g={}, b={}, a={})", r, g, b, a));
    }
    r_ = r;
    g_ = g;
    b_ = b;
    a_ = a;
  }

  /* Reports if two %Rgba values are equal within a given absolute `tolerance`.
   They are "equal" so long as the difference in no single channel is larger
   than the specified `tolerance`. */
  bool AlmostEqual(const Rgba& other, double tolerance = 0.0) const {
    return std::abs(r_ - other.r_) <= tolerance &&
           std::abs(g_ - other.g_) <= tolerance &&
           std::abs(b_ - other.b_) <= tolerance &&
           std::abs(a_ - other.a_) <= tolerance;
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
