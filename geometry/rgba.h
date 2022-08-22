#pragma once

#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/** Defines RGBA (red, green, blue, alpha) values on the range [0, 1].  */
class Rgba {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rgba);

  /** Default constructor produces fully opaque white. */
  Rgba() = default;

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
          "Rgba values must be within the range [0, 1]. Values provided: "
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

  /** Passes this object to an Archive.

   In YAML, an %Rgba is represented by an array-like list of three or four
   numbers. E.g.,

        rgba: [0.5, 0.5, 1.0]

   or

        rgba: [0.5, 0.5, 1.0, 0.5]

   such that the first three values are red, green, and blue, respectively. If
   no fourth value is provided, alpha is defined a 1.0.

   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    /* N.B. This "strange" spelling is to enable the following in YAML:
        - rgba: [r, g, b]
      or
        - rgba: [r, g, b, a]
     Rather than enumerating the fields, we treat the Rgba like an array-like
     construct. Validation is done *explicitly* here on the *size* of the array;
     the validity of the individual values is handled by Rgba::set(). */
    Eigen::VectorXd rgba = Eigen::Vector4d{r_, g_, b_, a_};
    a->Visit(MakeNameValue("rgba", &rgba));
    if (rgba.size() == 3) {
      set(rgba[0], rgba[1], rgba[2]);
    } else if (rgba.size() == 4) {
      set(rgba[0], rgba[1], rgba[2], rgba[3]);
    } else {
      throw std::runtime_error(fmt::format(
          "Rgba must contain either 3 or 4 elements (given [{}])",
          rgba.transpose()));
    }
  }

 private:
  // As documented above, default-constructed Rgba is opaque white.
  double r_{1};
  double g_{1};
  double b_{1};
  double a_{1};
};

}  // namespace geometry
}  // namespace drake
