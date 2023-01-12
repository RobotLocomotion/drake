#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"

namespace drake {
namespace geometry {

/** Defines RGBA (red, green, blue, alpha) values on the range [0, 1]. */
class Rgba {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rgba);

  /** Default constructor produces fully opaque white. */
  Rgba() = default;

  /** Constructs with given (r, g, b, a) values.
   @pre All values are within the range of [0, 1]. */
  Rgba(double r, double g, double b, double a = 1.0) {
    set(r, g, b, a);
  }

  /** Red. */
  double r() const { return value_[0]; }

  /** Green. */
  double g() const { return value_[1]; }

  /** Blue. */
  double b() const { return value_[2]; }

  /** Alpha. */
  double a() const { return value_[3]; }

  /** Returns all four elements in order. */
  Eigen::Vector4d rgba() const { return value_; }

  /** Sets (r, g, b, a) values.
   @throws std::exception if any values are outside of the range [0, 1]. */
  void set(double r, double g, double b, double a = 1.0) {
    set(Eigen::Vector4d{r, g, b, a});
  }

  /** Sets an (r, g, b, a) from a vector.
   @throws std::exception if the vector is not size 3 or 4.
   @throws std::exception if any values are outside of the range [0, 1]. */
  void set(const Eigen::Ref<const Eigen::VectorXd>& rgba);

  /** Reports if two %Rgba values are equal within a given absolute `tolerance`.
   They are "equal" so long as the difference in no single channel is larger
   than the specified `tolerance`. */
  bool AlmostEqual(const Rgba& other, double tolerance = 0.0) const {
    return std::abs(r() - other.r()) <= tolerance &&
           std::abs(g() - other.g()) <= tolerance &&
           std::abs(b() - other.b()) <= tolerance &&
           std::abs(a() - other.a()) <= tolerance;
  }

  bool operator==(const Rgba& other) const {
    return value_ == other.value_;
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
    // N.B. This spelling is to enable the following in YAML:
    //  - rgba: [r, g, b]
    // or
    //  - rgba: [r, g, b, a]
    //
    // N.B. For the Python binding use of DefAttributesUsingSerialize, this
    // must refer a member field; we cannot use a stack temporary here.
    a->Visit(MakeNameValue("rgba", &value_));

    // In case the archive modified our data, we need to re-validate it now.
    // We want to be sure that value_ remains well-formed even if set() throws,
    // so we'll clear it and then reset it.
    auto new_value = value_;
    value_ = Eigen::Vector4d::Zero();
    set(new_value);
  }

 private:
  void ThrowIfOutOfRange() const;

  // We'll store the `r, g, b, a` doubles in an Eigen::Vector. Normally we'd
  // use an `Eigen::Vector4d` for this purpose, but for compatibility with YAML
  // parsing of either a 3-tuple (rgb) or 4-tuple (rgba), we need allow for a
  // dynamic number of vector elements, and then carefully write our code to
  // ensure that `value_.size()` is always exactly `4` as a runtime invariant.
  // We'll also set the template argument `MaxRowsAtCompileTime == 4` so that
  // our data is stored inline as part of this object, instead of on the heap.
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 4, 1> value_ =
      // As documented above, default-constructed Rgba is opaque white.
      Eigen::Vector4d::Ones();
};

}  // namespace geometry
}  // namespace drake
