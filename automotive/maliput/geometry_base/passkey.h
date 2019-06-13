#pragma once

namespace drake {
namespace maliput {
namespace geometry_base {

/// Simple generic implementation of the "Passkey Idiom".
///
/// Passkey<T> allows a class to provide method-level friendship to another
/// class.  By tagging an otherwise `public` method with a `Passkey<Other>`
/// parameter, only the class `Other` will be able to construct the required
/// passkey and call the method (typically using just `{}` to construct the
/// passkey instance at the call site).
///
/// @see https://arne-mertz.de/2016/10/passkey-idiom/
/// @see https://stackoverflow.com/questions/3324898/can-we-increase-the-re-usability-of-this-key-oriented-access-protection-pattern
template <class T>
class DRAKE_DEPRECATED_AUTOMOTIVE
    Passkey {
 private:
  // Only T may construct a Passkey<T>!
  friend T;
  // NB: These trivial methods *must not* be "= default", otherwise C++'s
  //     "uniform initialization" will allow anyone to aggregate-construct
  //     an instance, not just T.
  Passkey() {}
  Passkey(const Passkey&) {}
  Passkey& operator=(const Passkey&) = delete;
};

}  // namespace geometry_base
}  // namespace maliput
}  // namespace drake
