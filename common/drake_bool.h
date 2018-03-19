#pragma once

#include <type_traits>

#include "drake/common/drake_copyable.h"

namespace drake {

/// Class representing a formula whose variables are T-typed.
/// For T = `double` and `AutoDiffXd`, this class includes a `bool` value.
/// For T = `symbolic::Expression`, a `symbolic::Formula` is embedded.
///
/// A value of this class is *not* contextually convertible to bool. As a result
/// the following code does not compile.
///
/// @code
/// const bool_t<double> b{3.0 < 4.0};
/// if (b) {
///   ...
/// }
/// @endcode
///
/// To convert a value of this class to `bool`, one needs to explicitly call
/// `ExtractBoolOrThrow` defined below in this file.
///
/// TODO(soonho-tri): Make DRAKE_ASSERT/DRAKE_DEMAND and cond compatible with
/// bool_t.
template <typename T>
class bool_t {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(bool_t);

  using formula_t = decltype(T() < T());

  /// Constructs with @p value.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  bool_t(const formula_t& value) : value_{value} {}

  /// Constructs with a Boolean @p b.
  ///
  /// @note This constructor is only enabled if `formula_t` is not `bool` in
  /// which case the above constructor, `bool_t(const formula_t& value)`, is
  /// used instead.
  template <typename = std::enable_if<!std::is_same<formula_t, bool>::value>>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  bool_t(bool b)
      : value_{b ? !(T(0) < T(0)) /* True */ : T(0) < T(0) /* False */} {}

  /// Returns a copy of its value.
  formula_t value() const { return value_; }

 private:
  formula_t value_;
};

/// Extracts a bool value from @p b or throws an exception if the extraction
/// fails.
template <typename T>
bool ExtractBoolOrThrow(const bool_t<T>& b) {
  return bool{b.value()};
}

}  // namespace drake
