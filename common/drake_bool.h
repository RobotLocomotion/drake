#pragma once

#include <type_traits>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {

/// Class representing a boolean value independent of the underlying
/// scalar type T:
///  - For `double` or autodiff, this class embeds a `bool` value.
///  - For `symbolic::Expression`, this class embeds a `symbolic::Formula`
///    value.
///
/// A value of this class is *not* contextually convertible to bool. To convert
/// a value to `bool`, one needs to explicitly call `ExtractBoolOrThrow` defined
/// below in this file. Here is an example use-case:
///
/// @code
/// const Bool<double> b{3.0 < 4.0};
/// if (ExtractBoolOrThrow(b)) {
///   ...
/// }
/// @endcode
///
/// In contrast, the following code does not compile:
///
/// @code
/// const Bool<double> b{3.0 < 4.0};
/// if (b) {
///   ...
/// }
/// @endcode
///
/// TODO(soonho-tri): Make cond compatible with Bool.
template <typename T>
class Bool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bool);

  using value_type = decltype(T() < T());

  /// Constructs with @p value.
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Bool(const value_type& value) : value_{value} {}

  /// Constructs with a Boolean @p b.
  ///
  /// @note This constructor is only enabled if `value_type` is not `bool` in
  /// which case the above constructor, `Bool(const value_type& value)`, is
  /// used instead.
  template <typename = std::enable_if<!std::is_same<value_type, bool>::value>>
  // NOLINTNEXTLINE(runtime/explicit) This conversion is desirable.
  Bool(bool b)
      : value_{b ? !(T(0) < T(0)) /* True */ : T(0) < T(0) /* False */} {}

  /// Returns a copy of its value.
  value_type value() const { return value_; }

 private:
  value_type value_{};
};

/// Extracts a bool value from @p b or throws an exception if the extraction
/// fails (i.e. due to free variables in a symbolic::Formula).
template <typename T>
bool ExtractBoolOrThrow(const Bool<T>& b) {
  return bool{b.value()};
}

namespace assert {
/// Provides the specialization of ConditionTraits for `Bool<T>` so that a value
/// of Bool<T> can be passed to DRAKE_ASSERT/DRAKE_DEMAND macros.
template <typename T>
struct ConditionTraits<Bool<T>> {
  static constexpr bool is_valid =
      ConditionTraits<typename Bool<T>::value_type>::is_valid;
  static bool Evaluate(const Bool<T>& b) {
    return ConditionTraits<typename Bool<T>::value_type>::Evaluate(b.value());
  }
};
}  // namespace assert
}  // namespace drake
