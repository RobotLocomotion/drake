#pragma once

#include "drake/common/drake_copyable.h"

namespace anzu {

/// A basic implementation of the NameValuePair concept as used in the
/// Serialize/Archive pattern.
template <typename T>
class NameValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NameValue)

  typedef T value_type;

  /// Both pointers are aliased for the lifetime of this class.
  NameValue(const char* name_in, T* value_in)
      : name_(name_in), value_(value_in) {}

  const char* name() const { return name_; }
  T* value() const { return value_; }
  const T& get_value() const { return *value_; }
  void set_value(const T& value_in) const { *value_ = value_in; }

 private:
  const char* const name_;
  T* const value_;
};

/// (Advanced.)  Creates a NameValue. The conventional method for calling this
/// function is the ANZU_NVP sugar macro below.
///
/// Both pointers are aliased for the lifetime of the return value.
template <typename T>
NameValue<T> MakeNameValue(const char* name, T* value) {
  return NameValue<T>(name, value);
}

}  // namespace anzu

// Creates a NameValue for an lvalue `x`.
#define ANZU_NVP(x) ::anzu::MakeNameValue(#x, &(x))
