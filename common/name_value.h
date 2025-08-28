#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {

/// (Advanced) A basic implementation of the Name-Value Pair concept as used in
/// the Serialize / Archive pattern.
///
/// %NameValue stores a pointer to a const `name` and a pointer to a mutable
/// `value`.  Both pointers must remain valid throughout the lifetime of an
/// object.  %NameValue objects are typically short-lived, existing only for a
/// transient moment while an Archive is visiting some Serializable field.
///
/// For more information, refer to Drake's
/// @ref yaml_serialization "YAML Serialization" and especially the
/// @ref implementing_serialize "Implementing Serialize" section, or also the
/// <a
/// href="https://www.boost.org/doc/libs/release/libs/serialization/doc/wrappers.html#nvp">Boost
/// Name-Value Pairs</a> documentation for background.
template <typename T>
class NameValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NameValue);

  /// Type of the referenced value.
  typedef T value_type;

  /// (Advanced) Constructs a %NameValue.  Prefer DRAKE_NVP instead of this
  /// constructor.  Both pointers are aliased and must remain valid for the
  /// lifetime of this object.  Neither pointer can be nullptr.
  NameValue(const char* name_in, T* value_in)
      : name_(name_in), value_(value_in) {
    DRAKE_ASSERT(name_in != nullptr);
    DRAKE_ASSERT(value_in != nullptr);
  }

  /// @name Accessors to the raw pointers
  //@{
  const char* name() const { return name_; }
  T* value() const { return value_; }
  //@}

 private:
  const char* const name_;
  T* const value_;
};

/// (Advanced) Creates a NameValue. The conventional method for calling this
/// function is the DRAKE_NVP sugar macro below.
///
/// Both pointers are aliased for the lifetime of the return value.
template <typename T>
NameValue<T> MakeNameValue(const char* name, T* value) {
  return NameValue<T>(name, value);
}

}  // namespace drake

/// Creates a NameValue pair for an lvalue `x`.
#define DRAKE_NVP(x) ::drake::MakeNameValue(#x, &(x))
