#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

/** @addtogroup technical_notes
@{
@defgroup serialize_tips Writing a Serialize method

Structured data sometimes provides a Serialize method to be compatible with a
variety of readers, writers, or any other code that needs to visit the data
generically.

Here is an example of implementing a Serialize method:
@code
struct DoubleStruct {
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(value));
  }

  double value{0.0};
};
@endcode

By convention, we place the Serialize method prior to the data members per
<a href="https://drake.mit.edu/styleguide/cppguide.html#Declaration_Order">the
styleguide rule</a>.  Each data member has a matching `Visit` line in the
Serialize method, in the same order as the member fields appear.

By convention, we declare all of the member fields as public, since they are
effectively so anyway (because anything that calls the Serialize method
receives a mutable pointer to them).  The typical way to do this is to declare
the data as a `struct`, instead of a `class`.

However, if
<a href="https://drake.mit.edu/styleguide/cppguide.html#Structs_vs._Classes">the
styleguide rule</a> for struct vs class points towards using a `class` instead,
then we follow that advice and make it a `class`, but we explicitly label the
member fields as `public`.  We also omit the trailing underscore from the field
names, so that the Serialize API presented to the caller of the class is
indifferent to whether it is phrased as a `struct` or a `class`.

For how Serialize and Archive interact, see the drake::yaml::YamlReadArchive
class overview.

@}
*/

namespace drake {

/// A basic implementation of the Name-Value Pair concept as used in the
/// Serialize / Archive pattern.  See, for example:
/// https://www.boost.org/doc/libs/release/libs/serialization/doc/wrappers.html#nvp
///
/// NameValue stores a pointer to a const `name` and a pointer to a mutable
/// `value`.  Both pointers must remain valid throughout the lifetime of an
/// object.  %NameValue objects are typically short-lived, existing only for a
/// transient moment while an Archive is visiting some Serializable field.
template <typename T>
class NameValue {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(NameValue)

  /// Type of the referenced value.
  typedef T value_type;

  /// (Advanced.)  Constructs a %NameValue.  Prefer DRAKE_NVP instead of this
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

/// (Advanced.)  Creates a NameValue. The conventional method for calling this
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
