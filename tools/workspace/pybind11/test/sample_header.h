#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"

/// Root-level symbol.
struct RootLevelSymbol {};

namespace drake {

/**
 * Mid-level symbol.
 * Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut
 * aliquip ex ea commodo consequat.
 */
struct MidLevelSymbol {};

namespace mkdoc_test {

/// Function.
void func();
/// Function, overload 1.
void func(int);
/// Function, template overload.
template <typename T>
void func(T);

/// Class.
/// Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod
/// tempor incididunt ut labore et dolore magna aliqua.
class Class {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class)

  // TODO(eric.cousineau): Make `mkdoc` recognize documentation for aliases. At
  // present, neither `UsingAlias` nor `TypedefAlias` show up.

  /// Using alias.
  using UsingAlias = std::vector<Class>;

  /// Typedef alias.
  typedef std::vector<Class> TypedefAlias;

  /// Custom constructor 1.
  Class() {}

  /// Custom constructor 2.
  explicit Class(int) {}

  /// Custom constructor 3.
  explicit Class(int, std::vector<int>) {}

  /// Public method.
  void PublicMethod() {}

  /// Public template method.
  template <typename T>
  void PublicTemplateMethod() {}

  /// Static method.
  static void PublicStatic() {}

 protected:
  /// Protected method.
  void ProtectedMethod() {}

  /// Protected nested class.
  class Nested {};

  /// Protected member.
  int protected_member_{};

 private:
  /// Private method, public documentation.
  void PrivateMethod() {}

  // Private member, private documentation.
  int private_member_{};
};

/// Struct.
struct Struct {
  /// Field 1.
  int field_1;
  /// Field 2.
  std::vector<int> field_2;
};

/// Template class.
template <typename T>
class TemplateClass {
 public:
  /// Default constructor.
  TemplateClass() {}
};

/// Specialize.
template <>
class TemplateClass<int> {
};

/// Enumeration.
enum Enum {
  /// Enumeration constant.
  EnumConstant,
};

/// Enumeration class.
enum EnumClass {
  /// Enumeration class constant.
  EnumClassConstant,
};

/// Anonymous values are ignored.
enum {
  /// Anonymous enum's constant.
  AnonymousConstant,
};

}  // namespace mkdoc_test
}  // namespace drake
