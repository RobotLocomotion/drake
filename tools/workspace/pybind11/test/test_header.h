#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/tools/workspace/pybind11/test/test_dep_header.h"

/// Root-level symbol.
struct RootLevelSymbol {};

namespace drake {

/// Mid-level symbol.
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
/// tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim
/// veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea
/// commodo consequat.
class Class {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Class);

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
  // Private method.
  void PrivateMethod() {}

  // Private member.
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
class Template {
 public:
  /// Default constructor.
  Template() {}
};

/// Specialize.
template <>
class Template<int> {
};

/// Enumeration.
enum Enum {
  /// Enumeration constant.
  Constant,
};

/// Anonymous values are ignored.
enum {
  /// Anonymous enum's constant.
  AnonymousConstant,
};

}  // namespace mkdoc_test
}  // namespace drake
